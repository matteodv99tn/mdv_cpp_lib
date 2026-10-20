#include "mdv/mesh/point.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

#include <range/v3/all.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/half_edge.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

namespace mdv::mesh {
namespace rs = ::ranges;

namespace {
    template <typename Id>
    void
    require_member(const Mesh& mesh, Id id) {
        const auto& m = internal::cgal_mesh(mesh);
        const auto  i = internal::to_cgal(id);
        if (!id.is_valid() || !m.has_valid_index(i) || m.is_removed(i))
            throw std::runtime_error("Invalid point mesh handle");
    }

    void
    require_bary(const Vec3d& b) {
        if (!b.allFinite() || (b.array() < 0.0).any()
            || !condition::is_zero(b.sum() - 1.0))
            throw std::runtime_error("Invalid barycentric coordinates");
    }
}  // namespace

Point::Point(PointOnVertex d) : _d(d) {
}

Point::Point(PointOnEdge d) : _d(d) {
    if (!d.he.is_valid() || !std::isfinite(d.t) || d.t <= 0.0 || d.t >= 1.0)
        throw std::runtime_error("Invalid edge point descriptor");
}

Point::Point(PointInFace d) : _d(d) {
    const Vec3d b = d.coords();
    require_bary(b);
    if (!d.f.is_valid() || (b.array() < 1e-13).any())
        throw std::runtime_error("Expected strict face interior coordinates");
}

Point::Point(const Mesh& mesh, FaceId f, Vec3d b, bool classify_boundary) {
    require_member(mesh, f);
    require_bary(b);
    if (!classify_boundary) {
        *this = PointInFace{
                .f = f, .bary = {b[0], b[1], b[2]}
        };
        return;
    }
    const auto& m = internal::cgal_mesh(mesh);
    const auto  h = CGAL::halfedge(internal::to_cgal(f), m);
    // CGAL barycentrics follow source(h), target(h), target(next(h)).
    const std::array vs{
            CGAL::source(h, m), CGAL::target(h, m), CGAL::target(CGAL::next(h, m), m)
    };
    const auto zeros = (b.array().abs() < 1e-12).count();
    if (zeros == 2) {
        Eigen::Index i = 0;
        b.maxCoeff(&i);
        *this = PointOnVertex{internal::from_cgal(vs[static_cast<std::size_t>(i)])};
    } else if (zeros == 1) {
        const auto p = internal::convert(
                internal::CgalImpl::ShortestPath::point(
                        internal::to_cgal(f), {b[0], b[1], b[2]}, m
                )
        );
        *this = make_on_edge(mesh, f, p);
    } else {
        *this = PointInFace{
                .f = f, .bary = {b[0], b[1], b[2]}
        };
    }
}

bool
Point::is_undefined() const noexcept {
    const auto* v = get_as<PointOnVertex>();
    return v != nullptr && !v->v.is_valid();
}

CartesianPoint
position(const Mesh& mesh, const Point& p) {
    if (p.is_undefined()) throw std::runtime_error("Undefined point has no position");
    if (const auto* v = p.get_as<PointOnVertex>()) {
        require_member(mesh, v->v);
        return position(mesh, v->v);
    }
    const auto& m = internal::cgal_mesh(mesh);
    if (const auto* e = p.get_as<PointOnEdge>()) {
        require_member(mesh, e->he);
        const auto                              h = internal::to_cgal(e->he);
        const internal::CgalImpl::Kernel::Ray_3 ray(
                m.point(CGAL::source(h, m)), m.point(CGAL::target(h, m))
        );
        return internal::convert(ray.point(e->t));
    }
    const auto& f = *p.get_as<PointInFace>();
    require_member(mesh, f.f);
    return internal::convert(
            internal::CgalImpl::ShortestPath::point(internal::to_cgal(f.f), f.bary, m)
    );
}

FaceId
face(const Mesh& mesh, const Point& p) {
    if (p.is_undefined()) throw std::runtime_error("Undefined point has no face");
    if (const auto* f = p.get_as<PointInFace>()) {
        require_member(mesh, f->f);
        return f->f;
    }
    if (const auto* e = p.get_as<PointOnEdge>()) {
        require_member(mesh, e->he);
        return face(mesh, e->he);
    }
    const auto v = p.get_as<PointOnVertex>()->v;
    require_member(mesh, v);
    const auto& m = internal::cgal_mesh(mesh);
    return internal::from_cgal(CGAL::face(CGAL::halfedge(internal::to_cgal(v), m), m));
}

PointOnEdge
make_on_edge(const Mesh& mesh, HalfEdgeId he, const CartesianPoint& pos) {
    require_member(mesh, he);
    if (!pos.allFinite()) throw std::runtime_error("Nonfinite edge position");
    using Kernel  = internal::CgalImpl::Kernel;
    const auto& m = internal::cgal_mesh(mesh);
    const auto  h = internal::to_cgal(he);
    const auto  a = m.point(CGAL::source(h, m));
    const auto  b = m.point(CGAL::target(h, m));
    const auto  p = internal::point3_from_eigen(pos);
    if (CGAL::squared_distance(Kernel::Segment_3(a, b), p) >= 1e-18)
        throw std::runtime_error("Point does not lie on edge");
    const double t =
            std::sqrt(CGAL::squared_distance(a, p) / CGAL::squared_distance(a, b));
    PointOnEdge d{.he = he, .t = t};
    (void)Point{d};
    return d;
}

PointOnEdge
make_on_edge(const Mesh& mesh, FaceId f, const CartesianPoint& pos) {
    require_member(mesh, f);
    if (!pos.allFinite()) throw std::runtime_error("Nonfinite edge position");
    using Kernel  = internal::CgalImpl::Kernel;
    const auto& m = internal::cgal_mesh(mesh);
    const auto  p = internal::point3_from_eigen(pos);
    std::vector<internal::CgalImpl::CgalHalfEdgeIndex> hes;
    for (const auto h :
         CGAL::halfedges_around_face(CGAL::halfedge(internal::to_cgal(f), m), m))
        hes.push_back(h);
    const auto segment = [&](auto h) {
        return Kernel::Segment_3(
                m.point(CGAL::source(h, m)), m.point(CGAL::target(h, m))
        );
    };
    const auto h = rs::min(hes, std::less{}, [&](auto he) {
        return CGAL::squared_distance(segment(he), p);
    });
    return make_on_edge(
            mesh,
            internal::from_cgal(h),
            internal::convert(Kernel::Line_3(segment(h)).projection(p))
    );
}

PointOnEdge
flip(const Mesh& mesh, const PointOnEdge& e) {
    require_member(mesh, e.he);
    (void)Point{e};
    return {.he = twin(mesh, e.he), .t = 1.0 - e.t};
}

PointOnEdge
PointOnEdge::random(const Mesh& mesh) {
    static std::random_device rd;
    static std::mt19937       generator(rd());
    const auto&               m = internal::cgal_mesh(mesh);
    if (m.num_halfedges() == 0) throw std::runtime_error("Cannot sample empty mesh");
    std::uniform_int_distribution<unsigned> ids(0, m.num_halfedges() - 1);
    std::uniform_real_distribution<double>  coords(0.0, 1.0);
    PointOnEdge d{.he = HalfEdgeId{ids(generator)}, .t = coords(generator)};
    require_member(mesh, d.he);
    (void)Point{d};
    return d;
}

Point
from_cartesian(const Mesh& mesh, const CartesianPoint& pos) {
    if (!pos.allFinite() || mesh.num_faces() == 0)
        throw std::runtime_error("Cannot project point onto mesh");
    const auto& m     = internal::cgal_mesh(mesh);
    const auto  p     = internal::point3_from_eigen(pos);
    const auto [f, b] = internal::CgalImpl::ShortestPath::locate(
            p, mesh.cgal()._aabb_tree, m, get(CGAL::vertex_point, m)
    );
    const auto h     = CGAL::halfedge(f, m);
    const auto zeros = rs::count_if(b, [](double x) { return std::abs(x) < 1e-12; });
    if (zeros == 2) return PointOnVertex{closest_vertex(mesh, pos)};
    if (zeros == 1) return make_on_edge(mesh, internal::from_cgal(f), pos);
    for (const auto v : CGAL::vertices_around_face(h, m)) {
        if (CGAL::squared_distance(p, m.point(v)) < 1e-15) {
            return PointOnVertex{internal::from_cgal(v)};
        }
    }
    using Segment = internal::CgalImpl::Kernel::Segment_3;
    for (const auto he : CGAL::halfedges_around_face(h, m)) {
        if (CGAL::squared_distance(
                    p,
                    Segment(m.point(CGAL::source(he, m)), m.point(CGAL::target(he, m)))
            )
            < 1e-18) {
            return make_on_edge(mesh, internal::from_cgal(he), pos);
        }
    }
    return PointInFace{
            .f = internal::from_cgal(f), .bary = {b[0], b[1], b[2]}
    };
}

Point
random_point(const Mesh& mesh) {
    if (mesh.num_faces() == 0) throw std::runtime_error("Cannot sample empty mesh");
    static std::random_device rd;
    static std::mt19937       generator(rd());
    const auto                f = random_face(mesh);
    require_member(mesh, f);
    std::uniform_real_distribution<double> distribution(0.1, 0.9);
    const double                           b0 = distribution(generator);
    const double b1 = 0.95 * (1.0 - b0) * distribution(generator);
    return PointInFace{
            .f = f, .bary = {b0, b1, 1.0 - b0 - b1}
    };
}

bool
same_location(const Mesh& mesh, const Point& a, const Point& b) {
    return condition::is_zero_norm(position(mesh, a) - position(mesh, b));
}

std::string
describe(const Mesh& mesh, const Point& p) {
    const auto pos = eigen_to_str(position(mesh, p));
    if (const auto* v = p.get_as<PointOnVertex>())
        return fmt::format("Point on vertex #{} located at {}", v->v.value, pos);
    if (const auto* e = p.get_as<PointOnEdge>())
        return fmt::format("Point on halfedge #{} located at {}", e->he.value, pos);
    return fmt::format(
            "Point on face #{} located at {}", p.get_as<PointInFace>()->f.value, pos
    );
}
}  // namespace mdv::mesh
