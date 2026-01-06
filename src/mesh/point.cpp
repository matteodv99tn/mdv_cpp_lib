#include "mdv/mesh/point.hpp"

#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <cstdlib>
#include <Eigen/Core>
#include <random>

#include <range/v3/all.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
namespace rs = ::ranges;
namespace rv = ::ranges::views;

using mdv::mesh::CartesianPoint;
using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Point;
using mdv::mesh::internal::CgalImpl;

// \endcond


//  ____  _                  __     __        _
// |  _ \| |_    ___  _ __   \ \   / /__ _ __| |_ _____  __
// | |_) | __|  / _ \| '_ \   \ \ / / _ \ '__| __/ _ \ \/ /
// |  __/| |_  | (_) | | | |   \ V /  __/ |  | ||  __/>  <
// |_|    \__|  \___/|_| |_|    \_/ \___|_|   \__\___/_/\_\
//
std::string
Point::PointOnVertexDescriptor::describe() const {
    return fmt::format(
            "Point on vertex #{} located at {}", _v.id(), eigen_to_str(_v.position())
    );
};

Face
Point::PointOnVertexDescriptor::face() const {
    auto default_face = [this]() -> Face {
        const auto& m            = internal::get_mesh_impl(_v);
        const auto  v_id         = static_cast<CgalImpl::CgalVertexIndex>(_v.id());
        const auto  he           = CGAL::halfedge(v_id, m);
        static bool warn_printed = false;
        if (!warn_printed) {
            std::cerr
                    << "Warning: Point::face() called from point described on vertex\n";
            warn_printed = true;
        }
        return {_v.mesh(), CGAL::face(he, m)};
    };

    if (_f.has_value()) return _f.value();
    return default_face();
};

void
Point::PointOnVertexDescriptor::assign_face(const Face& f) {
    const auto& m     = internal::get_mesh_impl(f.mesh());
    const auto  f_id  = internal::to_face_impl(f);
    const auto  he_id = CGAL::halfedge(f_id, m);
    const auto  v_id  = internal::CgalImpl::CgalVertexIndex{_v.id()};

    const auto face_vertices =
            CGAL::vertices_around_face(he_id, internal::get_mesh_impl(f.mesh()))
            | rs::to_vector;
    if (!rs::contains(face_vertices, v_id)) {
        const std::string msg = fmt::format(
                "Face {} does not contain vertex #{}", f_id.idx(), v_id.idx()
        );
        throw std::runtime_error(msg.c_str());
    }

    _f = f;
}

//  ____  _                   _____    _
// |  _ \| |_    ___  _ __   | ____|__| | __ _  ___
// | |_) | __|  / _ \| '_ \  |  _| / _` |/ _` |/ _ \
// |  __/| |_  | (_) | | | | | |__| (_| | (_| |  __/
// |_|    \__|  \___/|_| |_| |_____\__,_|\__, |\___|
//                                       |___/

std::string
Point::PointOnEdgeDescriptor::describe() const {
    return fmt::format(
            "Point on halfedge #{} located at {}", _he.id(), eigen_to_str(position())
    );
};

Point::PointOnEdgeDescriptor::PointOnEdgeDescriptor(
        HalfEdge he, const CartesianPoint& position
) :
        _he(std::move(he)) {
    using Kernel      = CgalImpl::Kernel;
    const auto& m     = internal::get_mesh_impl(he);
    const auto  he_id = internal::to_halfedge_impl(_he);

    const Kernel::Point_3 he_origin = m.point(source(he_id, m));
    const Kernel::Point_3 he_end    = m.point(target(he_id, m));
    const Kernel::Point_3 pt        = internal::point3_from_eigen(position);

    const Kernel::Segment_3 edge(he_origin, he_end);
    assert(CGAL::squared_distance(edge, pt) < 1e-18);

    const double squared_d = CGAL::squared_distance(he_origin, pt)
                             / CGAL::squared_distance(he_origin, he_end);
    _c = std::sqrt(squared_d);
    assert((_c >= 0.0) && (_c <= 1.0));
    assert(mdv::condition::are_equal(position, this->position()));
}

CartesianPoint
Point::PointOnEdgeDescriptor::position() const noexcept {
    using Kernel = internal::CgalImpl::Kernel;

    const auto& m     = internal::get_mesh_impl(_he);
    const auto  he_id = internal::to_halfedge_impl(_he);

    const Kernel::Point_3 he_origin = m.point(source(he_id, m));
    const Kernel::Point_3 he_dest   = m.point(target(he_id, m));
    const Kernel::Ray_3   ray(he_origin, he_dest);
    return internal::convert(ray.point(_c));
}

Face
Point::PointOnEdgeDescriptor::face() const {
    const auto& m     = internal::get_mesh_impl(_he);
    const auto  he_id = internal::to_halfedge_impl(_he);
    return {_he.mesh(), CGAL::face(he_id, m)};
}

Point::PointOnEdgeDescriptor
Point::PointOnEdgeDescriptor::display_in_opposite_halfedge() const {
    const auto& m     = internal::get_mesh_impl(_he);
    const auto  he_id = internal::to_halfedge_impl(_he);
    return {
            HalfEdge{_he.mesh(), CGAL::opposite(he_id, m)},
            1.0 - _c
    };
}

Point::PointOnEdgeDescriptor
Point::PointOnEdgeDescriptor::random(const Mesh& mesh) {
    static std::random_device rand_dev;
    static std::mt19937       generator(rand_dev());

    const auto& m = internal::get_mesh_impl(mesh);
    m.num_halfedges();
    std::uniform_int_distribution<unsigned> id_distr(0, m.num_halfedges() - 1);
    std::uniform_real_distribution<double>  c_distr(0.0, 1.0);

    return {HalfEdge(mesh, id_distr(generator)), c_distr(generator)};
}

//  ____  _     _         _____
// |  _ \| |_  (_)_ __   |  ___|_ _  ___ ___
// | |_) | __| | | '_ \  | |_ / _` |/ __/ _ \
// |  __/| |_  | | | | | |  _| (_| | (_|  __/
// |_|    \__| |_|_| |_| |_|  \__,_|\___\___|
//

Point::PointInFaceDescriptor::PointInFaceDescriptor(
        Face face, Vec3d barycentric_coords
) :
        _f(std::move(face)), _b(std::move(barycentric_coords)) {
    using mdv::condition::is_zero;
    const double sum = _b.sum();
    const bool   describes_interior_point =
            is_zero(sum - 1.0) && _b(0) > 0.0 && _b(1) > 0.0 && _b(2) > 0.0;

    std::size_t zero_bs = 0;
    if (std::abs(_b(0)) < 1e-10) ++zero_bs;
    if (std::abs(_b(1)) < 1e-10) ++zero_bs;
    if (std::abs(_b(2)) < 1e-10) ++zero_bs;

    if (zero_bs == 1) throw std::runtime_error("Should have been edge descriptor");
    if (zero_bs == 2) throw std::runtime_error("Should have been vertex descriptor");

    if (is_undefined()) std::cerr << "Undefined face!\n";
    if (!describes_interior_point) {
        std::cerr << "b1 = " << _b(0) << "\n";
        std::cerr << "b2 = " << _b(1) << "\n";
        std::cerr << "b3 = " << _b(2) << "\n";
        std::cerr << "sum = " << _b.sum() << "\n";
    }

    assert(is_undefined() || describes_interior_point);
}

std::string
Point::PointInFaceDescriptor::describe() const {
    return fmt::format(
            "Point on face #{} located at {}", _f.id(), eigen_to_str(position())
    );
};

CartesianPoint
Point::PointInFaceDescriptor::position() const noexcept {
    const CgalImpl::ShortestPath::Barycentric_coordinates bs{_b(0), _b(1), _b(2)};
    return internal::convert(
            CgalImpl::ShortestPath::point(
                    internal::to_face_impl(_f), bs, internal::get_mesh_impl(_f)
            )
    );
}

Point::PointInFaceDescriptor
Point::PointInFaceDescriptor::from_cartesian(const Face& f, const Vec3d& position) {
    using BarycentricConstructor = CgalImpl::ShortestPathTraits::
            Construct_barycentric_coordinate_in_triangle_3;
    BarycentricConstructor bc;
    const auto             tri = internal::triangle3_from_face(f);
    const auto             pt  = internal::point3_from_eigen(position);
    const auto [b1, b2, b3]    = bc(tri, pt);
    return {
            f, {b1, b2, b3}
    };
}

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//

Point::PointInFaceDescriptor Point::undefined_point{Face::invalid_face, Vec3d::Zero()};

Point::Point() : _pt(undefined_point) {
}

Point
Point::from_cartesian(const Mesh& mesh, const CartesianPoint& cartesian_pt) {
    // TODO: handle the case in which the provided cartesian position is outside the
    // actual surface, and closest point lies on edge/vertex

    const auto&            m_impl = mesh.cgal()._mesh;
    const CgalImpl::Point3 point{cartesian_pt(0), cartesian_pt(1), cartesian_pt(2)};
    const auto [face_id, coords] = CgalImpl::ShortestPath::locate(
            point,
            mesh.cgal()._aabb_tree,
            m_impl,
            get(CGAL::vertex_point, internal::get_mesh_impl(mesh))
    );
    const Face& face = mesh.face(static_cast<Index>(face_id.idx()));
    const auto  f_he = CGAL::halfedge(face_id, m_impl);

    for (const auto v_id : CGAL::vertices_around_face(f_he, m_impl)) {
        if (CGAL::squared_distance(point, m_impl.point(v_id)) < 1e-15)
            return PointOnVertexDescriptor{Vertex(mesh, v_id)};
    }

    using Segment_3 = CgalImpl::Kernel::Segment_3;
    for (const auto he : CGAL::halfedges_around_face(f_he, m_impl)) {
        const auto p0 = m_impl.point(CGAL::source(he, m_impl));
        const auto p1 = m_impl.point(CGAL::target(he, m_impl));
        if (CGAL::squared_distance(point, Segment_3(p0, p1)) < 1e-18)
            return PointOnEdgeDescriptor{HalfEdge(mesh, he), cartesian_pt};
    }

    return PointInFaceDescriptor{
            face, {coords[0], coords[1], coords[2]}
    };
}

Point
Point::undefined(const Mesh& m) noexcept {
    return {undefined_point};
}

Point
Point::random(const Mesh& m) noexcept {
    static std::random_device rand_dev;
    static std::mt19937       generator(rand_dev());

    const auto&                            face = m.random_face();
    std::uniform_real_distribution<double> distribution(0.1, 0.9);

    const double b0 = distribution(generator);
    const double b1 = 0.95 * (1.0 - b0) * distribution(generator);
    const double b2 = 1.0 - b0 - b1;

    return PointInFaceDescriptor{
            face, {b0, b1, b2}
    };
}

namespace {
bool
operator==(
        const Point::PointInFaceDescriptor& p1, const Point::PointInFaceDescriptor& p2
) {
    return (p1.face() == p2.face())
           && mdv::condition::is_zero_norm(p1.coords() - p2.coords());
}
}  // namespace

bool
Point::is_undefined() const noexcept {
    return std::holds_alternative<PointInFaceDescriptor>(_pt)
           && (std::get<PointInFaceDescriptor>(_pt) == undefined_point);
}

bool
Point::operator==(const Point& other) const noexcept {
    return condition::is_zero_norm(this->position() - other.position());
}

bool
Point::operator!=(const Point& other) const noexcept {
    return !(*this == other);
}
