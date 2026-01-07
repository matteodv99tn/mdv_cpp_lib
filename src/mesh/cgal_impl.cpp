#include "mdv/mesh/cgal_impl.hpp"

#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path/barycentric.h>
#include <filesystem>
#include <stdexcept>

#include <range/v3/algorithm/transform.hpp>

#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging.hpp"

using std::filesystem::path;

namespace rs = ranges;
// \cond DOXYGEN_IGNORE
using ::mdv::mesh::internal::CgalImpl;
using Kernel = CgalImpl::Kernel;

// \endcond

namespace mdv::mesh::internal {

template <class... Ts>
struct overload : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
overload(Ts...) -> overload<Ts...>;

}  // namespace mdv::mesh::internal

CgalImpl::CgalImpl(const Mesh&& mesh, Logger::SharedPtr&& logger) :
        _mesh(mesh), _logger(std::move(logger)) {
    _shortest_path        = std::make_unique<ShortestPath>(_mesh);
    _geodesic_constructor = new CgalGeodesicConstructor(&_mesh);
    this->logger().trace("Initialised geodesic constructor");

    _shortest_path->build_aabb_tree(_aabb_tree);
    this->logger().trace("Built AABB tree of the mesh");

    build_vertex_normals_map();
}

CgalImpl::~CgalImpl() {
    delete _current_shortpath_source;
    delete _geodesic_constructor;
}

CgalImpl::CgalImpl(CgalImpl&& other) noexcept :
        _logger(other._logger),
        _mesh(std::move(other._mesh)),
        _geodesic_constructor(other._geodesic_constructor),
        _aabb_tree(std::move(other._aabb_tree)),
        _current_shortpath_source(other._current_shortpath_source) {
    other._current_shortpath_source = nullptr;
    logger().trace(
            "Moved CgalImpl from {} to {}",
            static_cast<void*>(&other),
            static_cast<void*>(this)
    );
}

gsl::owner<CgalImpl*>
CgalImpl::from_file(const path& file_path, Logger::SharedPtr&& logger) {
    assert(logger != nullptr);
    CgalImpl::Mesh mesh;
    logger->info("Loading mesh from file {}", file_path.string());
    // const bool loaded = CGAL::IO::read_polygon_mesh(file_path.string(), mesh);
    const bool loaded = CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(
            file_path.string(), mesh
    );
    if (!loaded) {
        logger->error("Unable to load mesh {}", file_path.string());
        throw std::runtime_error("Cannot load mesh");
    }

    return new CgalImpl(std::move(mesh), std::move(logger));
}

CgalImpl::FaceLocation
mdv::mesh::internal::location_from_mesh_point(
        const ::mdv::mesh::Point& point
) noexcept {
    auto vertex_descriptor =
            [](const Point::PointOnVertexDescriptor& pt) -> CgalImpl::FaceLocation {
        const auto& m    = get_mesh_impl(pt.vertex());
        const auto  v_id = static_cast<CgalImpl::VertexDescriptor>(pt.vertex().id());
        return CgalImpl::ShortestPath::face_location(v_id, m);
    };

    auto face_descriptor =
            [](const Point::PointInFaceDescriptor& pt) -> CgalImpl::FaceLocation {
        const auto f_id = to_face_impl(pt.face());
        const auto b    = pt.coords();

        return {
                f_id, {b(0), b(1), b(2)}
        };
    };

    auto edge_descriptor =
            [](const Point::PointOnEdgeDescriptor& pt) -> CgalImpl::FaceLocation {
        const auto& aabb_tree = pt.mesh().cgal()._aabb_tree;
        const auto& m_impl    = internal::get_mesh_impl(pt.mesh());
        return CgalImpl::ShortestPath::locate(
                internal::point3_from_point(pt),
                aabb_tree,
                m_impl,
                get(CGAL::vertex_point, m_impl)
        );
    };
    return std::visit(
            overload{
                    std::move(vertex_descriptor),
                    std::move(face_descriptor),
                    std::move(edge_descriptor)
            },
            point.descriptor()
    );
}

void
CgalImpl::build_vertex_normals_map() noexcept {
    _mesh.property_map<VertexDescriptor, Vec3>("v:normal");
    auto&& [normals, new_map] = _mesh.add_property_map<VertexDescriptor, Vec3>(
            "v:normal", CGAL::NULL_VECTOR
    );
    if (new_map) {
        _logger->trace("Computing vertex normals");
        CGAL::Polygon_mesh_processing::compute_vertex_normals(_mesh, normals);
    } else {
        _logger->trace("Vertex normals already computed");
    }
}

std::vector<Eigen::Vector3d>
CgalImpl::yield_vertices() const {
    using Vec3 = Eigen::Vector3d;
    std::vector<Vec3> res;
    res.resize(CGAL::num_vertices(_mesh));
    for (const CgalImpl::Mesh::Vertex_index& vertex : _mesh.vertices()) {
        const auto cgal_vertex = _mesh.point(vertex);
        res[vertex.idx()] = Vec3(cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z());
    }
    return res;
}

std::vector<mdv::mesh::IndexTriplet>
CgalImpl::yield_faces() const {
    std::vector<mdv::mesh::IndexTriplet> res;
    res.resize(CGAL::num_faces(_mesh));
    for (const CgalImpl::Mesh::Face_index& face : _mesh.faces()) {
        const auto vertex_iter =
                CGAL::vertices_around_face(_mesh.halfedge(face), _mesh);
        int i = 0;
        for (const auto& vi : vertex_iter) {
            Expects(i < 3);
            res[static_cast<long>(face)].at(i) = vi.idx();
            i++;
        }
    }
    return res;
}

//   ____                              _
//  / ___|___  _ ____   _____ _ __ ___(_) ___  _ __
// | |   / _ \| '_ \ \ / / _ \ '__/ __| |/ _ \| '_ \
// | |__| (_) | | | \ V /  __/ |  \__ \ | (_) | | | |
//  \____\___/|_| |_|\_/ \___|_|  |___/_|\___/|_| |_|
//
//  _          _
// | |__   ___| |_ __   ___ _ __ ___
// | '_ \ / _ \ | '_ \ / _ \ '__/ __|
// | | | |  __/ | |_) |  __/ |  \__ \
// |_| |_|\___|_| .__/ \___|_|  |___/
//              |_|
Eigen::Vector3d
mdv::mesh::internal::convert(const CgalImpl::Vec3& x) {
    return {x.x(), x.y(), x.z()};
}

Eigen::Vector3d
mdv::mesh::internal::convert(const CgalImpl::Point3& x) {
    return {x.x(), x.y(), x.z()};
}

Eigen::Quaterniond
mdv::mesh::internal::relative_face_rotation(
        const CgalImpl::CgalHalfEdgeIndex& he, const CgalImpl::Mesh& mesh
) {
    using namespace mdv::condition;
    using CGAL::Polygon_mesh_processing::compute_face_normal;

    const auto this_face     = face(he, mesh);
    const auto opposite_face = face(opposite(he, mesh), mesh);

    const Vec3d n1 = internal::convert(compute_face_normal(this_face, mesh));
    const Vec3d n2 = internal::convert(compute_face_normal(opposite_face, mesh));

    if (are_parallel(n1, n2)) return Eigen::Quaterniond::Identity();

    const auto& v0 = mesh.point(source(he, mesh));
    const auto& v1 = mesh.point(target(he, mesh));
    const Vec3d ax = internal::convert(v1 - v0).normalized();

    // Binormal axis computation
    const Vec3d b1 = -ax.cross(n1);
    const Vec3d b2 = ax.cross(n2);

    Eigen::Quaterniond res = Eigen::Quaterniond::FromTwoVectors(-b2, b1);
    assert(are_orthogonal(n1, b1));
    assert(are_orthogonal(n2, b2));
    assert(are_orthogonal(n1, res * b2));
    return res;
}

std::optional<Kernel::Point_3>
mdv::mesh::internal::edge_ray_intersection(
        const Kernel::Segment_3& edge, const Kernel::Ray_3& ray
) {
    using namespace mdv::condition;

    /* or(.) = origin of .
     * dir(.) = direction of .
     *
     * or(e) + t*dir(e) = or(ray) + s*dir(ray)   <-- solve for t, s
     * or(e) - or(ray) = -t*dir(e) + s*dir(ray)
     */
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;

    const Vec3 or_e  = internal::convert(edge.source());
    const Vec3 or_r  = internal::convert(ray.source());
    const Vec3 dir_e = internal::convert(edge.target() - edge.source());
    const Vec3 dir_r = internal::convert(ray.to_vector());

    if (are_parallel(dir_r, dir_e) && !are_equal(or_e, or_r)) return std::nullopt;
    if (are_parallel(dir_r, dir_e) && are_equal(or_e, or_r))
        throw std::runtime_error("Can't compute intersection when ray overlaps edge");

    Mat32 A;
    A.col(0)       = -dir_e;
    A.col(1)       = dir_r;
    const Vec3   b = or_e - or_r;
    const Vec2   x = A.colPivHouseholderQr().solve(b);
    const double t = x(0);
    const double s = x(1);

    assert(are_equal(or_e + t * dir_e, or_r + s * dir_r));

    if ((t >= 0.0) && (t <= 1.0 + 1e-9) && (s > 1e-9))
        return internal::point3_from_eigen(or_e + t * dir_e);

    return std::nullopt;
}

double
mdv::mesh::internal::total_curvature_deg(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
) {
    const auto v_pos = m.point(v);

    double res = 0.0;
    for (const auto he : CGAL::halfedges_around_target(v, m)) {
        const auto p1 = m.point(source(he, m));
        const auto p2 = m.point(target(next(he, m), m));

        auto e1 = p1 - v_pos;
        auto e2 = p2 - v_pos;
        res += CGAL::approximate_angle(e1, e2);
    }

    return res;
}

double
mdv::mesh::internal::total_curvature_rad(
        const CgalImpl::Mesh& m, const CgalImpl::VertexDescriptor& v
) {
    return total_curvature_deg(m, v) * M_PI / 180.0;
}
