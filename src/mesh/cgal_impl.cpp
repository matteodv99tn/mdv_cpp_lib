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
#include "mdv/utils/logging.hpp"

using std::filesystem::path;

namespace rs = ranges;
// \cond DOXYGEN_IGNORE
using ::mdv::mesh::internal::CgalImpl;

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
        const auto m    = get_mesh_impl(pt.vertex());
        const auto v_id = static_cast<CgalImpl::VertexDescriptor>(pt.vertex().id());
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

    CgalImpl::FaceLocation res = std::visit(
            overload{
                    std::move(vertex_descriptor),
                    std::move(face_descriptor),
                    [](const auto& pt) -> CgalImpl::FaceLocation {
                        throw std::runtime_error(
                                "Can't convert PointOnEdge to barycentric!"
                        );
                        return {
                                CGAL::SM_Face_index{0},
                                {0, 0, 0}
                        };
                    }
            },
            point.descriptor()
    );

#ifndef NDEBUG
    CgalImpl::ShortestPathTraits::Classify_barycentric_coordinate classifier;
    using CGAL::Surface_mesh_shortest_paths_3::BARYCENTRIC_COORDINATES_ON_BOUNDARY;
    using CGAL::Surface_mesh_shortest_paths_3::BARYCENTRIC_COORDINATES_ON_BOUNDED_SIDE;
    using CGAL::Surface_mesh_shortest_paths_3::BARYCENTRIC_COORDINATES_ON_VERTEX;

    auto get_point_class = overload{
            [](const Point::PointOnVertexDescriptor& pt) {
                return BARYCENTRIC_COORDINATES_ON_VERTEX;
            },
            [](const Point::PointOnEdgeDescriptor& pt) {
                return BARYCENTRIC_COORDINATES_ON_BOUNDARY;
            },
            [](const Point::PointInFaceDescriptor& pt) {
                return BARYCENTRIC_COORDINATES_ON_BOUNDED_SIDE;
            }

    };
    const auto internal_state        = std::visit(get_point_class, point.descriptor());
    const auto [classified_state, _] = classifier(res.second);
    assert(classified_state == internal_state);
#endif

    return res;
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
