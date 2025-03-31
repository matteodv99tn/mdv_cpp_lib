#include "mdv/mesh/cgal_impl.hpp"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <filesystem>
#include <spdlog/spdlog.h>

#include <range/v3/algorithm/transform.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/conditions.hpp"
#include "mdv/mesh/point.hpp"

using std::filesystem::path;

namespace rs = ranges;
using ::mdv::mesh::internal::CgalImpl;

CgalImpl::CgalImpl(const Mesh&& mesh, SpdLoggerPtr&& logger) :
        _mesh(mesh), _logger(std::move(logger)) {
    _shortest_path = std::make_unique<ShortestPath>(_mesh);
    this->logger().trace("Initialised shortest path object");

    _shortest_path->build_aabb_tree(_aabb_tree);
    this->logger().trace("Built AABB tree of the mesh");

    build_vertex_normals_map();
}

CgalImpl::~CgalImpl() {
    delete _current_shortpath_source;
}

gsl::owner<CgalImpl*>
CgalImpl::from_file(const path& file_path, SpdLoggerPtr&& logger) {
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
mdv::mesh::internal::location_from_mesh_point(const ::mdv::mesh::Point& pt) noexcept {
    // Note: barycentric coordinates are exported in the "wrong" order, since this
    // appears coorect w.r.t. to Cgal internal data alignment
    const Eigen::Vector3d b = pt.barycentric();
    std::array<double, 3> bar_coords{b(2), b(0), b(1)};
    return {static_cast<CGAL::SM_Face_index>(pt.face().id()), bar_coords};
}

::mdv::mesh::Geodesic
mdv::mesh::internal::construct_geodesic(
        CgalImpl::ShortestPath& shpath, const ::mdv::mesh::Point& from
) {
    const auto [face_id, barycentric_coords] = location_from_mesh_point(from);
    std::vector<CgalImpl::Point3> cgal_geod;
    shpath.shortest_path_points_to_source_points(
            face_id, barycentric_coords, std::back_inserter(cgal_geod)
    );

    Geodesic geod;
    geod.reserve(cgal_geod.size());
    rs::transform(
            cgal_geod,
            std::back_inserter(geod),
            [](const auto& pt) -> Eigen::Vector3d { return convert(pt); }
    );
    return geod;
}

::mdv::mesh::Geodesic
mdv::mesh::internal::construct_geodesic(
        const CgalImpl&           cgal_data,
        const ::mdv::mesh::Point& from,
        const ::mdv::mesh::Point& to
) {
    require_on_same_mesh(from.face(), to.face());

    auto& curr_target = cgal_data._current_shortpath_source;

    if (curr_target == nullptr || distance(*curr_target, to) > 1e-3) {
        cgal_data.logger().trace("Updating shortest path source point");
        if (curr_target != nullptr)
            cgal_data._shortest_path->remove_all_source_points();
        cgal_data._shortest_path->add_source_point(internal::location_from_mesh_point(to
        ));
        cgal_data._current_shortpath_source = new Point(to);
    }

    return construct_geodesic(*cgal_data._shortest_path, from);
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
