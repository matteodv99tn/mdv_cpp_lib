#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <spdlog/spdlog.h>

#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"
#include "mdv/utils/logging_extras.hpp"

using ::mdv::mesh::Mesh;

Mesh::CgalMesh::FaceLocation
mdv::mesh::location_from_mesh_point(const ::mdv::mesh::Mesh::Point& pt) noexcept {
    // Note: barycentric coordinates are exported in the "wrong" order, since this
    // appears coorect w.r.t. to Cgal internal data alignment
    const Eigen::Vector3d b = pt.barycentric();
    std::array<double, 3> bar_coords{b(2), b(0), b(1)};
    return {static_cast<CGAL::SM_Face_index>(pt.face_id()), bar_coords};
}

Mesh::Geodesic
mdv::mesh::construct_geodesic(
        Mesh::CgalMesh::ShortestPath& shpath, const Mesh::Point& from
) {
    const auto [face_id, barycentric_coords] = location_from_mesh_point(from);
    std::vector<Mesh::CgalMesh::Point3> cgal_geod;
    shpath.shortest_path_points_to_source_points(
            face_id, barycentric_coords, std::back_inserter(cgal_geod)
    );

    Mesh::Geodesic geod(cgal_geod.size());
    std::transform(
            cgal_geod.cbegin(),
            cgal_geod.cend(),
            geod.begin(),
            [](const auto& pt) -> Point3d { return convert(pt); }
    );
    return geod;
}

void
Mesh::CgalMesh::build_vertex_normals_map() noexcept {
    _mesh.property_map<VertexDescriptor, Vec3>("v:normal");
    auto&& [normals, new_map] = _mesh.add_property_map<VertexDescriptor, Vec3>(
            "v:normal", CGAL::NULL_VECTOR
    );
    if (new_map) {
        _logger->debug("Computing vertex normals");
        CGAL::Polygon_mesh_processing::compute_vertex_normals(_mesh, normals);
    } else {
        _logger->debug("Vertex normals already computed");
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
mdv::mesh::convert(const Mesh::CgalMesh::Vec3& x) {
    return {x.x(), x.y(), x.z()};
}

Eigen::Vector3d
mdv::mesh::convert(const Mesh::CgalMesh::Point3& x) {
    return {x.x(), x.y(), x.z()};
}
