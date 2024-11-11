#include <CGAL/Surface_mesh/Surface_mesh.h>

#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"

using ::mdv::mesh::Mesh;

Mesh::CgalMesh::FaceLocation
mdv::mesh::location_from_mesh_point(const ::mdv::mesh::Mesh::Point& pt) noexcept {
    const Eigen::Vector3d b = pt.barycentric();
    std::array<double, 3> bar_coords{b(0), b(1), b(2)};
    return {static_cast<CGAL::SM_Face_index>(pt.face_id()), bar_coords};
}

Mesh::Geodesic
mdv::mesh::construct_geodesic(
        Mesh::CgalMesh::ShortestPath& shpath, const Mesh::Point& from
) {
    const auto&                         from_loc = location_from_mesh_point(from);
    std::vector<Mesh::CgalMesh::Point3> cgal_geod;
    shpath.shortest_path_points_to_source_points(
            from_loc.first, from_loc.second, std::back_inserter(cgal_geod)
    );

    Mesh::Geodesic geod(cgal_geod.size());
    std::transform(
            cgal_geod.cbegin(),
            cgal_geod.cend(),
            geod.begin(),
            [](const auto& pt) -> Point3d { return {pt.x(), pt.y(), pt.z()}; }
    );
    return geod;
}
