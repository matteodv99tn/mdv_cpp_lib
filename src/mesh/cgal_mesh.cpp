#include <CGAL/Surface_mesh/Surface_mesh.h>

#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"

using ::mdv::mesh::Mesh;

Mesh::CgalMesh::FaceLocation
Mesh::CgalMesh::location_from_mesh_point(const ::mdv::mesh::Mesh::Point& pt) noexcept {
    const Eigen::Vector3d b = pt.barycentric();
    std::array<double, 3> bar{b(0), b(0), b(0)};
    return {static_cast<CGAL::SM_Face_index>(pt.face_id()), bar};
}
