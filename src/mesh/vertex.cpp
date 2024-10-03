#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"

using mdv::mesh::Mesh;

Eigen::Vector3d
Mesh::Vertex::position() const noexcept {
    const auto& cgal_id     = static_cast<Mesh::CgalMesh::Mesh::Vertex_index>(_id);
    const auto& cgal_vertex = _mesh->_cgal_data->_mesh.point(cgal_id);
    return {cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z()};
}
