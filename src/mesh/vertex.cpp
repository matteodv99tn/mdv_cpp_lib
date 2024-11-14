#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>

#include "cgal_data.hpp"

using mdv::mesh::Mesh;

Eigen::Vector3d
Mesh::Vertex::normal() const noexcept {
    using VertexDescriptor = Mesh::CgalMesh::VertexDescriptor;
    using Vec3             = Mesh::CgalMesh::Vec3;
    // _mesh->_cgal_data->build_vertex_normals_map();
    const auto normals =
            _mesh->_cgal_data->_mesh.property_map<VertexDescriptor, Vec3>("v:normal")
                    .value();
    return convert(normals[VertexDescriptor(_id)]);
}
