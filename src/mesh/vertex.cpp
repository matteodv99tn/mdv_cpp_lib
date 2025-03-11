#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>

#include "mdv/mesh/cgal_data.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Mesh;

Eigen::Vector3d
Mesh::Vertex::normal() const noexcept {
    using VertexDescriptor = Mesh::CgalData::VertexDescriptor;
    using Vec3             = Mesh::CgalData::Vec3;
#if MDV_CGAL_VERSION == 5
    const auto normals =
            _mesh->_data->_mesh.property_map<VertexDescriptor, Vec3>("v:normal").first;
#elif MDV_CGAL_VERSION == 6
    const auto normals =
            _mesh->_data->_mesh.property_map<VertexDescriptor, Vec3>("v:normal")
                    .value();
#endif
    return convert(normals[VertexDescriptor(_id)]);
}

std::string
Mesh::Vertex::describe() const {
    return fmt::format("ID #{}: {}", id(), eigen_to_str(position()));
}
