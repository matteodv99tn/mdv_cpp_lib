#include "mdv/mesh/vertex.hpp"

#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Vertex;

Eigen::Vector3d
Vertex::normal() const {
    using VertexDescriptor = internal::CgalImpl::VertexDescriptor;
    using Vec3             = internal::CgalImpl::Vec3;

#if MDV_CGAL_VERSION == 5
    const auto normals =
            data().impl->_mesh.property_map<VertexDescriptor, Vec3>("v:normal").first;
#elif MDV_CGAL_VERSION == 6
    const auto normals =
            data().impl->_mesh.property_map<VertexDescriptor, Vec3>("v:normal").value();
#endif
    return internal::convert(normals[VertexDescriptor(_id)]);
}

std::string
Vertex::describe() const {
    if (_mesh_data == nullptr) return "Vertex object of unspecified mesh";
    if (id() == invalid_index)
        return fmt::format("Invalid vertex on mesh '{}'", data().name);

    return fmt::format(
            "Vertex ID #{} (position: {}) on mesh '{}'",
            id(),
            eigen_to_str(position()),
            data().name
    );
}
