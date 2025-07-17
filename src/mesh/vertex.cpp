#include "mdv/mesh/vertex.hpp"

#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <iterator>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging_extras.hpp"

using mdv::mesh::Vertex;

std::size_t
Vertex::id() const {
    assert(is_valid());
    auto it = Vertex::ConstIterator(this);
    return std::distance(mesh().vertices_begin(), it);
}

std::string
Vertex::describe() const {
    if (undefined_mesh()) return "Vertex object of unspecified mesh";

    return fmt::format(
            "Vertex ID #{} (position: {}) on mesh '{}'",
            id(),
            eigen_to_str(position()),
            mesh().name()
    );
}

void
Vertex::bake_properties() {
    using VertexDescriptor = internal::CgalImpl::VertexDescriptor;
    using Vec3             = internal::CgalImpl::Vec3;

#if MDV_CGAL_VERSION == 5
    const auto normals =
            mesh().cgal() _mesh.property_map<VertexDescriptor, Vec3>("v:normal").first;
#elif MDV_CGAL_VERSION == 6
    const auto normals = mesh().cgal()
                                 ._mesh.property_map<VertexDescriptor, Vec3>("v:normal")
                                 .value();
#endif
    _n = internal::convert(normals[VertexDescriptor(id())]);
}
