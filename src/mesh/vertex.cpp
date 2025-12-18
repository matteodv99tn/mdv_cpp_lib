#include "mdv/mesh/vertex.hpp"

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <iterator>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging_extras.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Vertex;

// \endcond


std::string
Vertex::describe() const {
    if (!is_valid()) return "Invalid Vertex";

    return fmt::format(
            "Vertex ID #{} (position: {}) on mesh '{}'",
            id(),
            eigen_to_str(position()),
            mesh().name()
    );
}

mdv::mesh::CartesianPoint
Vertex::position() const noexcept {
    const auto cgal_pt = internal::to_vertex_impl(*this);
    return {cgal_pt.x(), cgal_pt.y(), cgal_pt.z()};
}

Eigen::Vector3d
Vertex::normal() const noexcept {
    using VertexDescriptor = internal::CgalImpl::VertexDescriptor;
    using Vec3             = internal::CgalImpl::Vec3;
    const auto& m          = internal::get_mesh_impl(*this);
#if MDV_CGAL_VERSION == 5
    const auto normals = m.property_map<VertexDescriptor, Vec3>("v:normal").first;
#elif MDV_CGAL_VERSION == 6
    const auto normals = m.property_map<VertexDescriptor, Vec3>("v:normal").value();
#endif
    return internal::convert(normals[VertexDescriptor(id())]);
}

double
Vertex::total_curvature() const {
    const auto&                                m     = internal::get_mesh_impl(mesh());
    const auto                                 v_pos = internal::to_vertex_impl(*this);
    const internal::CgalImpl::VertexDescriptor v_id{id()};

    double res = 0.0;
    for (const auto he : CGAL::halfedges_around_target(v_id, m)) {
        const auto p1 = m.point(source(he, m));
        const auto p2 = m.point(target(next(he, m), m));

        auto e1 = p1 - v_pos;
        auto e2 = p2 - v_pos;
        res += CGAL::approximate_angle(e1, e2);
    }

    return res * M_PI / 180.0;
}

double
Vertex::gauss_curvature() const {
    return 2 * M_PI - total_curvature();
}
