#include "mdv/mesh/vertex.hpp"

#include <CGAL/boost/graph/iterator.h>
#include <CGAL/boost/graph/properties.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <iterator>

#include <range/v3/all.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging_extras.hpp"

namespace mdv::mesh {
namespace rs = ::ranges;

std::string
describe(const Mesh& mesh, VertexId v) {
    if (!v.is_valid()) return "Invalid Vertex";

    return fmt::format(
            "Vertex ID #{} (position: {}) on mesh '{}'",
            v.value,
            eigen_to_str(position(mesh, v)),
            mesh.name()
    );
}

CartesianPoint
position(const Mesh& mesh, VertexId v) {
    const auto cgal_pt = internal::cgal_mesh(mesh).point(internal::to_cgal(v));
    return {cgal_pt.x(), cgal_pt.y(), cgal_pt.z()};
}

Eigen::Vector3d
normal(const Mesh& mesh, VertexId v) {
    using VertexDescriptor = internal::CgalImpl::VertexDescriptor;
    using Vec3             = internal::CgalImpl::Vec3;
    const auto& m          = internal::cgal_mesh(mesh);
#if MDV_CGAL_VERSION == 5
    const auto normals = m.property_map<VertexDescriptor, Vec3>("v:normal").first;
#elif MDV_CGAL_VERSION == 6
    const auto normals = m.property_map<VertexDescriptor, Vec3>("v:normal").value();
#endif
    return internal::convert(normals[internal::to_cgal(v)]);
}

double
total_angle(const Mesh& mesh, VertexId v) {
    const auto& m    = internal::cgal_mesh(mesh);
    const auto  v_id = internal::to_cgal(v);
    return internal::total_curvature_rad(m, v_id);
}

double
gauss_curvature(const Mesh& mesh, VertexId v) {
    return 2 * M_PI - total_angle(mesh, v);
}

VertexId
closest_vertex(const Mesh& mesh, const CartesianPoint& pt) {
    const auto [_, f_id] = mesh.cgal()._aabb_tree.closest_point_and_primitive(
            internal::point3_from_eigen(pt)
    );
    const auto& m                = internal::cgal_mesh(mesh);
    auto        squared_distance = [&pt, &m](const auto id) -> double {
        return (internal::convert(m.point(id)) - pt).norm();
    };
    const auto       h = halfedge(f_id, m);
    const std::array vs{target(h, m), target(next(h, m), m), source(h, m)};
    const auto*      min_id = rs::min_element(vs, std::less{}, squared_distance);
    return internal::from_cgal(*min_id);
}

}  // namespace mdv::mesh
