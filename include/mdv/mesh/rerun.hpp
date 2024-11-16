#ifndef MDV_MESH_RERUN_INTEGRATION_HPP
#define MDV_MESH_RERUN_INTEGRATION_HPP

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <rerun/archetypes/arrows3d.hpp>
#include <rerun/archetypes/mesh3d.hpp>
#include <rerun/archetypes/points3d.hpp>
#include <rerun/components/line_strip3d.hpp>

namespace mdv::mesh::rerun_convert {

::rerun::archetypes::Mesh3D mesh(const Mesh& mesh);

::rerun::components::LineStrip3D geodesic(const Geodesic& geodesic);

::rerun::archetypes::Arrows3D vertex_normals(const Mesh& mesh);

::rerun::archetypes::Points3D point(const Mesh::Point& pt);
::rerun::archetypes::Points3D points(const std::vector<Mesh::Point>& pt);

}  // namespace mdv::mesh::rerun_convert


#endif  // MDV_MESH_RERUN_INTEGRATION_HPP
