#ifndef MDV_MESH_RERUN_INTEGRATION_HPP
#define MDV_MESH_RERUN_INTEGRATION_HPP

#include <mdv/mesh/fwd.hpp>
#include <rerun/archetypes/mesh3d.hpp>

namespace mdv::mesh::rerun {

::rerun::archetypes::Mesh3D from_mesh(const Mesh& mesh);


}  // namespace mdv::mesh::rerun


#endif  // MDV_MESH_RERUN_INTEGRATION_HPP
