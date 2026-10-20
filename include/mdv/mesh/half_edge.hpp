#ifndef MDV_MESH_HALF_EDGE_HPP
#define MDV_MESH_HALF_EDGE_HPP

#include "mdv/eigen_defines.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

/** @brief Outgoing halfedge at v on f; throws if v is not incident to f. */
MDV_NODISCARD HalfEdgeId halfedge(const Mesh& mesh, VertexId v, FaceId f);

/** @brief Previous halfedge in the same face or boundary cycle. */
MDV_NODISCARD HalfEdgeId prev(const Mesh& mesh, HalfEdgeId h);

/** @brief Oppositely directed halfedge. */
MDV_NODISCARD HalfEdgeId twin(const Mesh& mesh, HalfEdgeId h);

/** @brief Incident face, or an invalid handle on the mesh boundary. */
MDV_NODISCARD FaceId face(const Mesh& mesh, HalfEdgeId h);

/** @brief Unnormalized directed edge vector. */
MDV_NODISCARD Vec3d direction(const Mesh& mesh, HalfEdgeId h);

/** @brief Unit direction perpendicular to h pointing into its incident face. */
MDV_NODISCARD Vec3d inbound_direction(const Mesh& mesh, HalfEdgeId h);

}  // namespace mdv::mesh

#endif  // MDV_MESH_HALF_EDGE_HPP
