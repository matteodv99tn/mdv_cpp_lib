#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include <string>

#include "mdv/eigen_defines.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

/** @brief Legacy mesh-bound holder, retained until Point migrates to handles. */
class Vertex final : public internal::IndexedMeshElement {
public:
    Vertex(const Mesh& mesh, Index id) : IndexedMeshElement(mesh, id) {}

    MDV_NODISCARD std::string describe() const override;
};

/** @brief Vertex position. The handle must belong to mesh. */
MDV_NODISCARD CartesianPoint position(const Mesh& mesh, VertexId v);

/** @brief Precomputed vertex normal. The handle must belong to mesh. */
MDV_NODISCARD Vec3d normal(const Mesh& mesh, VertexId v);

/** @brief Sum of incident angles, in radians. */
MDV_NODISCARD double total_angle(const Mesh& mesh, VertexId v);

/** @brief Gaussian curvature (2*pi minus the total angle), in radians. */
MDV_NODISCARD double gauss_curvature(const Mesh& mesh, VertexId v);

MDV_NODISCARD std::string describe(const Mesh& mesh, VertexId v);

/** @brief Closest vertex of the AABB-projected face (preserves legacy selection). */
MDV_NODISCARD VertexId closest_vertex(const Mesh& mesh, const CartesianPoint& pt);

}  // namespace mdv::mesh

#endif  // MDV_MESH_VERTEX_HPP
