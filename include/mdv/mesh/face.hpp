#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

/**
 * @brief Triangular face of a discrete surface mesh.
 *
 * Faces are the fundamental surface elements used to define local tangent
 * planes and geodesic propagation across the mesh.
 *
 * Faces are view-types on a Mesh object.
 */
class Face final : public internal::IndexedMeshElement {
public:
    /**
     * @brief Constructs a face from mesh and index.
     *
     * @param mesh Owning mesh.
     * @param id Face index.
     */
    Face(const Mesh& mesh, const Index id) noexcept : IndexedMeshElement(mesh, id) {}

    /**
     * @brief Sentinel invalid face.
     */
    static Face invalid_face;

    /**
     * @brief Face normal in 3D.
     *
     * @return Unit normal vector.
     */
    MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

    /**
     * @brief Equality check by element index.
     *
     * @param other Face to compare.
     * @return True if both handles identify the same index.
     */
    MDV_NODISCARD bool
    operator==(const Face& other) const noexcept {
        return id() == other.id();
    }

    /**
     * @brief Human-readable face description.
     *
     * @return Description string.
     */
    MDV_NODISCARD std::string describe() const override;

    /**
     * @brief Vertex indices of this triangle.
     *
     * @return Array of three vertex indices.
     */
    MDV_NODISCARD std::array<Index, 3> vertices_ids() const;

    /**
     * @brief Indices of neighboring faces across each edge.
     *
     * @return Array of three neighbor face indices.
     */
    MDV_NODISCARD std::array<Index, 3> neighbour_ids() const;

private:
    friend class Mesh;

    Face() = default;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
