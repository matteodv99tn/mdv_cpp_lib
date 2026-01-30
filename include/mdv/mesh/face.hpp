#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <optional>
#include <vector>

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
class Face : public internal::IndexedMeshElement {
public:
    using Vector        = std::vector<Face>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

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
     * @brief Returns one of the half-edges belonging to this face.
     *
     * @return A half-edge on this face.
     */
    MDV_NODISCARD HalfEdge half_edge() const noexcept;

    /**
     * @brief Face normal in 3D.
     *
     * @return Unit normal vector.
     */
    MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

    /**
     * @brief Equality check by identity.
     *
     * @param other Face to compare.
     * @return True if same object.
     */
    MDV_NODISCARD bool
    operator==(const Face& other) const {
        return this == &other;
    }

    /**
     * @brief Inequality check by identity.
     *
     * @param other Face to compare.
     * @return True if different object.
     */
    MDV_NODISCARD bool
    operator!=(const Face& other) const {
        return this != &other;
    }

    /**
     * @brief Human-readable face description.
     *
     * @return Description string.
     */
    MDV_NODISCARD std::string describe() const override;

    /**
     * @brief Returns the shared half-edge if this face is adjacent to another.
     *
     * @param other Face to test adjacency with.
     * @return Shared half-edge if adjacent, otherwise empty.
     */
    std::optional<HalfEdge> adjacent_to(const Face& other) const;

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
