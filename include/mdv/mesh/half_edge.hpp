#ifndef MDV_MESH_HALF_EDGE_HPP
#define MDV_MESH_HALF_EDGE_HPP

#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"
#include "mdv/mesh/vertex.hpp"

namespace mdv::mesh {

/**
 * @brief Represents a half-edge in a mesh.
 *
 * A half-edge is one of two directed edges that share the same edge but
 * point in opposite directions. It is used to represent the connectivity between
 * vertices and faces in a mesh.
 */
struct HalfEdge : internal::IndexedMeshElement {
    using Vector        = std::vector<HalfEdge>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    /**
     * @brief Constructs a half-edge associated with the given mesh.
     *
     * @param mesh The mesh to which this half-edge belongs.
     */
    HalfEdge(const Mesh& mesh, const Index id) : IndexedMeshElement(mesh, id) {}

    /**
     * @brief Retrieves the face that contains this half-edge.
     *
     * @return Reference to the face object.
     * @throw std::runtime_error If this half-edge is not associated with any face.
     */
    MDV_NODISCARD Face face() const noexcept;

    /**
     * @brief Retrieves the position of the origin vertex of this half-edge.
     *
     * @return CartesianPoint representing the position of the origin vertex.
     */
    MDV_NODISCARD CartesianPoint
    origin_position() const {
        return origin().position();
    }

    /**
     * @brief Retrieves the direction vector of this half-edge.
     *
     * @return Eigen::Vector3d representing the direction vector.
     */
    MDV_NODISCARD Eigen::Vector3d
                  direction() const {
        return next().origin_position() - origin_position();
    }

    /**
     * @brief Retrieves the normalized direction vector of this half-edge.
     *
     * @return Eigen::Vector3d representing the normalized direction vector.
     */
    MDV_NODISCARD Eigen::Vector3d
                  normalised_direction() const {
        return direction().normalized();
    }

    /**
     * @brief Computes the inbound direction vector for this half-edge.
     *
     * The inbound direction points "inward" the face containing this half-edge.
     *
     * @return Eigen::Vector3d representing the inbound direction vector.
     */
    MDV_NODISCARD Eigen::Vector3d inbound_direction() const;

    /**
     * @brief Computes the rotation that aligns the inbound direction of this
     *        half-edge with the inbound direction of its opposite face.
     *
     * @return Eigen::Quaterniond representing the alignment rotation.
     */
    MDV_NODISCARD Eigen::Quaterniond aligning_rotation() const;

    /**
     * @brief Retrieves the next half-edge in the sequence around this edge.
     *
     * @return Pointer to the next half-edge.
     */
    MDV_NODISCARD HalfEdge next() const noexcept;

    /**
     * @brief Retrieves the previous half-edge in the sequence around this edge.
     *
     * @return Pointer to the previous half-edge.
     */
    MDV_NODISCARD HalfEdge prev() const noexcept;

    /**
     * @brief Retrieves the twin half-edge of this half-edge.
     *
     * The twin half-edge is the one that shares the same edge but points in
     * the opposite direction.
     *
     * @return Pointer to the twin half-edge.
     */
    MDV_NODISCARD HalfEdge twin() const noexcept;

    /**
     * @brief Retrieves the origin vertex of this half-edge.
     *
     * @return Reference to the origin vertex object.
     */
    MDV_NODISCARD Vertex origin() const noexcept;

    /**
     * @brief Retrieves the opposite face of this half-edge.
     *
     * The opposite face is the one that shares the same edge but points in
     * the opposite direction.
     *
     * @return Reference to the opposite face object.
     */
    MDV_NODISCARD Face opposite_face() const noexcept;

    /**
     * @brief Checks if this half-edge is equal to another half-edge.
     *
     * Two half-edges are considered equal if they point to the same memory
     * location.
     *
     * @param other The half-edge to compare with.
     * @return true If both half-edges are the same.
     * @return false Otherwise.
     */
    MDV_NODISCARD bool
    operator==(const HalfEdge& other) const {
        return this == &other;
    }

    /**
     * @brief Checks if this half-edge is not equal to another half-edge.
     *
     * Two half-edges are considered not equal if they do not point to the same
     * memory location.
     *
     * @param other The half-edge to compare with.
     * @return true If both half-edges are different.
     * @return false Otherwise.
     */
    MDV_NODISCARD bool
    operator!=(const HalfEdge& other) const {
        return this != &other;
    }

    /**
     * @brief Provides a string representation of this half-edge.
     *
     * The description includes the face ID and the origin vertex ID.
     *
     * @return std::string representing the half-edge.
     */
    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;
    friend class Face;

    HalfEdge() = default;

    /**
     * @brief Checks if this half-edge is opposite to another half-edge.
     *
     * Two half-edges are considered opposite if they share the same edge but
     * point in opposite directions.
     *
     * @param other The half-edge to compare with.
     * @return true If both half-edges are opposite.
     * @return false Otherwise.
     */
    bool is_opposite_of(const HalfEdge& other) const noexcept;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_HALF_EDGE_HPP
