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
 * @brief Directed half-edge for mesh connectivity and local frame transport.
 *
 * A half-edge represents one orientation of an undirected edge. It is used to
 * traverse the mesh, compute local frames, and propagate tangent directions
 * across faces, which are central operations for geodesic-based learning on
 * surfaces.
 * 
 * Half-edges are view-types on a Mesh object.
 */
struct HalfEdge : internal::IndexedMeshElement {
    using Vector        = std::vector<HalfEdge>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    /**
     * @brief Constructs a half-edge associated with the given mesh.
     *
     * @param mesh Owning mesh.
     * @param id Half-edge index.
     */
    HalfEdge(const Mesh& mesh, const Index id) : IndexedMeshElement(mesh, id) {}

    /**
     * @brief Constructs the half-edge on a face with a given source vertex.
     *
     * Throws if the vertex is not part of the face.
     *
     * @param vertex Source vertex.
     * @param face Face containing the half-edge.
     */
    HalfEdge(const Vertex& vertex, const Face& face);

    /**
     * @brief Face incident to this half-edge.
     *
     * @return Face containing the half-edge.
     */
    MDV_NODISCARD Face face() const noexcept;

    /**
     * @brief Position of the origin vertex.
     *
     * @return Origin position.
     */
    MDV_NODISCARD CartesianPoint
    origin_position() const {
        return origin().position();
    }

    /**
     * @brief Direction vector from origin to next vertex.
     *
     * @return Direction vector.
     */
    MDV_NODISCARD Eigen::Vector3d
                  direction() const {
        return next().origin_position() - origin_position();
    }

    /**
     * @brief Normalized direction vector.
     *
     * @return Unit direction vector.
     */
    MDV_NODISCARD Eigen::Vector3d
                  normalised_direction() const {
        return direction().normalized();
    }

    /**
     * @brief Inbound direction within the face.
     *
     * The inbound direction is orthogonal to the edge and lies in the face.
     *
     * @return Inbound unit direction.
     */
    MDV_NODISCARD Eigen::Vector3d inbound_direction() const;

    /**
     * @brief Rotation aligning this half-edge frame to its twin face frame.
     *
     * @return Quaternion aligning local frames.
     */
    MDV_NODISCARD Eigen::Quaterniond aligning_rotation() const;

    /**
     * @brief Next half-edge in the face loop.
     *
     * @return Next half-edge.
     */
    MDV_NODISCARD HalfEdge next() const noexcept;

    /**
     * @brief Previous half-edge in the face loop.
     *
     * @return Previous half-edge.
     */
    MDV_NODISCARD HalfEdge prev() const noexcept;

    /**
     * @brief Twin half-edge (same edge, opposite direction).
     *
     * @return Twin half-edge.
     */
    MDV_NODISCARD HalfEdge twin() const noexcept;

    /**
     * @brief Origin vertex of the half-edge.
     *
     * @return Origin vertex.
     */
    MDV_NODISCARD Vertex origin() const noexcept;

    /**
     * @brief Face on the opposite side of the edge.
     *
     * @return Opposite face.
     */
    MDV_NODISCARD Face opposite_face() const noexcept;

    /**
     * @brief Equality check by identity.
     *
     * @param other Half-edge to compare.
     * @return True if same object.
     */
    MDV_NODISCARD bool
    operator==(const HalfEdge& other) const {
        return this == &other;
    }

    /**
     * @brief Inequality check by identity.
     *
     * @param other Half-edge to compare.
     * @return True if different object.
     */
    MDV_NODISCARD bool
    operator!=(const HalfEdge& other) const {
        return this != &other;
    }

    /**
     * @brief Human-readable half-edge description.
     *
     * @return Description string.
     */
    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;
    friend class Face;

    HalfEdge() = default;

    /**
     * @brief Returns true if this half-edge is opposite to another.
     *
     * @param other Half-edge to compare.
     * @return True if opposite.
     */
    bool is_opposite_of(const HalfEdge& other) const noexcept;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_HALF_EDGE_HPP
