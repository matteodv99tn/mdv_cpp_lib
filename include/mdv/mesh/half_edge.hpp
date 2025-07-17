#ifndef MDV_MESH_HALF_EDGE_HPP
#define MDV_MESH_HALF_EDGE_HPP

#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"
#include "mdv/mesh/vertex.hpp"

namespace mdv::mesh {

struct HalfEdge : internal::MeshElement {
    using Vector        = std::vector<HalfEdge>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    HalfEdge(Mesh& mesh) : MeshElement(mesh) {}

    MDV_NODISCARD Face&
    face() const {
        if (_face == nullptr)
            throw std::runtime_error("HalfEdge is not associated to any Face!");
        return *_face;
    }

    MDV_NODISCARD CartesianPoint
    origin_position() const {
        return origin().position();
    }

    MDV_NODISCARD Eigen::Vector3d
                  direction() const {
        return next()->origin_position() - origin_position();
    }

    MDV_NODISCARD Eigen::Vector3d
                  normalised_direction() const {
        return direction().normalized();
    }

    /**
     * Yields the normal vector to the halfedge which points "inward" the face
     */
    MDV_NODISCARD Eigen::Vector3d inbound_direction() const;

    /**
     * Computes the rotation which will make the inbound direction of "this" face
     * aligned with inbound rotation of the opposite face.
     */
    MDV_NODISCARD Eigen::Quaterniond aligning_rotation() const;

    // clang-format off
    MDV_NODISCARD HalfEdge* next() const { assert(_next); return _next; }
    MDV_NODISCARD HalfEdge* prev() const { assert(_prev); return _prev; }
    MDV_NODISCARD HalfEdge* twin() const { assert(_twin); return _twin; }

    MDV_NODISCARD Vertex& origin() const { assert(_origin); return *_origin; }

    MDV_NODISCARD Face& opposite_face() const { assert(_twin != nullptr); return _twin->face(); }
        
    MDV_NODISCARD bool operator==(const HalfEdge& other) const { return this == &other; }
    MDV_NODISCARD bool operator!=(const HalfEdge& other) const { return this != &other; }

    // clang-format on

    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;
    Vertex*   _origin = nullptr;
    HalfEdge* _twin   = nullptr;
    HalfEdge* _next   = nullptr;
    HalfEdge* _prev   = nullptr;
    Face*     _face   = nullptr;

    Vertex*
    tip_ptr() const {
        assert(_next != nullptr);
        return _next->_origin;
    }

    bool
    is_opposite_of(const HalfEdge& other) const {
        return (this->_origin == other.tip_ptr()) && (this->tip_ptr() == other._origin);
    }
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_HALF_EDGE_HPP
