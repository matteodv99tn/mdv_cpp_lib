#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <optional>
#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

class Face : public internal::IndexedMeshElement {
public:
    using Vector        = std::vector<Face>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    Face(const Mesh& mesh, const Index id) noexcept : IndexedMeshElement(mesh, id) {}

    static Face invalid_face;

    MDV_NODISCARD HalfEdge half_edge() const noexcept;

    MDV_NODISCARD Eigen::Vector3d normal() const noexcept;

    MDV_NODISCARD bool
    operator==(const Face& other) const {
        return this == &other;
    }

    MDV_NODISCARD bool
    operator!=(const Face& other) const {
        return this != &other;
    }

    MDV_NODISCARD std::string describe() const override;

    std::optional<HalfEdge> adjacent_to(const Face& other) const;

    MDV_NODISCARD std::array<Index, 3> vertices_ids() const;

    /*
     * ids of neighbouring faces.
     */
    MDV_NODISCARD std::array<Index, 3> neighbour_ids() const;

private:
    friend class Mesh;

    Face() = default;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
