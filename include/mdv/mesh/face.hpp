#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <optional>
#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"
#include "mdv/mesh/uv_map.hpp"

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

    MDV_NODISCARD const UvMap&
    uv_map() const noexcept {
        return _uv_map;
    };

    MDV_NODISCARD bool
    operator==(const Face& other) const {
        return this == &other;
    }

    MDV_NODISCARD bool
    operator!=(const Face& other) const {
        return this != &other;
    }

    MDV_NODISCARD std::string describe() const override;

    HalfEdge adjacent_to(const Face& other) const;

private:
    friend class Mesh;

    Face() = default;
    UvMap _uv_map;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
