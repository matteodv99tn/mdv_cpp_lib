#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <optional>
#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"
#include "mdv/mesh/uv_map.hpp"

namespace mdv::mesh {

class Face : public internal::MeshElement {
public:
    using Vector        = std::vector<Face>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    Face(const Mesh& mesh) : MeshElement(mesh) {}

    static Face invalid_face;

    MDV_NODISCARD std::size_t id() const;

    // clang-format off
    MDV_NODISCARD HalfEdge*       half_edge() const noexcept { assert(_he); return _he; }
    MDV_NODISCARD Eigen::Vector3d normal() const noexcept    { assert(_n.norm() > 0.001); return _n; }
    MDV_NODISCARD const UvMap&    uv_map() const noexcept    { return _uv_map; };

    MDV_NODISCARD bool operator==(const Face& other) const { return this == &other; }
    MDV_NODISCARD bool operator!=(const Face& other) const { return this != &other; }

    // clang-format on

    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;

    Face() = default;

    HalfEdge*       _he = nullptr;
    UvMap           _uv_map;
    Eigen::Vector3d _n = Eigen::Vector3d::Zero();

    void bake_properties();
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
