#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include <vector>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

class Vertex : internal::MeshElement {
public:
    using Vector        = std::vector<Vertex>;
    using Iterator      = Vector::iterator;
    using ConstIterator = Vector::const_iterator;

    Vertex(const Mesh& mesh, const CartesianPoint& position) :
            MeshElement(mesh), _pos(position) {}

    // clang-format off
    MDV_NODISCARD const CartesianPoint& position() const { return _pos; }

    MDV_NODISCARD HalfEdge& half_edge() const { assert(_he); return *_he; }


    MDV_NODISCARD const Eigen::Vector3d& normal() const noexcept { return _n; };

    // clang-format on

    MDV_NODISCARD std::size_t id() const;

    MDV_NODISCARD std::string describe() const override;

private:
    friend class Mesh;

    CartesianPoint  _pos = CartesianPoint::Zero();
    HalfEdge*       _he  = nullptr;
    Eigen::Vector3d _n   = Eigen::Vector3d::Zero();

    void bake_properties();
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
