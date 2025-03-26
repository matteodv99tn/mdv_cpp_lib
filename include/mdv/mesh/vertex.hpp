#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/index_element.hpp"
#include "mdv/mesh/mesh_iterator.hpp"

namespace mdv::mesh {

class Vertex : public internal::IndexBasedElement {
public:
    using Index     = ::mdv::mesh::Index;
    using MeshData  = ::mdv::mesh::internal::MeshData;
    using EigenData = ::mdv::mesh::internal::EigenData;
    using Iterator  = MeshIterator<Vertex, Index>;

    Vertex(const MeshData& data, const Index& id) noexcept :
            IndexBasedElement(data, id) {};

    MDV_NODISCARD Eigen::Vector3d normal() const;

    bool
    operator==(const Vertex& other) const noexcept {
        return (_mesh_data == other._mesh_data) && (_id == other._id);
    }

    MDV_NODISCARD const CartesianPoint&
    position() const noexcept {
        assert(is_valid());
        return eigen_data().vertices[id()];
    }

    MDV_NODISCARD
    std::string describe() const;

private:
    friend class MeshIterator<Vertex, Index>;
};
}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
