#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/index_element.hpp"
#include "mdv/mesh/mesh_iterator.hpp"

namespace mdv::mesh {

class Vertex : public internal::IndexBasedMeshElement {
public:
    using Index     = ::mdv::mesh::Index;
    using MeshData  = ::mdv::mesh::internal::MeshData;
    using EigenData = ::mdv::mesh::internal::EigenData;
    using Iterator  = MeshIterator<Vertex, Index>;

    Vertex(const MeshData& data, const Index& id) noexcept :
            IndexBasedMeshElement(data, id){};

    MDV_NODISCARD Eigen::Vector3d normal() const;

    MDV_NODISCARD const CartesianPoint&
    position() const noexcept {
        assert(is_valid());
        return eigen_data().vertices[id()];
    }

    MDV_NODISCARD
    std::string describe() const override;

private:
    friend class MeshIterator<Vertex, Index>;
};
}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
