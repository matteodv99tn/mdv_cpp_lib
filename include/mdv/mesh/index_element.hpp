#ifndef MDV_MESH_index_elem_HPP
#define MDV_MESH_index_elem_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_data.hpp"

namespace mdv::mesh::internal {
class IndexBasedElement {
public:
    using Index     = ::mdv::mesh::Index;
    using MeshData  = ::mdv::mesh::internal::MeshData;
    using EigenData = ::mdv::mesh::internal::EigenData;

    IndexBasedElement() = default;

    IndexBasedElement(const MeshData& data, const Index& id) noexcept :
            _mesh_data(&data), _id(id) {};

    // clang-format off
    MDV_NODISCARD bool             is_valid() const noexcept   { return (_mesh_data != nullptr) && (_id != invalid_index);}
    MDV_NODISCARD Index            id() const noexcept         { return _id; }

    MDV_NODISCARD const MeshData&  data() const noexcept       { assert(is_valid()); return *_mesh_data; }
    MDV_NODISCARD const EigenData& eigen_data() const noexcept { assert(is_valid()); return _mesh_data->eigen_data; }
    MDV_NODISCARD spdlog::logger&  logger() const noexcept     { assert(is_valid()); return *_mesh_data->logger.get(); };

    // clang-format on

protected:
    const MeshData* _mesh_data = nullptr;
    Index           _id        = invalid_index;
};


}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_index_elem_HPP
