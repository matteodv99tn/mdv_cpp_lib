#ifndef MDV_MESH_index_elem_HPP
#define MDV_MESH_index_elem_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh_data.hpp"

namespace mdv::mesh::internal {

class MeshElement {
public:
    using MeshData  = ::mdv::mesh::internal::MeshData;
    using EigenData = ::mdv::mesh::internal::EigenData;
    using CgalImpl  = ::mdv::mesh::internal::CgalImpl;

    MeshElement() = default;

    MeshElement(const MeshData& data) noexcept : _mesh_data(&data) {}

    // clang-format off
    MDV_NODISCARD virtual bool     is_valid() const noexcept   { return (_mesh_data != nullptr); }

    MDV_NODISCARD const MeshData&  data() const noexcept       { assert(is_valid()); return *_mesh_data; }
    MDV_NODISCARD const EigenData& eigen_data() const noexcept { assert(is_valid()); return _mesh_data->eigen_data; }
    MDV_NODISCARD spdlog::logger&  logger() const noexcept     { assert(is_valid()); return *_mesh_data->logger.get(); };
    MDV_NODISCARD const CgalImpl&  cgal() const noexcept       { assert(is_valid()); return *_mesh_data->impl; }

    // clang-format on

    MDV_NODISCARD virtual std::string describe() const = 0;

    MDV_NODISCARD const MeshData*
    data_ptr() const {
        return _mesh_data;
    }

protected:
    const MeshData* _mesh_data = nullptr;
};

class IndexBasedMeshElement : public MeshElement {
public:
    using Index = ::mdv::mesh::Index;

    IndexBasedMeshElement() = default;

    IndexBasedMeshElement(const MeshData& data, const Index& id) noexcept :
            MeshElement(data), _id(id){};

    // clang-format off
    MDV_NODISCARD bool  is_valid() const noexcept override { return MeshElement::is_valid() && (_id != invalid_index);}
    MDV_NODISCARD Index id() const noexcept                { return _id; }

    // clang-format on

    bool
    operator==(const IndexBasedMeshElement& other) const noexcept {
        return (_mesh_data == other._mesh_data) && (_id == other._id);
    }

protected:
    Index _id = invalid_index;
};


}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_index_elem_HPP
