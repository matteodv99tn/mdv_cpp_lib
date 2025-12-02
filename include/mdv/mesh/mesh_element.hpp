#ifndef MDV_MESH_ELEMENT_HPP
#define MDV_MESH_ELEMENT_HPP

#include <cassert>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh::internal {

class MeshElement {
public:
    using CgalImpl = ::mdv::mesh::internal::CgalImpl;

    MeshElement() = default;

    MeshElement(const Mesh& mesh) noexcept : _mesh_ptr(const_cast<Mesh*>(&mesh)) {}

    MeshElement(const MeshElement& other)            = default;
    MeshElement(MeshElement&& other)                 = default;
    MeshElement& operator=(const MeshElement& other) = default;
    MeshElement& operator=(MeshElement&& other)      = default;

    MDV_NODISCARD virtual bool
    is_valid() const noexcept {
        return (_mesh_ptr != nullptr);
    }

    MDV_NODISCARD virtual bool
    undefined_mesh() const noexcept {
        return (_mesh_ptr == nullptr);
    }

    MDV_NODISCARD const Mesh&
    mesh() const noexcept {
        assert(is_valid());
        return *_mesh_ptr;
    }

    MDV_NODISCARD virtual std::string describe() const = 0;

protected:
    Mesh* _mesh_ptr = nullptr;
};

class IndexedMeshElement : public MeshElement {
public:
    IndexedMeshElement() = default;

    IndexedMeshElement(const Mesh& mesh, const Index id) noexcept :
            MeshElement(mesh), _id(id) {}

    IndexedMeshElement(const IndexedMeshElement& other)            = default;
    IndexedMeshElement(IndexedMeshElement&& other)                 = default;
    IndexedMeshElement& operator=(const IndexedMeshElement& other) = default;
    IndexedMeshElement& operator=(IndexedMeshElement&& other)      = default;

    MDV_NODISCARD Index
    id() const noexcept {
        return _id;
    }

    MDV_NODISCARD bool
    is_valid() const noexcept override {
        return MeshElement::is_valid() && _id != invalid_index;
    }

private:
    Index _id = invalid_index;
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_ELEMENT_HPP
