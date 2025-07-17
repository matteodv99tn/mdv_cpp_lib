#ifndef MDV_MESH_ELEMENT_HPP
#define MDV_MESH_ELEMENT_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh::internal {

class MeshElement {
public:
    using CgalImpl = ::mdv::mesh::internal::CgalImpl;

    MeshElement() = default;

    MeshElement(const Mesh& mesh) noexcept : _mesh_ptr(&mesh) {}

    // Mesh elements (faces, vertices, half-edges) cannot be copied!
    MeshElement(const MeshElement& other)            = delete;
    MeshElement(MeshElement&& other)                 = default;
    MeshElement& operator=(const MeshElement& other) = delete;
    MeshElement& operator=(MeshElement&& other)      = delete;

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
    const Mesh* _mesh_ptr = nullptr;
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_ELEMENT_HPP
