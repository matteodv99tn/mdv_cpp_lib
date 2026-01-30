#ifndef MDV_MESH_ELEMENT_HPP
#define MDV_MESH_ELEMENT_HPP

#include <cassert>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh::internal {

/**
 * @brief Base class for view entities tied to a mesh instance.
 *
 * Mesh elements (faces, vertices, half-edges) share a reference to the owning
 * mesh and provide validity checks. This is part of the internal API used to
 * keep mesh relationships consistent.
 */
class MeshElement {
public:
    using CgalImpl = ::mdv::mesh::internal::CgalImpl;

    /**
     * @brief Default constructor, yields an invalid element.
     */
    MeshElement() = default;

    /**
     * @brief Constructs an element bound to a mesh.
     *
     * @param mesh Owning mesh.
     */
    MeshElement(const Mesh& mesh) noexcept : _mesh_ptr(const_cast<Mesh*>(&mesh)) {}

    MeshElement(const MeshElement& other)            = default;
    MeshElement(MeshElement&& other)                 = default;
    MeshElement& operator=(const MeshElement& other) = default;
    MeshElement& operator=(MeshElement&& other)      = default;

    /**
     * @brief Returns true if the element is bound to a mesh.
     *
     * @return True if the mesh pointer is valid.
     */
    MDV_NODISCARD virtual bool
    is_valid() const noexcept {
        return (_mesh_ptr != nullptr);
    }

    /**
     * @brief Returns true if the element has no associated mesh.
     *
     * @return True if the mesh pointer is null.
     */
    MDV_NODISCARD virtual bool
    undefined_mesh() const noexcept {
        return (_mesh_ptr == nullptr);
    }

    /**
     * @brief Returns the owning mesh.
     *
     * @return Mesh reference.
     */
    MDV_NODISCARD const Mesh&
    mesh() const noexcept {
        assert(is_valid());
        return *_mesh_ptr;
    }

    /**
     * @brief Human-readable description of the element.
     *
     * @return Description string.
     */
    MDV_NODISCARD virtual std::string describe() const = 0;

protected:
    Mesh* _mesh_ptr = nullptr;
};

/**
 * @brief Mesh element carrying a stable index into the mesh.
 */
class IndexedMeshElement : public MeshElement {
public:
    /**
     * @brief Default constructor, invalid index.
     */
    IndexedMeshElement() = default;

    /**
     * @brief Constructs an indexed element bound to a mesh.
     *
     * @param mesh Owning mesh.
     * @param id Element index.
     */
    IndexedMeshElement(const Mesh& mesh, const Index id) noexcept :
            MeshElement(mesh), _id(id) {}

    IndexedMeshElement(const IndexedMeshElement& other)            = default;
    IndexedMeshElement(IndexedMeshElement&& other)                 = default;
    IndexedMeshElement& operator=(const IndexedMeshElement& other) = default;
    IndexedMeshElement& operator=(IndexedMeshElement&& other)      = default;

    /**
     * @brief Returns the element index in the mesh.
     *
     * @return Element index.
     */
    MDV_NODISCARD Index
    id() const noexcept {
        return _id;
    }

    /**
     * @brief Returns true if mesh is defined and index is valid.
     *
     * @return True if mesh is valid and index is not invalid.
     */
    MDV_NODISCARD bool
    is_valid() const noexcept override {
        return MeshElement::is_valid() && _id != invalid_index;
    }

protected:
    /**
     * @brief Updates the element index.
     *
     * @param id New element index.
     */
    void
    set_id(Index id) noexcept {
        _id = id;
    }

private:
    Index _id = invalid_index;
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_ELEMENT_HPP
