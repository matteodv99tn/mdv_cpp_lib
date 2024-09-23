#ifndef MDV_MESH_ELEMENT_HPP
#define MDV_MESH_ELEMENT_HPP

#include <gsl/pointers>

#include <mdv/macros.hpp>
#include <mdv/mesh/fwd.hpp>

namespace mdv::mesh {

class MeshElement {
public:
    MeshElement(const Mesh* mesh) noexcept : _mesh(mesh) {};

    /**
     * @brief Const pointer to the mesh.
     *
     */
    MDV_NODISCARD const Mesh*
    mesh() const noexcept {
        return _mesh;
    };

    bool
    operator==(const MeshElement& other) const noexcept {
        return _mesh == other._mesh;
    }

    bool
    operator!=(const MeshElement& other) const noexcept {
        return _mesh != other._mesh;
    }


private:
    gsl::not_null<const Mesh*> _mesh;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_ELEMENT_HPP
