#ifndef MDV_MESH_ELEMENT_HPP
#define MDV_MESH_ELEMENT_HPP

#include <gsl/pointers>

#include <mdv/macros.hpp>
#include <mdv/mesh/fwd.hpp>

namespace mdv::mesh {

class MeshElement {
public:
    MeshElement(Mesh* mesh) noexcept;
    MeshElement(const Mesh* mesh) noexcept;

    /**
     * @brief Provides a modifiable pointer to the mesh.
     * This method also guards against calling it on a const object, so use it with
     * caution.
     *
     * If you want just to access the mesh, use cmesh() instead.
     *
     */
    MDV_NODISCARD Mesh* mesh();

    /**
     * @brief Const pointer to the mesh.
     *
     */
    MDV_NODISCARD const Mesh* cmesh() const noexcept;

    bool operator==(const MeshElement&) const noexcept;
    bool operator!=(const MeshElement&) const noexcept;


private:
    gsl::not_null<Mesh*> _mesh;
    bool                 _is_modifiable_mesh{false};
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_ELEMENT_HPP
