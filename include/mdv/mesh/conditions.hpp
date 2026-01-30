#ifndef MDV_MESH_CONDITIONS_HPP
#define MDV_MESH_CONDITIONS_HPP

#include "mdv/mesh/mesh_element.hpp"

namespace mdv::mesh {

/**
 * @brief Returns true if two mesh elements belong to the same mesh.
 *
 * @param e1 First mesh element.
 * @param e2 Second mesh element.
 * @return True if both are on the same mesh.
 */
bool are_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
) noexcept;

/**
 * @brief Throws if the two elements are not on the same mesh.
 *
 * @param e1 First mesh element.
 * @param e2 Second mesh element.
 */
void require_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_CONDITIONS_HPP
