#ifndef MDV_MESH_CONDITIONS_HPP
#define MDV_MESH_CONDITIONS_HPP

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/index_element.hpp"
#include "mdv/mesh/mesh_data.hpp"

namespace mdv::mesh {

bool are_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
) noexcept;

/**
 * @brief Uses the are_on_same_mesh function to throw an execption if the two elements
 * are not belonging to the same mesh.
 */
void require_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_CONDITIONS_HPP
