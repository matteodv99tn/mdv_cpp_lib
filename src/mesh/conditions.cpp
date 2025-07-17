#include "mdv/mesh/conditions.hpp"

#include <exception>
#include <stdexcept>

#include "mdv/mesh/mesh.hpp"

bool
mdv::mesh::are_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
) noexcept {
    return e1.is_valid() && e2.is_valid() && (&e1.mesh() == &e2.mesh());
}

void
mdv::mesh::require_on_same_mesh(
        const internal::MeshElement& e1, const internal::MeshElement& e2
) {
    if (are_on_same_mesh(e1, e2)) return;

    if (!e1.is_valid() && !e2.is_valid()) {
        throw std::runtime_error(
                "Both provided mesh elements are not pointing to a valid mesh object"
        );
    }

    Logger& logger = *Mesh::default_logger.get();

    logger.error("Provided elements do not belong to the same mesh object");

    if (e1.is_valid()) logger.error("Element 1: {}", e1.describe());
    else logger.error("Element 1 is not on a valid mesh");
    if (e2.is_valid()) logger.error("Element 2: {}", e2.describe());
    else logger.error("Element 2 is not on a valid mesh");

    throw std::runtime_error("Two elements are not belonging to the same mesh");
}
