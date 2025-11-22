#include "mdv/mesh/face.hpp"

#include <fmt/format.h>
#include <gsl/assert>

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Face;
// \endcond

Face Face::invalid_face = Face();

std::string
Face::describe() const {
    if (!is_valid()) return "Invalid face object";

    const auto& he = half_edge();
    const auto  v1 = he.origin().id();
    const auto  v2 = he.next().origin().id();
    const auto  v3 = he.next().next().origin().id();
    return fmt::format(
            "Face ID #{} (vertices {}, {}, {}) of mesh '{}'",
            id(),
            v1,
            v2,
            v3,
            mesh().name()
    );
}

mdv::mesh::HalfEdge
Face::adjacent_to(const Face& other) const {
    HalfEdge he = this->half_edge();

    bool first_iter = true;
    while (first_iter || he != this->half_edge()) {
        first_iter          = false;
        Face opposite_face = he.twin().face();
        if (opposite_face != Face::invalid_face && opposite_face == other) return he;
        he = he.next();
    }

    return {};
}
