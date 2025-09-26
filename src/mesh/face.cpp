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

std::size_t
Face::id() const {
    assert(is_valid());
    auto it = Face::ConstIterator(this);
    return std::distance(mesh().faces_begin(), it);
}

std::string
Face::describe() const {
    if (undefined_mesh()) return "Face object of unspecified mesh";
    if (_he == nullptr) return fmt::format("Invalid face on mesh '{}'", mesh().name());

    const auto& he = half_edge();
    const auto  v1 = he->origin().id();
    const auto  v2 = he->next()->origin().id();
    const auto  v3 = he->next()->next()->origin().id();
    return fmt::format(
            "Face ID #{} (vertices {}, {}, {}) of mesh '{}'",
            id(),
            v1,
            v2,
            v3,
            mesh().name()
    );
}

mdv::mesh::HalfEdge*
Face::adjacent_to(const Face& other) const {
    HalfEdge* he = this->half_edge();

    bool first_iter = true;
    while (first_iter || he != this->half_edge()) {
        first_iter = false;
        Face* opposite_face = he->_twin->_face;
        if (opposite_face != nullptr && opposite_face == &other) return he;
        he = he->next();
    }

    return nullptr;
}

void
Face::bake_properties() {
    assert(&_he->face() == &_he->prev()->face());

    const auto u = _he->direction();
    const auto v = _he->prev()->twin()->direction();

    _uv_map.set_origin(_he->origin_position());
    _uv_map.set_u_vector(u);
    _uv_map.set_v_vector(v);

    _n = (u.cross(v)).normalized();

    assert(mdv::condition::is_unit_norm(_n));
    assert(!mdv::condition::is_zero_norm(_uv_map.u_dir()));
    assert(!mdv::condition::is_zero_norm(_uv_map.v_dir()));
    assert(mdv::condition::are_orthogonal(_uv_map.u_dir(), normal()));
    assert(mdv::condition::are_orthogonal(_uv_map.v_dir(), normal()));
}
