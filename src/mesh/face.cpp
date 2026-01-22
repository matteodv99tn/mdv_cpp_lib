#include "mdv/mesh/face.hpp"

#include <fmt/format.h>
#include <gsl/assert>

#include "CGAL/Polygon_mesh_processing/compute_normal.h"
#include "mdv/mesh/cgal_impl.hpp"
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
Face::half_edge() const noexcept {
    return {mesh(),
            CGAL::halfedge(
                    internal::to_face_impl(*this), internal::get_mesh_impl(*this)
            )};
}

Eigen::Vector3d
Face::normal() const noexcept {
    const auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(
            internal::to_face_impl(*this), mesh().cgal()._mesh
    );
    return {normal.x(), normal.y(), normal.z()};
}

std::optional<mdv::mesh::HalfEdge>
Face::adjacent_to(const Face& other) const {
    const auto  this_f = internal::to_face_impl(*this);
    const auto& m      = internal::get_mesh_impl(*this);

    for (auto h : CGAL::halfedges_around_face(halfedge(this_f, m), m)) {
        const auto h_opposite = CGAL::opposite(h, m);
        if (m.face(h_opposite) == internal::to_face_impl(other))
            return HalfEdge{mesh(), static_cast<Index>(h)};
    }
    return std::nullopt;
}

std::array<mdv::mesh::Index, 3>
Face::vertices_ids() const {
    const auto           f_id = internal::to_face_impl(*this);
    const auto&          m    = internal::get_mesh_impl(*this);
    std::array<Index, 3> res;
    std::size_t          j = 0;
    for (const auto v_id : CGAL::vertices_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = v_id;
        ++j;
    }
    assert(j == 3);
    return res;
}

std::array<mdv::mesh::Index, 3>
Face::neighbour_ids() const {
    const auto           f_id = internal::to_face_impl(*this);
    const auto&          m    = internal::get_mesh_impl(*this);
    std::array<Index, 3> res;
    std::size_t          j = 0;
    for (const auto v_id : CGAL::faces_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = v_id;
        ++j;
    }
    assert(j == 3);
    return res;
}
