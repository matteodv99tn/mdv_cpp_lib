#include "mdv/mesh/face.hpp"

#include <fmt/format.h>
#include <gsl/assert>

#include "CGAL/Polygon_mesh_processing/compute_normal.h"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

Face Face::invalid_face = Face();

std::string
Face::describe() const {
    if (!is_valid()) return "Invalid face object";

    const auto& m  = internal::get_mesh_impl(*this);
    const auto  he = CGAL::halfedge(internal::to_face_impl(*this), m);
    const Index v1 = CGAL::source(he, m);
    const Index v2 = CGAL::source(CGAL::next(he, m), m);
    const Index v3 = CGAL::source(CGAL::next(CGAL::next(he, m), m), m);
    return fmt::format(
            "Face ID #{} (vertices {}, {}, {}) of mesh '{}'",
            id(),
            v1,
            v2,
            v3,
            mesh().name()
    );
}

Eigen::Vector3d
Face::normal() const noexcept {
    const auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(
            internal::to_face_impl(*this), mesh().cgal()._mesh
    );
    return {normal.x(), normal.y(), normal.z()};
}

std::array<Index, 3>
Face::vertices_ids() const {
    const auto           f_id = internal::to_face_impl(*this);
    const auto&          m    = internal::get_mesh_impl(*this);
    std::array<Index, 3> res{};
    std::size_t          j = 0;
    for (const auto v_id : CGAL::vertices_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = v_id;
        ++j;
    }
    assert(j == 3);
    return res;
}

std::array<Index, 3>
Face::neighbour_ids() const {
    const auto           f_id = internal::to_face_impl(*this);
    const auto&          m    = internal::get_mesh_impl(*this);
    std::array<Index, 3> res{};
    std::size_t          j = 0;
    for (const auto v_id : CGAL::faces_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = v_id;
        ++j;
    }
    assert(j == 3);
    return res;
}

}  // namespace mdv::mesh
