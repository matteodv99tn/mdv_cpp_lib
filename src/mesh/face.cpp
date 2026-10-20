#include "mdv/mesh/face.hpp"

#include <fmt/format.h>
#include <gsl/assert>
#include <random>

#include "CGAL/Polygon_mesh_processing/compute_normal.h"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

std::string
describe(const Mesh& mesh, FaceId f) {
    if (!f.is_valid()) return "Invalid face object";

    const auto& m  = internal::cgal_mesh(mesh);
    const auto  he = CGAL::halfedge(internal::to_cgal(f), m);
    const Index v1 = CGAL::source(he, m);
    const Index v2 = CGAL::source(CGAL::next(he, m), m);
    const Index v3 = CGAL::source(CGAL::next(CGAL::next(he, m), m), m);
    return fmt::format(
            "Face ID #{} (vertices {}, {}, {}) of mesh '{}'",
            f.value,
            v1,
            v2,
            v3,
            mesh.name()
    );
}

Eigen::Vector3d
normal(const Mesh& mesh, FaceId f) {
    const auto normal = CGAL::Polygon_mesh_processing::compute_face_normal(
            internal::to_cgal(f), internal::cgal_mesh(mesh)
    );
    return {normal.x(), normal.y(), normal.z()};
}

std::array<VertexId, 3>
vertices(const Mesh& mesh, FaceId f) {
    const auto              f_id = internal::to_cgal(f);
    const auto&             m    = internal::cgal_mesh(mesh);
    std::array<VertexId, 3> res{};
    std::size_t             j = 0;
    for (const auto v_id : CGAL::vertices_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = internal::from_cgal(v_id);
        ++j;
    }
    assert(j == 3);
    return res;
}

std::array<FaceId, 3>
neighbours(const Mesh& mesh, FaceId f) {
    const auto            f_id = internal::to_cgal(f);
    const auto&           m    = internal::cgal_mesh(mesh);
    std::array<FaceId, 3> res{};
    std::size_t           j = 0;
    for (const auto v_id : CGAL::faces_around_face(CGAL::halfedge(f_id, m), m)) {
        assert(j < 3);
        res[j] = internal::from_cgal(v_id);
        ++j;
    }
    assert(j == 3);
    return res;
}

FaceId
random_face(const Mesh& mesh) {
    static std::random_device                  rand_dev;
    static std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<std::size_t> distribution(0, mesh.num_faces() - 1);
    return FaceId{static_cast<Index>(distribution(generator))};
}

}  // namespace mdv::mesh
