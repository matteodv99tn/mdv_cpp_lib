#include <iostream>

#include "mdv/config.hpp"
#include "mdv/mesh/cgal_data.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"

using mdv::mesh::Mesh;

// Test deprecated:
// Order of vertices within CGAL is different from the one used internally by the
// library, thus the test is not valid anymore. Another, more appropriate, test shall be
// developed.
//
// TEST_CASE("Conversion to barycentric coordinates", "[mesh][cgal]") {
//     using mdv::mesh::location_from_mesh_point;
//     const std::string     mesh_path = std::string(mdv::config::mesh_dir) +
//     "/torus.off"; const auto            mesh      = Mesh::from_file(mesh_path); const
//     Eigen::Vector3d b1_des    = {1.0, 0.0, 0.0}; const Eigen::Vector3d b2_des    =
//     {0.0, 1.0, 0.0}; const Eigen::Vector3d b3_des    = {0.0, 0.0, 1.0};
//
//     for (int i = 0; i < 256; ++i) {
//         const auto pt         = Mesh::Point::random(mesh);
//         const auto [id, locs] = location_from_mesh_point(pt);
//         const Eigen::Vector3d b(locs[0], locs[1], locs[2]);
//         REQUIRE(static_cast<Mesh::Face::Index>(id) == pt.face_id());
//         REQUIRE(mdv::condition::is_zero_norm(b - pt.barycentric()));
//     }
// }
