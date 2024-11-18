#include <catch2/catch_test_macros.hpp>
#include <mdv/config.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/conditions.hpp>

using mdv::mesh::Mesh;

TEST_CASE("Conversion to barycentric coordinates", "[mesh][point]") {
    using mdv::condition::is_zero_norm;
    const std::string     mesh_path = std::string(mdv::config::mesh_dir) + "/torus.off";
    const Mesh            mesh(mesh_path);
    const Eigen::Vector3d b1_des = {1.0, 0.0, 0.0};
    const Eigen::Vector3d b2_des = {0.0, 1.0, 0.0};
    const Eigen::Vector3d b3_des = {0.0, 0.0, 1.0};

    for (int i = 0; i < 256; ++i) {
        // Check conversion to barycentric coordinates
        const auto face     = Mesh::Face::random(mesh);
        const auto pt_on_v1 = Mesh::Point(face, {0.0, 0.0});
        const auto pt_on_v2 = Mesh::Point(face, {1.0, 0.0});
        const auto pt_on_v3 = Mesh::Point(face, {0.0, 1.0});

        REQUIRE(is_zero_norm(pt_on_v1.barycentric() - b1_des));
        REQUIRE(is_zero_norm(pt_on_v2.barycentric() - b2_des));
        REQUIRE(is_zero_norm(pt_on_v3.barycentric() - b3_des));

        // Create random point, and check that different constructors works as expected
        const auto pt_random = Mesh::Point::random(mesh);
        const auto pt_reconstructed =
                Mesh::Point::from_cartesian(mesh, pt_random.position());
        REQUIRE(is_zero_norm(pt_random.position() - pt_reconstructed.position()));
    }
}
