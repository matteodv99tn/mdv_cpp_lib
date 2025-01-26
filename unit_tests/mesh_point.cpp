#include <gtest/gtest.h>

#include <mdv/config.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/conditions.hpp>

using mdv::mesh::Mesh;

TEST(MdvMesh, PointToBarycentric) {
    using mdv::condition::is_zero_norm;
    const std::string mesh_path =
            std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    const auto            mesh   = Mesh::from_file(mesh_path);
    const Eigen::Vector3d b1_des = {1.0, 0.0, 0.0};
    const Eigen::Vector3d b2_des = {0.0, 1.0, 0.0};
    const Eigen::Vector3d b3_des = {0.0, 0.0, 1.0};

    using Face    = Mesh::Face;
    using Point   = Mesh::Point;
    using UvCoord = Mesh::Point::UvCoord;

    for (int i = 0; i < 256; ++i) {
        // Check conversion to barycentric coordinates
        const auto face     = Face::random(mesh);
        const auto pt_on_v1 = Point(face, UvCoord({0.0, 0.0}));
        const auto pt_on_v2 = Point(face, UvCoord({1.0, 0.0}));
        const auto pt_on_v3 = Point(face, UvCoord({0.0, 1.0}));

        EXPECT_NEAR(pt_on_v1.barycentric()[0], b1_des[0], 1e-9);
        EXPECT_NEAR(pt_on_v2.barycentric()[1], b2_des[1], 1e-9);
        EXPECT_NEAR(pt_on_v3.barycentric()[2], b3_des[2], 1e-9);

        // Create random point, and check that different constructors works as expected
        const auto pt_random = Point::random(mesh);
        const auto pt_reconstructed =
                Mesh::Point::from_cartesian(mesh, pt_random.position());
        EXPECT_NEAR(
                pt_random.position()[0] - pt_reconstructed.position()[0], 0.0, 1e-9
        );
        EXPECT_NEAR(
                pt_random.position()[1] - pt_reconstructed.position()[1], 0.0, 1e-9
        );
        EXPECT_NEAR(
                pt_random.position()[2] - pt_reconstructed.position()[2], 0.0, 1e-9
        );
    }
}
