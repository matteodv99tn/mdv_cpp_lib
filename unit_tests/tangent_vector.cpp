#include "mdv/mesh/tangent_vector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <gtest/gtest.h>
#include <stdexcept>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"

using namespace mdv::mesh;

using std::filesystem::path;

TEST(MdvMesh, TangentVectorUnitRandomInitialisation) {
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);

    for (int i{0}; i < 256; ++i) {
        const auto pt  = Point::random(mesh);
        const auto vec = TangentVector::unit_random(pt);
        ASSERT_TRUE(mdv::condition::is_unit_norm(vec.tip() - pt.position()));
    }
}

TEST(MdvMesh, TangentVectorFromTipInitialisation) {
    using Vec3           = Eigen::Vector3d;
    using Mat3           = Eigen::Matrix3d;
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);
    const auto pref_dir  = Vec3::UnitX();

    for (int i{0}; i < 256; ++i) {
        const auto pt    = Point::random(mesh);
        const auto n     = pt.face().normal();
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = pt.position() + delta;

        const auto vec = TangentVector::from_tip_position(pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.tip() - tip));
    }
}

TEST(MdvMesh, TangentVectorDifference) {
    using Vec3           = Eigen::Vector3d;
    using Mat3           = Eigen::Matrix3d;
    const path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto mesh      = Mesh::from_file(mesh_path);
    const auto pref_dir  = Vec3::UnitX();

    for (int i{0}; i < 256; ++i) {
        const auto pt = Point::random(mesh);
        const auto v1 = TangentVector::unit_random(pt);
        const auto v2 = TangentVector::unit_random(pt);

        const auto n     = pt.face().normal();
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = pt.position() + delta;

        const auto vec = TangentVector::from_tip_position(pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.tip() - tip));
    }
}

TEST(MdvMesh, TangentVectorOrder) {
    using mdv::mesh::internal::vector_inside_triangle;
    using Vec3 = Eigen::Vector3d;

    ASSERT_THROW(
            vector_inside_triangle(Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()),
            std::runtime_error
    );

    const std::vector<double> v1_v3_angles{
            0.02, M_PI / 3, M_PI / 2, 2 * M_PI / 3, M_PI - 0.01
    };

    for (const double angle : v1_v3_angles) {
        const Vec3 v1 = Vec3::UnitX();
        const Vec3 v3 = Eigen::AngleAxisd(angle, Vec3::UnitZ()) * Vec3::UnitX();

        const Eigen::VectorXd valid_angles =
                Eigen::VectorXd::LinSpaced(50, 0.01, angle - 0.001);
        const Eigen::VectorXd invalid_angles =
                Eigen::VectorXd::LinSpaced(50, angle + 0.01, 2.0 * M_PI - 0.01);

        for (const double theta : valid_angles) {
            const Vec3 v2 = Eigen::AngleAxisd(theta, Vec3::UnitZ()) * Vec3::UnitX();
            ASSERT_TRUE(vector_inside_triangle(v1, v2, v3))
                    << "v1 = " << v1.transpose() << "\n"
                    << "v2 = " << v2.transpose() << "\n"
                    << "v3 = " << v3.transpose() << "\n";
            ASSERT_TRUE(vector_inside_triangle(v3, v2, v1));
        }
        for (const double theta : invalid_angles) {
            const Vec3 v2 = Eigen::AngleAxisd(theta, Vec3::UnitZ()) * Vec3::UnitX();
            ASSERT_FALSE(vector_inside_triangle(v1, v2, v3))
                    << "v1 = " << v1.transpose() << "\n"
                    << "v2 = " << v2.transpose() << "\n"
                    << "v3 = " << v3.transpose() << "\n"
                    << "Angle: " << angle << "\n"
                    << "Theta: " << theta;
            ASSERT_FALSE(vector_inside_triangle(v3, v2, v1));
        }
    };
}

TEST(MdvMesh, TangentVectorConstructor) {
    // Cube is 100x100x100 in size, and the centroid of the cube is at the origin
    // -> Cube stays in [-50, -50, -50] x [50, 50, 50]
    using mdv::condition::are_equal;

    const path mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto mesh      = Mesh::from_file(mesh_path);

    const auto p_v = Point::from_cartesian(mesh, {-50, -50, -50});
    ASSERT_EQ(location_type(p_v), LocationType::ON_VERTEX);
    {
        const auto tv = TangentVector::from_ambient_vector(p_v, {0, 0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_v, {0.0, -5.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_v, {-5.0, -5.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_v, {0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_v, {-5.0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }

    const auto p_he = Point::from_cartesian(mesh, {-50, 0.0, -50});
    ASSERT_EQ(location_type(p_he), LocationType::ON_EDGE);
    {
        const auto tv = TangentVector::from_ambient_vector(p_he, {0, 0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_he, {0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_he, {0, 10.0, 0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_he, {0, 10.0, -10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(p_he, {-10.0, 10.0, -10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 0}));
    }
}

TEST(MdvMesh, ExponentialMap) {
    // Cube is 100x100x100 in size, and the centroid of the cube is at the origin
    // -> Cube stays in [-50, -50, -50] x [50, 50, 50]
    using mdv::condition::are_equal;
    using Vec3 = Eigen::Vector3d;
    using AS   = Eigen::AngleAxisd;

    const path mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto mesh      = Mesh::from_file(mesh_path);

    {
        // Trivial exponential map: start and end at middle of the face by passing on 1
        // edge
        const Vec3 p0_value{50, 0, 0};
        const Vec3 tv_value{0, 0, 100};

        const auto p0 = Point::from_cartesian(mesh, p0_value);
        const auto tv = TangentVector::from_ambient_vector(p0, tv_value);
        ASSERT_TRUE(are_equal(p0.position(), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);

        Geodesic   geod;
        const auto res = exponential_map(tv, &geod);

        ASSERT_TRUE(are_equal(res.position(), Vec3{0, 0, 50}));
        ASSERT_EQ(geod.size(), 3);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], Vec3{50, 0, 50}));
        ASSERT_TRUE(are_equal(geod[2], Vec3{0, 0, 50}));
    }
    {
        // Exponential map with start and end at middle of the face and passing through
        // 1 vertex
        const Vec3 p1_value{50, 50, 50};  // Vertex position to pass through

        // angle of the tangent vector from the vector (0, 0, 1):
        const double theta_deg = 20.0;
        const double theta_rad = theta_deg * M_PI / 180.0;

        const double phi_deg = 270.0 / 2.0 - theta_deg - 90.0;
        const double phi_rad = phi_deg * M_PI / 180.0;
        EXPECT_GT(phi_deg, 0.0);

        // Construction of other points
        const Vec3 delta0   = AS(-theta_rad, Vec3::UnitX()) * (50.0 * Vec3::UnitZ());
        const Vec3 delta2   = AS(phi_rad, Vec3::UnitZ()) * (-50.0 * Vec3::UnitX());
        const Vec3 p0_value = p1_value - delta0;
        const Vec3 p2_value = p1_value + delta2;
        const Vec3 tv_value = (p1_value - p0_value).normalized() * 100.0;

        const auto p0 = Point::from_cartesian(mesh, p0_value);
        const auto p1 = Point::from_cartesian(mesh, p1_value);
        const auto p2 = Point::from_cartesian(mesh, p2_value);
        const auto tv = TangentVector::from_ambient_vector(p0, tv_value);
        ASSERT_TRUE(are_equal(p0.position(), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::INSIDE_FACE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::INSIDE_FACE);
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);

        Geodesic   geod;
        const auto res = exponential_map(tv, &geod);

        ASSERT_TRUE(are_equal(res.position(), p2_value));
        ASSERT_EQ(geod.size(), 3);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
    }
}
