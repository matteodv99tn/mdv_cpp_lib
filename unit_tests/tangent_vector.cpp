#include "mdv/mesh/tangent_vector.hpp"

#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <gtest/gtest.h>
#include <numbers>
#include <stdexcept>
#include <unistd.h>
#include <vector>

#include "mdv/config.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/sampler.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {
namespace fs = std::filesystem;

TEST(MdvMesh, TangentVectorUnitRandomInitialisation) {
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);
    Sampler        sampler(mesh, 42);

    for (int i{0}; i < 256; ++i) {
        const auto pt  = sampler.point_in_face();
        const auto vec = sampler.unit_tangent(pt);
        ASSERT_TRUE(mdv::condition::is_unit_norm(tip(mesh, vec) - position(mesh, pt)));
    }
}

TEST(MdvMesh, TangentVectorFromTipInitialisation) {
    using Vec3               = Eigen::Vector3d;
    using Mat3               = Eigen::Matrix3d;
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);
    const auto     pref_dir  = Vec3::UnitX();
    Sampler        sampler(mesh, 42);

    for (int i{0}; i < 256; ++i) {
        const auto pt    = sampler.point_in_face();
        const auto n     = normal(mesh, face(mesh, pt));
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = position(mesh, pt) + delta;

        const auto vec = TangentVector::from_tip_position(mesh, pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(mesh::tip(mesh, vec) - tip));
    }
}

TEST(MdvMesh, TangentVectorDifference) {
    using Vec3               = Eigen::Vector3d;
    using Mat3               = Eigen::Matrix3d;
    const fs::path mesh_path = mdv::config::meshes_directory() / "torus_simple.off";
    const auto     mesh      = Mesh::from_file(mesh_path);
    const auto     pref_dir  = Vec3::UnitX();
    Sampler        sampler(mesh, 42);

    for (int i{0}; i < 256; ++i) {
        const auto pt = sampler.point_in_face();
        const auto v1 = sampler.unit_tangent(pt);
        const auto v2 = sampler.unit_tangent(pt);

        const auto n     = normal(mesh, face(mesh, pt));
        const Vec3 delta = (Mat3::Identity() - n * n.transpose()) * pref_dir;
        const Vec3 tip   = position(mesh, pt) + delta;

        const auto vec = TangentVector::from_tip_position(mesh, pt, tip);
        ASSERT_TRUE(mdv::condition::are_orthogonal(n, delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(vec.cartesian_vector() - delta));
        ASSERT_TRUE(mdv::condition::is_zero_norm(mesh::tip(mesh, vec) - tip));
    }
}

TEST(MdvMesh, TangentVectorOrder) {
    using internal::vector_inside_triangle;
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

    const fs::path mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto     mesh      = Mesh::from_file(mesh_path);

    const auto p_v = from_cartesian(mesh, {-50, -50, -50});
    ASSERT_EQ(location_type(p_v), LocationType::ON_VERTEX);
    {
        const auto tv = TangentVector::from_ambient_vector(mesh, p_v, {0, 0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv =
                TangentVector::from_ambient_vector(mesh, p_v, {0.0, -5.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv =
                TangentVector::from_ambient_vector(mesh, p_v, {-5.0, -5.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(mesh, p_v, {0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }
    {
        const auto tv =
                TangentVector::from_ambient_vector(mesh, p_v, {-5.0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }

    const auto p_he = from_cartesian(mesh, {-50, 0.0, -50});
    ASSERT_EQ(location_type(p_he), LocationType::ON_EDGE);
    {
        const auto tv = TangentVector::from_ambient_vector(mesh, p_he, {0, 0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(mesh, p_he, {0, 10.0, 10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 10.0}));
    }
    {
        const auto tv = TangentVector::from_ambient_vector(mesh, p_he, {0, 10.0, 0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 0}));
    }
    {
        const auto tv =
                TangentVector::from_ambient_vector(mesh, p_he, {0, 10.0, -10.0});
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), Eigen::Vector3d{0, 10.0, 0}));
    }
    {
        const auto tv =
                TangentVector::from_ambient_vector(mesh, p_he, {-10.0, 10.0, -10.0});
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

    const fs::path mesh_path = mdv::config::meshes_directory() / "cube.stl";
    const auto     mesh      = Mesh::from_file(mesh_path);

    {
        // Trivial exponential map: start and end at middle of the face by passing on 1
        // edge
        const Vec3 p0_value{50, 0, 0};
        const Vec3 tv_value{0, 0, 100};

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_TRUE(are_equal(position(mesh, res), Vec3{0, 0, 50}));
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

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto p1 = from_cartesian(mesh, p1_value);
        const auto p2 = from_cartesian(mesh, p2_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::INSIDE_FACE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::INSIDE_FACE);
        ASSERT_EQ(tv.type(), TangentVector::Type::INSIDE_FACE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_TRUE(are_equal(position(mesh, res), p2_value));
        ASSERT_EQ(geod.size(), 3);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
    }
    {
        // Exponential map with starting on edge and finishing on face

        // Vertex position to pass through; choice is made to avoid that the landing on
        // the new face coincides with an edge
        const Vec3 p1_value{50, -50, 50};
        const Vec3 p0_value{50, -50, 0};

        const Vec3 delta2   = AS(M_PI_4, Vec3::UnitZ()) * (50.0 * Vec3::UnitY());
        const Vec3 p2_value = p1_value + delta2;
        const Vec3 tv_value{0, 0, 100};

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto p1 = from_cartesian(mesh, p1_value);
        const auto p2 = from_cartesian(mesh, p2_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::INSIDE_FACE);
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_TRUE(are_equal(position(mesh, res), p2_value));
        ASSERT_EQ(geod.size(), 3);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
    }
    {
        // Exponential map running along edges

        // Vertex position to pass through; choice is made so that the "landing" lies on
        // another edge
        const Vec3 p1_value{50, 50, 50};
        const Vec3 p0_value{50, 50, 0};

        const Vec3 delta2   = AS(M_PI_4, Vec3::UnitZ()) * (-50.0 * Vec3::UnitX());
        const Vec3 p2_value = p1_value + delta2;
        const Vec3 tv_value{0, 0, 100};

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto p1 = from_cartesian(mesh, p1_value);
        const auto p2 = from_cartesian(mesh, p2_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::ON_EDGE);
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_TRUE(are_equal(position(mesh, res), p2_value));
        ASSERT_EQ(geod.size(), 3);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
    }

    {
        // Exponential map running along multiple edges

        // Vertex position to pass through; choice is made so that the "landing" lies on
        // another edge
        const Vec3 p1_value{50, 50, 50};
        const Vec3 p0_value{50, 50, 0};
        const Vec3 p2_value{-50, -50, 50};
        const Vec3 p3_value{-50, -50, 0};

        const Vec3 tv_value{0, 0, 100 + 100 * std::numbers::sqrt2};

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto p1 = from_cartesian(mesh, p1_value);
        const auto p2 = from_cartesian(mesh, p2_value);
        const auto p3 = from_cartesian(mesh, p3_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p3), LocationType::ON_EDGE);
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_EQ(geod.size(), 4);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
        ASSERT_TRUE(are_equal(geod[3], p3_value));
        ASSERT_TRUE(are_equal(position(mesh, res), p3_value))
                << position(mesh, res).transpose();
    }

    {
        // Exponential map running along multiple vertices and edges

        // Vertex position to pass through; choice is made so that the "landing" lies on
        // another edge
        const Vec3 p0_value{50, -50, 0};
        const Vec3 p1_value{50, -50, 50};
        const Vec3 p2_value{0, 0, 50};
        const Vec3 p3_value{-50, 50, 50};
        const Vec3 p4_value{-50, 50, 0};

        const Vec3 tv_value{0, 0, 100 + 100 * std::numbers::sqrt2};

        const auto p0 = from_cartesian(mesh, p0_value);
        const auto p1 = from_cartesian(mesh, p1_value);
        const auto p2 = from_cartesian(mesh, p2_value);
        const auto p3 = from_cartesian(mesh, p3_value);
        const auto p4 = from_cartesian(mesh, p4_value);
        const auto tv = TangentVector::from_ambient_vector(mesh, p0, tv_value);
        ASSERT_TRUE(are_equal(position(mesh, p0), p0_value));
        ASSERT_TRUE(are_equal(tv.cartesian_vector(), tv_value));
        ASSERT_EQ(location_type(p0), LocationType::ON_EDGE);
        ASSERT_EQ(location_type(p1), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p2), LocationType::ON_EDGE);
        ASSERT_EQ(location_type(p3), LocationType::ON_VERTEX);
        ASSERT_EQ(location_type(p4), LocationType::ON_EDGE);
        ASSERT_EQ(tv.type(), TangentVector::Type::ALONG_EDGE);

        Geodesic   geod;
        const auto res = exponential_map(mesh, tv, &geod);

        ASSERT_EQ(geod.size(), 5);
        ASSERT_TRUE(are_equal(geod[0], p0_value));
        ASSERT_TRUE(are_equal(geod[1], p1_value));
        ASSERT_TRUE(are_equal(geod[2], p2_value));
        ASSERT_TRUE(are_equal(geod[3], p3_value));
        ASSERT_TRUE(are_equal(geod[4], p4_value));
        ASSERT_TRUE(are_equal(position(mesh, res), p4_value));
    }
}

TEST(MdvMesh, DeterministicTangentInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "cube.stl");
    const std::vector<Vec3d> positions{
            {50,  -20, 5  },
            {-50, 0,   -50},
            {-50, -50, -50}
    };
    const std::vector<Vec3d> ambient_vectors{
            {7, 3,  4 },
            {0, 10, 0 },
            {0, 0,  10}
    };
    for (std::size_t i = 0; i < positions.size(); ++i) {
        SCOPED_TRACE(i);
        const auto  p = from_cartesian(mesh, positions[i]);
        auto        v = TangentVector::from_ambient_vector(mesh, p, ambient_vectors[i]);
        const Vec3d original = v.cartesian_vector();
        EXPECT_NEAR(
                (position(mesh, v.application_point()) - position(mesh, p)).norm(),
                0.0,
                1e-9
        );
        EXPECT_NEAR(original.dot(normal(mesh, v.face())), 0.0, 1e-9);
        EXPECT_NEAR((tip(mesh, v) - position(mesh, p) - original).norm(), 0.0, 1e-9);
        if (i != 0) {
            ASSERT_EQ(v.type(), TangentVector::ALONG_EDGE);
            EXPECT_NEAR(
                    original.cross(direction(mesh, *v.halfedge()).normalized()).norm(),
                    0.0,
                    1e-9
            );
        }
        const auto unit = v.normalised();
        EXPECT_NEAR(unit.cartesian_vector().norm(), 1.0, 1e-12);
        EXPECT_NEAR((v.cartesian_vector() - original).norm(), 0.0, 1e-12);
        EXPECT_NEAR(
                (position(mesh, unit.application_point()) - position(mesh, p)).norm(),
                0.0,
                1e-9
        );
        v.scale(2.5);
        EXPECT_NEAR((v.cartesian_vector() - 2.5 * original).norm(), 0.0, 1e-9);
    }
}

TEST(MdvMesh, FoldedMapAndTransportInvariants) {
    const auto mesh = Mesh::from_file(config::meshes_directory() / "cube.stl");
    const auto p    = from_cartesian(mesh, {50, -20, 5});
    const auto q    = from_cartesian(mesh, {10, -20, 50});
    ASSERT_EQ(location_type(p), LocationType::INSIDE_FACE);
    ASSERT_EQ(location_type(q), LocationType::INSIDE_FACE);
    const auto   log      = logarithmic_map(mesh, p, q);
    const double distance = length(build_geodesic(mesh, p, q));
    EXPECT_NEAR(log.cartesian_vector().norm(), distance, 1e-9);
    Geodesic   path;
    const auto end = exponential_map(mesh, log, &path);
    ASSERT_GE(path.size(), 2);
    EXPECT_NEAR((path.front() - position(mesh, p)).norm(), 0.0, 1e-9);
    EXPECT_NEAR((path.back() - position(mesh, end)).norm(), 0.0, 1e-9);
    EXPECT_NEAR((position(mesh, end) - position(mesh, q)).norm(), 0.0, 1e-9);
    EXPECT_NEAR(length(path), log.cartesian_vector().norm(), 1e-9);
    EXPECT_LE(length(build_geodesic(mesh, p, end)), length(path) + 1e-9);
    EXPECT_NEAR(
            (position(mesh, from_cartesian(mesh, position(mesh, end)))
             - position(mesh, end))
                    .norm(),
            0.0,
            1e-9
    );

    const TangentVector v(mesh, p, {0, 2, 3});
    const TangentVector w(mesh, p, {0, -4, 1});
    const auto          pv = parallel_transport(mesh, v, q);
    const auto          pw = parallel_transport(mesh, w, q);
    EXPECT_NEAR(pv.cartesian_vector().norm(), v.cartesian_vector().norm(), 1e-9);
    EXPECT_NEAR(pw.cartesian_vector().norm(), w.cartesian_vector().norm(), 1e-9);
    EXPECT_NEAR(
            pv.cartesian_vector().dot(pw.cartesian_vector()),
            v.cartesian_vector().dot(w.cartesian_vector()),
            1e-9
    );
    EXPECT_NEAR(pv.cartesian_vector().dot(normal(mesh, pv.face())), 0.0, 1e-9);
    EXPECT_NEAR(
            (position(mesh, pv.application_point()) - position(mesh, q)).norm(),
            0.0,
            1e-9
    );
    EXPECT_NEAR(
            (parallel_transport(mesh, v, p).cartesian_vector() - v.cartesian_vector())
                    .norm(),
            0.0,
            1e-9
    );
}

TEST(TangentVector, FrameAndNormalisationSurviveMeshMove) {
    auto       mesh = Mesh::from_file(config::meshes_directory() / "cube.stl");
    const auto p    = from_cartesian(mesh, {-50, -50, -50});
    const auto v    = TangentVector::from_ambient_vector(mesh, p, {0, 20, 10});
    const auto unit = v.normalised();
    EXPECT_EQ(unit.face(), v.face());
    EXPECT_EQ(unit.halfedge(), v.halfedge());
    EXPECT_EQ(unit.application_point(), v.application_point());
    EXPECT_NEAR(unit.cartesian_vector().norm(), 1.0, 1e-12);
    const Vec3d original_tip = tip(mesh, v);
    auto        moved        = std::move(mesh);
    EXPECT_NEAR(v.cartesian_vector().dot(normal(moved, v.face())), 0.0, 1e-12);
    EXPECT_NEAR((tip(moved, v) - original_tip).norm(), 0.0, 1e-12);
}

// Regression: the geodesic crosses edge 67-1382 about 1e-8 away from vertex 67; the
// walk then bounced between faces 381 and 53, firing an assert in Debug and hanging
// in Release. Run in a child process so both failures are reported, not fatal.
TEST(TangentVectorDeathTest, ExponentialMapNearVertexTerminates) {
    ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
    EXPECT_EXIT(
            {
                alarm(2);
                const auto mesh = Mesh::from_file(
                        config::meshes_directory() / "bunny_simple.off"
                );
                const Point p(
                        mesh,
                        FaceId{381},
                        {0.0082300908311814247,
                         0.88294487025889135,
                         0.10882503890992713}
                );
                const TangentVector v(
                        mesh,
                        p,
                        {-0.63980435339904296,
                         -0.043272505502871726,
                         0.89804960738758843}
                );
                static_cast<void>(exponential_map(mesh, v));
                std::exit(0);
            },
            ::testing::ExitedWithCode(0),
            ""
    );
}

}  // namespace mdv::mesh
