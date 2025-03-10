#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "mdv/riemann_geometry/hypershere.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/utils/conditions.hpp"

using mdv::condition::are_equal;

TEST(HyperSphere, S1) {
    using S1  = mdv::riemann::S<1>;
    using Vec = S1::Vec;

    const Vec p1(0.0, 1.0);
    const Vec q1 = Vec(1.0, 0.0);
    const Vec v1 = Vec(1.0, 0.0) * M_PI / 2.0;
    const Vec w1 = Vec(0.0, -1.0) * M_PI / 2.0;
    ASSERT_TRUE(are_equal(S1::exponential_map(p1, v1), q1));
    ASSERT_TRUE(are_equal(S1::logarithmic_map(p1, q1), v1));
    ASSERT_TRUE(are_equal(S1::parallel_transport(p1, p1, v1), v1));
    ASSERT_TRUE(are_equal(S1::parallel_transport(p1, q1, v1), w1));
    ASSERT_TRUE(are_equal(S1::covariant_derivative(p1, v1), v1));
    ASSERT_TRUE(are_equal(S1::covariant_derivative(p1, w1), Vec::Zero()));

    const Vec q2 = Vec(1.0, 1.0).normalized();
    const Vec v2 = Vec(1.0, 0.0) * M_PI / 4.0;
    ASSERT_TRUE(are_equal(S1::exponential_map(p1, v2), q2));
    ASSERT_TRUE(are_equal(S1::logarithmic_map(p1, q2), v2));

    // Covariant derivative test
    const Vec v2cd = Vec(1.0, 0.0);
    const Vec w2cd = Vec(0.5, -0.5);
    ASSERT_TRUE(are_equal(S1::covariant_derivative(q2, v2cd), w2cd));

    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(1.0, 0.0), Vec(1.0, 0.0)), Vec(0.0, 0.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(1.0, 0.0), Vec(0.0, 1.0)), Vec(0.0, 1.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(1.0, 0.0), Vec(1.0, 1.0)), Vec(0.0, 1.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(1.0, 0.0), Vec(0.5, 2.0)), Vec(0.0, 2.0)
    ));

    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(0.0, 1.0), Vec(1.0, 0.0)), Vec(1.0, 0.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(0.0, 1.0), Vec(0.0, 1.0)), Vec(0.0, 0.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(0.0, 1.0), Vec(1.0, 1.0)), Vec(1.0, 0.0)
    ));
    ASSERT_TRUE(are_equal(
            S1::covariant_derivative(Vec(0.0, 1.0), Vec(0.5, 2.0)), Vec(0.5, 0.0)
    ));


    const Vec p3 = Vec(1.0, 1.0).normalized();
    const Vec q3 = Vec(0.0, 1.0);
    const Vec v3 = Vec(-1.0, 1.0).normalized() * M_PI / 4.0;
    ASSERT_TRUE(are_equal(S1::exponential_map(p3, v3), q3));
    ASSERT_TRUE(are_equal(S1::logarithmic_map(p3, q3), v3));
}

TEST(HyperSphere, S2) {
    using S2  = mdv::riemann::S<2>;
    using Vec = S2::Vec;

    const Vec p1(0.0, 0.0, 1.0);
    const Vec q1 = Vec(1.0, 0.0, 0.0);
    const Vec v1 = Vec(1.0, 0.0, 0.0) * M_PI / 2.0;
    const Vec w1 = Vec(0.0, 0.0, -1.0) * M_PI / 2.0;

    ASSERT_TRUE(q1.isApprox(S2::exponential_map(p1, v1)));
    ASSERT_TRUE(are_equal(S2::logarithmic_map(p1, q1), v1));
    ASSERT_TRUE(are_equal(S2::logarithmic_map(p1, p1), Vec::Zero()));
    ASSERT_TRUE(are_equal(S2::exponential_map(p1, Vec::Zero()), p1));
    ASSERT_TRUE(are_equal(S2::parallel_transport(p1, p1, v1), v1));
    ASSERT_TRUE(are_equal(S2::parallel_transport(p1, q1, v1), w1));

    const Vec v1b(0.0, 1.0, 0.0);
    const Vec w1b(0.0, 1.0, 0.0);
    ASSERT_TRUE(are_equal(S2::parallel_transport(p1, q1, v1b), w1b));
    const Vec v1c(1.0, 1.0, 0.0);
    const Vec w1c(0.0, 1.0, -1.0);
    ASSERT_TRUE(are_equal(S2::parallel_transport(p1, q1, v1c), w1c));

    const Vec q2 = Vec(1.0, 0.0, 1.0).normalized();
    const Vec v2 = Vec(1.0, 0.0, 0.0) * M_PI / 4.0;
    ASSERT_TRUE(are_equal(S2::exponential_map(p1, v2), q2));
    ASSERT_TRUE(are_equal(S2::logarithmic_map(p1, q2), v2));

    const Vec q3 = Vec(1.0, 1.0, 0.0).normalized();
    const Vec v3 = Vec(1.0, 1.0, 0.0).normalized() * M_PI / 2.0;
    ASSERT_TRUE(are_equal(S2::exponential_map(p1, v3), q3));
    ASSERT_TRUE(are_equal(S2::logarithmic_map(p1, q3), v3));


    const Eigen::AngleAxis rot(M_PI / 4.0, Eigen::Vector3d::UnitZ());
    const Vec              q4 = rot * q2;
    const Vec              v4 = rot * v2;
    ASSERT_TRUE(are_equal(S2::exponential_map(p1, v4), q4));
    ASSERT_TRUE(are_equal(S2::logarithmic_map(p1, q4), v4));

    ASSERT_TRUE(
            are_equal(S2::covariant_derivative(Vec::UnitX(), Vec::UnitX()), Vec::Zero())
    );
    ASSERT_TRUE(are_equal(
            S2::covariant_derivative(Vec::UnitX(), Vec::UnitY()), Vec::UnitY()
    ));
    ASSERT_TRUE(are_equal(
            S2::covariant_derivative(Vec::UnitX(), Vec::UnitZ()), Vec::UnitZ()
    ));
    ASSERT_TRUE(are_equal(
            S2::covariant_derivative(Vec::UnitX(), Vec(1.0, 2.0, 3.0)),
            Vec(0.0, 2.0, 3.0)
    ));
    ASSERT_TRUE(are_equal(
            S2::covariant_derivative(Vec::UnitY(), Vec(1.0, 2.0, 3.0)),
            Vec(1.0, 0.0, 3.0)
    ));
}

TEST(HyperSphere, S3) {
    using S3   = mdv::riemann::S3;
    using Quat = S3::Point;
    using Vec  = S3::TangentVector;

    for (int i = 0; i < 100; ++i) {
        const Quat q1 = Quat::UnitRandom();
        const Quat q2 = Quat::UnitRandom();
        ASSERT_TRUE(are_equal(
                S3::logarithmic_map(q1, q2),
                -S3::parallel_transport(q2, q1, S3::logarithmic_map(q2, q1))
        ));
    }
}
