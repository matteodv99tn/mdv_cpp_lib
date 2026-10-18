#include "mdv/utils/eigen_solvers.hpp"

#include <array>
#include <gtest/gtest.h>

#include "mdv/dmp/learnable_function.hpp"

TEST(EigenSolvers, QuaternionHandlesParallelAndAntiparallelVectors) {
    const Eigen::Vector3d                from(2.0, -1.0, 3.0);
    const Eigen::Vector3d                other(-1.0, 4.0, 2.0);
    const std::array<Eigen::Vector3d, 3> targets{from, -from, other};
    for (const Eigen::Vector3d& to : targets) {
        const auto q = mdv::quaternion_from_two_vectors(from, to);
        EXPECT_NEAR(q.norm(), 1.0, 1e-12);
        EXPECT_TRUE((q * from.normalized()).isApprox(to.normalized(), 1e-12));
    }
}

TEST(EigenSolvers, QrHandlesMultipleRightHandSidesAndRankDeficiency) {
    Eigen::MatrixXd a(3, 2);
    a << 1, 0, 0, 1, 1, 1;
    const Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(2, 2);
    const Eigen::MatrixXd rhs      = a * expected;
    EXPECT_TRUE(mdv::fullpiv_qr_solve(a, rhs).isApprox(expected, 1e-12));

    a.col(1)                            = 2.0 * a.col(0);
    const Eigen::MatrixXd deficient_rhs = a * expected;
    const Eigen::MatrixXd solution      = mdv::fullpiv_qr_solve(a, deficient_rhs);
    EXPECT_TRUE(solution.allFinite());
    EXPECT_TRUE((a * solution).isApprox(deficient_rhs, 1e-12));
}

TEST(EigenSolvers, LearnableFunctionSupportsScalarAndPerOutputScaling) {
    using Function = mdv::dmp::LearnableFunction<2>;
    Function              scalar(Function::Basis(2));
    Function              per_output(Function::Basis(2));
    const Eigen::MatrixXd phi     = Eigen::MatrixXd::Identity(2, 2);
    const Eigen::MatrixXd desired = 6.0 * phi;
    scalar.learn(phi, desired, 2.0);
    per_output.learn(phi, desired, Eigen::Vector2d(2.0, 3.0));
    EXPECT_TRUE(scalar.weights().isApprox(3.0 * phi, 1e-12));
    Eigen::MatrixXd expected = phi;
    expected(0, 0)           = 3.0;
    expected(1, 1)           = 2.0;
    EXPECT_TRUE(per_output.weights().isApprox(expected, 1e-12));
}
