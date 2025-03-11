#include "mdv/dmp/dmp.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

TEST(Dmp, ScalarDmp) {
    using mdv::Dmp;
    using mdv::riemann::Scalar;
    using Demonstration = mdv::Demonstration<Scalar>;
    using namespace std::chrono_literals;

    auto get_position_vector = [](const auto& demo) -> Eigen::VectorXd {
        Eigen::VectorXd res(demo.size());
        for (auto i = 0; i < res.size(); ++i) res(i) = demo[i].y();
        return res;
    };

    const Demonstration demo = mdv::build_scalar_demonstration();
    const auto          n_ts = demo.size();

    Dmp<Scalar> dmp;
    dmp.learn(demo);
    ASSERT_FALSE(dmp.weights().hasNaN());
    ASSERT_EQ(dmp.tau, mdv::convert::seconds(demo.back().t() - demo.front().t()));

    // Evaluate learned forcing term
    const auto      fdes = dmp.evaluate_desired_forcing_term(demo);
    Eigen::VectorXd fval(fdes.size());
    for (auto i = 0; i < fdes.size(); ++i)
        fval(i) = dmp.eval_weighted_basis(mdv::convert::seconds(demo[i].t()));

    const double f_mae = (fdes - fval).cwiseAbs().mean();
    ASSERT_EQ(fdes.rows(), n_ts);
    // ASSERT_LE(f_mae, 1) << "Forcing term mean absolute error";

    const auto res =
            dmp.integrate(demo.front().y(), demo.back().y(), n_ts, demo[1].t());
    ASSERT_EQ(res.size(), demo.size());

    const auto   ydem_vec = get_position_vector(demo);
    const auto   yrec_vec = get_position_vector(res);
    const double mae      = (ydem_vec - yrec_vec).cwiseAbs().mean();

    ASSERT_LE(mae, 0.02) << "Reconstructed position mean absolute error";
}

TEST(Dmp, QuaternionDmp) {
    using M    = mdv::riemann::S3;
    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::Dmp<M>;

    const Demo demo = mdv::build_quaternion_demonstration();
    const long n_ts = demo.size();

    Dmp dmp;
    dmp.learn(demo);
    const Demo res =
            dmp.integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());
    ASSERT_FALSE(dmp.weights().hasNaN());
    ASSERT_EQ(dmp.tau, mdv::convert::seconds(demo.back().t() - demo.front().t()));

    ASSERT_EQ(res.size(), demo.size());

    Eigen::VectorXd angle_error(n_ts);
    for (auto i = 0; i < n_ts; ++i) {
        const auto delta = demo[i].y().inverse() * res[i].y();
        angle_error(i)   = std::acos(std::abs(delta.w())) * 180.0 / M_PI;
        ASSERT_TRUE(mdv::condition::is_unit_norm(res[i].y().coeffs()))
                << "Quaternions obtained by dynamic integration must have unit norm";
    }
    const double mae = angle_error.cwiseAbs().mean();
    ASSERT_LE(mae, 3) << "Reconstructed orientation mean absolute error (degrees)";
}
