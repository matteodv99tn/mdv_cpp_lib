#include "mdv/dmp/dmp.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"
#include "mdv/utils/conditions.hpp"

static_assert(mdv::Embedding<mdv::riemann::Scalar::TangentVector>::dimension == 1);
static_assert(mdv::Embedding<mdv::riemann::S3::TangentVector>::dimension == 4);
static_assert(mdv::Embedding<mdv::riemann::SE3::TangentVector>::dimension == 7);
static_assert(mdv::embedding_dimension<mdv::riemann::Scalar::TangentVector> == 1);
static_assert(mdv::embedding_dimension<mdv::riemann::S3::TangentVector> == 4);
static_assert(mdv::embedding_dimension<mdv::riemann::SE3::TangentVector> == 7);

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
        fval(i) = dmp.fun()(mdv::convert::seconds(demo[i].t()));

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

TEST(Dmp, R3Dmp) {
    using M    = mdv::riemann::Rn<3>;
    using Demo = mdv::Demonstration<M>;
    using Dmp  = mdv::Dmp<M>;

    const Demo demo = mdv::build_position_demonstration();
    const long n_ts = demo.size();

    Dmp dmp;
    dmp.learn(demo);
    const Demo res =
            dmp.integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());
    ASSERT_FALSE(dmp.weights().hasNaN());
    ASSERT_EQ(dmp.tau, mdv::convert::seconds(demo.back().t() - demo.front().t()));

    ASSERT_EQ(res.size(), demo.size());

    Eigen::VectorXd abs_error(n_ts);
    for (auto i = 0; i < n_ts; ++i) {
        const auto   delta = demo[i].y() - res[i].y();
        const double err   = std::abs(delta.norm());
        abs_error(i)       = err;
        // ASSERT_TRUE(mdv::condition::is_unit_norm(res[i].y().coeffs()));
    }
    const double mae = abs_error.cwiseAbs().mean();
    ASSERT_LE(mae, 0.01) << "Reconstructed orientation mean absolute error (degrees)";
}

TEST(Dmp, S3Dmp) {
    auto get_rotation_deg = [](double w) -> double {
        w                      = std::abs(w);
        const double angle_rad = std::acos(w);
        return angle_rad * 180.0 / M_PI;
    };
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

    Eigen::VectorXd angle_error = Eigen::VectorXd::Zero(n_ts);
    for (auto i = 0; i < n_ts; ++i) {
        const Eigen::Quaterniond delta = demo[i].y().inverse() * res[i].y();
        ASSERT_FALSE(delta.coeffs().hasNaN());
        const double w = std::abs(delta.w());
        angle_error(i) = (w < 1.0) ? get_rotation_deg(w) : 0.0;
        ASSERT_FALSE(angle_error.hasNaN());
        ASSERT_TRUE(mdv::condition::is_unit_norm(res[i].y().coeffs()));
    }
    ASSERT_FALSE(angle_error.hasNaN());
    const double mae = angle_error.cwiseAbs().mean();
    ASSERT_LE(mae, 3) << "Reconstructed orientation mean absolute error (degrees)";
}

TEST(Dmp, SE3Dmp) {
    using R3     = mdv::riemann::Rn<3>;
    using S3     = mdv::riemann::S3;
    using SE3    = mdv::riemann::SE3;
    using Demo   = mdv::Demonstration<SE3>;
    using Se3Dmp = mdv::Dmp<SE3>;
    using R3Dmp  = mdv::Dmp<R3>;
    using S3Dmp  = mdv::Dmp<S3>;
    using Se3Dmp = mdv::Dmp<SE3>;

    const auto r3_demo  = mdv::build_position_demonstration();
    const auto s3_demo  = mdv::build_quaternion_demonstration();
    const auto se3_demo = mdv::build_se3_demonstration();
    const long n_ts     = se3_demo.size();
    ASSERT_EQ(r3_demo.size(), s3_demo.size());
    ASSERT_EQ(r3_demo.size(), se3_demo.size());

    R3Dmp  r3_dmp;
    S3Dmp  s3_dmp;
    Se3Dmp se3_dmp;
    r3_dmp.learn(r3_demo);
    s3_dmp.learn(s3_demo);
    se3_dmp.learn(se3_demo);
    const auto r3_res = r3_dmp.integrate(
            r3_demo.front().y(), r3_demo.back().y(), r3_demo.size(), r3_demo[1].t()
    );
    const auto s3_res = s3_dmp.integrate(
            s3_demo.front().y(), s3_demo.back().y(), s3_demo.size(), s3_demo[1].t()
    );
    const auto se3_res = se3_dmp.integrate(
            se3_demo.front().y(), se3_demo.back().y(), se3_demo.size(), se3_demo[1].t()
    );
    ASSERT_FALSE(se3_dmp.weights().hasNaN());
    ASSERT_EQ(
            se3_dmp.tau,
            mdv::convert::seconds(se3_demo.back().t() - se3_demo.front().t())
    );
    ASSERT_EQ(se3_res.size(), se3_demo.size());

    ASSERT_EQ(r3_dmp.weights().rows(), s3_dmp.weights().rows());
    ASSERT_EQ(r3_dmp.weights().rows(), se3_dmp.weights().rows());

    for (long i = 0; i < se3_dmp.weights().rows(); ++i) {
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 0), r3_dmp.weights()(i, 0));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 1), r3_dmp.weights()(i, 1));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 2), r3_dmp.weights()(i, 2));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 3), s3_dmp.weights()(i, 0));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 4), s3_dmp.weights()(i, 1));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 5), s3_dmp.weights()(i, 2));
        ASSERT_FLOAT_EQ(se3_dmp.weights()(i, 6), s3_dmp.weights()(i, 3));
    }

    for (std::size_t i = 0; i < se3_res.size(); ++i) {
        const double s = se3_dmp.time_to_s(mdv::convert::seconds(se3_res[i].t()));
        const SE3::TangentVector se3_f = se3_dmp.fun()(s, s);
        const auto               s3_f  = s3_dmp.fun()(s, s);
        ASSERT_TRUE(mdv::condition::is_zero_norm(se3_f.ori - s3_f))
                << "At iter " << i << " of " << se3_res.size();
        ASSERT_EQ(se3_res[i].y().pos, r3_res[i].y());
        ASSERT_TRUE(
                mdv::condition::is_zero_norm(
                        se3_res[i].y().ori.coeffs() - s3_res[i].y().coeffs()
                )
        ) << "At iter "
          << i << " of " << se3_res.size();
    }

    // Eigen::VectorXd angle_error(n_ts);
    // for (auto i = 0; i < n_ts; ++i) {
    //     const auto delta = demo[i].y().inverse() * res[i].y();
    //     angle_error(i)   = std::acos(std::abs(delta.w())) * 180.0 / M_PI;
    //     ASSERT_TRUE(mdv::condition::is_unit_norm(se3_res[i].y().coeffs()))
    //             << "Quaternions obtained by dynamic integration must have unit norm";
    // }
    // const double mae = angle_error.cwiseAbs().mean();
    // ASSERT_LE(mae, 3) << "Reconstructed orientation mean absolute error (degrees)";
}

static_assert(mdv::transformation_system<
              mdv::dmp::TransformationSystem<mdv::riemann::Scalar>,
              mdv::riemann::Scalar>);
