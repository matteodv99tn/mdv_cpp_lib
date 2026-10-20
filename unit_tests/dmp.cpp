#include "mdv/dmp/dmp.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/riemann_geometry/concepts.hpp"
#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"
#include "mdv/riemann_geometry/utils.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv {

static_assert(riemann::space_dimension_v<riemann::Scalar::TangentVector> == 1);
static_assert(riemann::space_dimension_v<riemann::S3::TangentVector> == 4);
// static_assert(mdv::riemann::space_dimension_v<mdv::riemann::SE3::TangentVector> ==
// 7);

static_assert(concepts::euclidean_space<riemann::Scalar>);
static_assert(concepts::trivially_embeddable_manifold<riemann::Scalar>);
static_assert(concepts::trivially_embeddable_manifold<riemann::Rn<3>>);
static_assert(concepts::trivially_embeddable_manifold<riemann::S3>);
static_assert(concepts::trivially_embeddable_manifold<riemann::SE3>);

static_assert(mdv::concepts::transformation_system<
              mdv::dmp::TransformationSystem<mdv::riemann::Scalar>,
              mdv::riemann::Scalar>);

TEST(Dmp, ScalarDmp) {
    using mdv::riemann::Scalar;
    using Demonstration = mdv::Demonstration<Scalar>;
    using namespace std::chrono_literals;

    Scalar                                s;
    mdv::DefaultManifoldEmbedding<Scalar> emb(&s);

    auto get_position_vector = [](const auto& demo) -> Eigen::VectorXd {
        Eigen::VectorXd res(demo.size());
        for (Eigen::Index i = 0; i < res.size(); ++i)
            res(i) = demo[static_cast<std::size_t>(i)].y();
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
    for (Eigen::Index i = 0; i < fdes.size(); ++i) {
        fval(i) =
                dmp.fun()(mdv::convert::seconds(demo[static_cast<std::size_t>(i)].t()));
    }

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
    const long n_ts = std::ssize(demo);

    Dmp dmp;
    dmp.learn(demo);
    const Demo res =
            dmp.integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());
    ASSERT_FALSE(dmp.weights().hasNaN());
    ASSERT_EQ(dmp.tau, mdv::convert::seconds(demo.back().t() - demo.front().t()));

    ASSERT_EQ(res.size(), demo.size());

    Eigen::VectorXd abs_error(n_ts);
    for (std::size_t i = 0; i < demo.size(); ++i) {
        const auto   delta                      = demo[i].y() - res[i].y();
        const double err                        = std::abs(delta.norm());
        abs_error(static_cast<Eigen::Index>(i)) = err;
        // ASSERT_TRUE(mdv::condition::is_unit_norm(res[i].y().coeffs()));
    }
    const double mae = abs_error.cwiseAbs().mean();
    ASSERT_LE(mae, 0.01) << "Reconstructed orientation mean absolute error(degrees)";
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
    const long n_ts = std::ssize(demo);

    Dmp dmp;
    dmp.learn(demo);
    const Demo res =
            dmp.integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());
    ASSERT_FALSE(dmp.weights().hasNaN());
    ASSERT_EQ(dmp.tau, mdv::convert::seconds(demo.back().t() - demo.front().t()));

    ASSERT_EQ(res.size(), demo.size());

    Eigen::VectorXd angle_error = Eigen::VectorXd::Zero(n_ts);
    for (std::size_t i = 0; i < demo.size(); ++i) {
        const Eigen::Quaterniond delta = demo[i].y().inverse() * res[i].y();
        ASSERT_FALSE(delta.coeffs().hasNaN());
        const double w = std::abs(delta.w());
        angle_error(static_cast<Eigen::Index>(i)) =
                (w < 1.0) ? get_rotation_deg(w) : 0.0;
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
    const long n_ts     = std::ssize(se3_demo);
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

    // Weights are compared in single precision
    const auto as_float = [](const double x) -> float { return static_cast<float>(x); };
    for (long i = 0; i < se3_dmp.weights().rows(); ++i) {
        const auto& w_se3 = se3_dmp.weights();
        const auto& w_r3  = r3_dmp.weights();
        const auto& w_s3  = s3_dmp.weights();
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 0)), as_float(w_r3(i, 0)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 1)), as_float(w_r3(i, 1)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 2)), as_float(w_r3(i, 2)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 3)), as_float(w_s3(i, 0)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 4)), as_float(w_s3(i, 1)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 5)), as_float(w_s3(i, 2)));
        ASSERT_FLOAT_EQ(as_float(w_se3(i, 6)), as_float(w_s3(i, 3)));
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
}

TEST(Dmp, R2MeshDmpComparison) {
    using namespace std::chrono_literals;

    using Vec2     = Eigen::Vector2d;
    using Vec3     = Eigen::Vector3d;
    using R2       = mdv::riemann::Rn<2>;
    using MeshMan  = mdv::riemann::MeshManifold;
    using R2Demo   = mdv::Demonstration<R2>;
    using MeshDemo = mdv::Demonstration<MeshMan>;
    using R2Dmp    = mdv::Dmp<R2>;
    using MeshDmp  = mdv::Dmp<
            MeshMan,
            mdv::dmp::TransformationSystem<MeshMan>,
            mdv::dmp::ExponentialCoordinateSystem,
            mdv::DefaultFunction,
            mdv::riemann::MeshEmbedder>;

    const auto fun       = [](const double x, const double y) -> double { return 0.0; };
    const auto file_path = mdv::mesh::create_from_function(fun);
    const auto mesh_owner =
            std::make_shared<mesh::Mesh>(mesh::Mesh::from_file(file_path));
    const auto& mesh = *mesh_owner;
    auto        m    = std::make_shared<MeshMan>(mesh_owner);

    // TODO: fix numeric issue when using this initial conditions
    // const R2::Point y0{-0.5, -0.5};
    // const R2::Point g{0.5, 0.5};
    const R2::Point y0{-0.504, -0.534};
    const R2::Point g{0.52, 0.541};

    const std::vector<Vec2> r2_positions = mdv::build_r2_position(101, y0, g);
    ASSERT_TRUE(mdv::condition::are_equal(y0, r2_positions.front()));
    ASSERT_TRUE(mdv::condition::are_equal(g, r2_positions.back()));

    std::vector<MeshMan::Point> mesh_positions;
    mesh_positions.reserve(r2_positions.size());
    for (const Vec2& p : r2_positions) {
        mesh_positions.emplace_back(mesh::from_cartesian(mesh, {p(0), p(1), 0}));
    }


    const R2Demo   r2_demo   = R2Demo::builder(r2_positions.size())
                                       .set_sampling_period(5ms)
                                       .assign_position(r2_positions)
                                       .velocity_automatic_differentiation()
                                       .acceleration_automatic_differentiation()
                                       .create();
    const MeshDemo mesh_demo = MeshDemo::builder(m, mesh_positions.size())
                                       .set_sampling_period(5ms)
                                       .assign_position(mesh_positions)
                                       .velocity_automatic_differentiation()
                                       .acceleration_automatic_differentiation()
                                       .create();

    for (std::size_t i = 0; i < r2_demo.size(); ++i) {
        const Vec2 r2_p = r2_demo[i].y();
        const Vec3 r2pos_in_r3{r2_p(0), r2_p(1), 0.0};
        const Vec3 mesh_pos = mesh::position(mesh, mesh_demo[i].y());
        ASSERT_TRUE(mdv::condition::are_equal(r2pos_in_r3, mesh_pos));
    };

    R2Dmp   r2_dmp;
    auto    ts = std::make_shared<MeshDmp::TransformationSystem>(48.0, 12.0, m);
    MeshDmp mesh_dmp(
            12,
            ts,
            std::make_shared<MeshDmp::CoordinateSystem>(),
            get_default_logger(),
            m
    );
    mesh_dmp.embedding().setup(mesh_demo.front().y(), mesh_demo.back().y());
    r2_dmp.learn(r2_demo);
    mesh_dmp.learn(mesh_demo);
    const auto r2_res = r2_dmp.integrate(
            r2_demo.front().y(), r2_demo.back().y(), r2_demo.size(), r2_demo[1].t()
    );
    const auto mesh_res = mesh_dmp.integrate(
            mesh_demo.front().y(),
            mesh_demo.back().y(),
            mesh_demo.size(),
            mesh_demo[1].t()
    );

    for (std::size_t i = 0; i < r2_res.size(); ++i) {
        const Vec2 r2_p = r2_res[i].y();
        const Vec3 r2pos_in_r3{r2_p(0), r2_p(1), 0.0};
        const Vec3 mesh_pos = mesh::position(mesh, mesh_res[i].y());
        ASSERT_TRUE(mdv::condition::are_equal(r2pos_in_r3, mesh_pos));
    };
}

}  // namespace mdv
