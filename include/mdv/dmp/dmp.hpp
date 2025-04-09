#ifndef MDV_DMP_HPP
#define MDV_DMP_HPP

#include <cmath>
#include <gsl/assert>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/dmp/learnable_function.hpp"
#include "mdv/dmp/transformation_system/transformation_system.hpp"
#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/utils/conversions.hpp"
#include "mdv/utils/logging.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv {

template <
        riemann::manifold        M,
        transformation_system<M> TransfSystem = dmp::TransformationSystem<M>,
        typename CoordSystem                  = dmp::ExponentialCoordinateSystem>
class Dmp {
public:
    using Manifold = M;
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);
    using Function = dmp::LearnableFunction<Manifold>;

    using MinimumGoalSample = TransfSystem::MinimumGoalSample;
    using MinimumSample     = TransfSystem::MinimumSample;

    using basis_size_t = unsigned long;
    using weights_t    = Eigen::MatrixXd;

    double tau;

    Dmp(const double       alpha   = 48.0,
        const double       beta    = 12.0,
        const double       gamma   = 3.0,
        const basis_size_t n_basis = 12) :
            _cs(gamma), _ts(alpha, beta), _fun(n_basis) {
        construct_basis_parameters();

        logger().info("Initialised DMP object");
        logger().debug("  alpha = {}", _ts.alpha());
        logger().debug("  beta  = {}", _ts.beta());
        logger().debug("  gamma = {}", _cs.gamma());
        logger().debug("  number of basis: {}", this->n_basis());
    }

    Dmp(const Dmp&)            = default;
    Dmp& operator=(const Dmp&) = default;
    Dmp(Dmp&&)                 = default;
    Dmp& operator=(Dmp&&)      = default;

    template <typename Demonstration>
    MDV_NODISCARD Eigen::MatrixXd
                  evaluate_desired_forcing_term(const Demonstration& demo) {
        using mdv::convert::seconds;
        static constexpr bool is_scalar = Function::tan_vec_dim == 1;

        Eigen::MatrixXd f_des(demo.size(), Function::tan_vec_dim);
        const auto      goal = demo.back();

        for (long i = 0; i < demo.size(); ++i) {
            const auto force = _ts.eval_forcing(demo[i], goal, tau);

            if constexpr (is_scalar) f_des(i) = force;
            else f_des.row(i) = Function::to_eigen(force);
        }
        assert(!f_des.hasNaN());
        return f_des;
    }

    template <typename Demonstration>
    void
    learn(const Demonstration& demo) {
        using mdv::convert::seconds;
        tau = seconds(demo.back().t() - demo.front().t());

        logger().info("Training DMP on a demonstration with {} samples", demo.size());
#ifdef MDV_VERBOSE_DMP
        logger().info("  tau = {}s", tau);
        logger().info("  dt  = {}s", mdv::convert::seconds(demo[1].t()));
        logger().info("  y0  = {}", demo.front().y());
        logger().info("  g   = {}", demo.back().y());
#endif

        logger().trace("Evaluating desired forcing term");
        const Eigen::MatrixXd f_des = evaluate_desired_forcing_term(demo);

        logger().trace("Evaluating matrix Phi");
        Eigen::MatrixXd phi(demo.size(), n_basis());
        for (auto i = 0; i < demo.size(); ++i)
            phi.row(i) = eval_basis(time_to_s(demo[i].t())) * time_to_s(demo[i].t());

        assert(phi.rows() == demo.size());
        assert(phi.cols() == n_basis());
        assert(f_des.rows() == demo.size());
        assert(f_des.cols() == Function::tan_vec_dim);
        _fun.learn(phi, f_des);
        logger().info("DMP succesfully learned");


        const Eigen::MatrixXd ae = (phi * _fun.weights() - f_des).cwiseAbs();
        logger().info("Linear regression mean absolute error: {}", ae.mean());
        logger().info("Linear regression maximum absolute error: {}", ae.maxCoeff());

#ifdef MDV_VERBOSE_DMP
        logger().info("Integrating learned dynamics to compare learning outcome...");
        const auto reconstructed_demo =
                integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());

        Eigen::MatrixXd pos_err(f_des.rows(), f_des.cols());
        Eigen::VectorXd pos_err_norm(f_des.rows());
        for (auto i = 0; i < demo.size(); ++i) {
            if constexpr (std::is_scalar_v<Point>) {
                pos_err(i, 0) =
                        M::logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
                // pos_err_norm(i) = std::abs(pos_err(i, 0));
            } else {
                pos_err.row(i) =
                        M::logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
            }
            pos_err_norm(i) = pos_err.row(i).norm();
        }
        logger().info("Position error (norm in the tangent space):");
        logger().info("  mean: {}", double(pos_err_norm.mean()));
        logger().info("  min:  {}", double(pos_err_norm.minCoeff()));
        logger().info("  max:  {}", double(pos_err_norm.maxCoeff()));
#endif
    }

    Demonstration<M>
    integrate(
            const Point&                 y0,
            const Point&                 g,
            std::size_t                  n_steps,
            const Demonstration<M>::Time dt
    ) const {
        using mdv::condition::are_orthogonal;
        using mdv::convert::seconds;
        logger().info("Performing integration");
#ifdef MDV_VERBOSE_DMP
        logger().info("  y0: {}", y0);
        logger().info("  g:  {}", g);
        logger().info("  dt: {}s", mdv::convert::seconds(dt));
        logger().info("  number of steps: {}", n_steps);
#endif

        Demonstration<M> res = Demonstration<M>::builder(n_steps).create();
        res.front().y()      = y0;
        res.front().yd()     = M::default_tangent_vector();
        res.front().ydd()    = M::default_tangent_vector();
        typename Demonstration<M>::Sample goal;
        goal.y() = g;

        const auto dts = seconds(dt);
        for (auto i = 0; i < n_steps - 1; ++i) {
            const double        s = time_to_s(i * dt);
            const TangentVector f = _fun.eval(s, s);
            _ts.step(res[i], goal, f, tau, dts, res[i + 1]);
        }
        return res;
    }

    MDV_NODISCARD double
    time_to_s(const double t) const {
        return _cs.eval_exact(t, tau);
    }

    template <typename Duration>
    MDV_NODISCARD double
    time_to_s(const Duration t) const {
        return time_to_s(mdv::convert::seconds(t));
    }

    Eigen::VectorXd
    eval_basis(const double s) const {
        return _fun.eval_basis(s);
    }

    // clang-format off
    MDV_NODISCARD std::size_t            n_basis() const { return _fun.n_basis(); }
    MDV_NODISCARD const Eigen::MatrixXd& weights() const { return _fun.weights(); }
    MDV_NODISCARD Logger&                logger() const  { return *(_logger.get()); }
    MDV_NODISCARD CoordSystem&           coord_sys()     { return _cs; }
    MDV_NODISCARD TransfSystem&          transf_sys()    { return _ts; }
    MDV_NODISCARD Function&              fun()           { return _fun; }

    // clang-format on


private:
    // Transformation - Coordinate System
    CoordSystem  _cs;
    TransfSystem _ts;

    // Basis
    Function _fun;

    void
    construct_basis_parameters() {
        logger().debug("Constructing parameters for a basis of size {}", n_basis());
        Eigen::VectorXd cs = Eigen::VectorXd(n_basis());
        Eigen::VectorXd hs = Eigen::VectorXd(n_basis());

        for (auto i = 0; i < n_basis(); ++i) {
            cs(i) = _cs.eval_exact(double(i) / double(n_basis()));
            logger().trace("c[{}] = {}", i, cs(i));
        }

        for (auto i = 0; i < n_basis() - 1; ++i) hs(i) = 1 / SQUARE(cs(i + 1) - cs(i));
        hs(n_basis() - 1) = hs(n_basis() - 2);
        for (auto i = 0; i < n_basis(); ++i) logger().trace("h[{}] = {}", i, hs(i));

        _fun.assign_centers(std::move(cs));
        _fun.assign_widths(std::move(hs));
    }

    mutable LoggerPtr _logger =
            mdv::static_logger_factory("dmp", Logger::LogLevel::Debug);
};

#undef SQUARE

template <typename Dmp>
class IntegrableDmp {
public:
    static constexpr double min_tau = 0.1;

    using MinimumSample     = typename Dmp::MinimumSample;
    using MinimumGoalSample = typename Dmp::MinimumGoalSample;
    using Point             = typename Dmp::Manifold::Point;
    using TangentVector     = typename Dmp::Manifold::TangentVector;

    IntegrableDmp(Dmp&& dmp) : _dmp(std::move(dmp)) {}

    MinimumSample     curr_state;
    MinimumGoalSample goal_state;
    double            s  = 1.0;
    double            dt = 1e-3;  // NOLINT 1ms

    void
    update_dmp(Dmp&& dmp) {
        _dmp = std::move(dmp);
        if (_dmp.tau <= min_tau) {
            _dmp.logger().debug(
                    "Current dmp object has tau = {}, setting to default {}",
                    _dmp.tau,
                    min_tau
            );
            _dmp.tau = min_tau;
        }
    }

    void
    step() {
        s                             = _dmp.coord_sys().step(s, _dmp.tau, dt);
        const TangentVector         f = _dmp.fun().eval(s, s);
        typename Dmp::MinimumSample next_state;
        _dmp.transf_sys().step(curr_state, goal_state, f, _dmp.tau, dt, next_state);
        curr_state = next_state;
    }

    template <typename T>
    void
    set_sampling_period(const T& dt) {
        dt = mdv::convert::seconds(dt);
        _dmp.logger().debug("Dmp integration sampling period set to {}ms", dt * 1000);
    }

    // clang-format off
    MDV_NODISCARD Dmp&       dmp()       { return _dmp; }
    MDV_NODISCARD const Dmp& dmp() const { return _dmp; }

    // clang-format on

private:
    Dmp _dmp;
};
}  // namespace mdv

#endif  // MDV_DMP_HPP
