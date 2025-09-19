#ifndef MDV_DMP_HPP
#define MDV_DMP_HPP

#include <gsl/assert>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/dmp/learnable_function.hpp"
#include "mdv/dmp/transformation_system/transformation_system.hpp"
#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/riemann_geometry/se3.hpp"
#include "mdv/riemann_geometry/utils.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/conversions.hpp"
#include "mdv/utils/logging.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv {

template <concepts::trivially_embeddable_manifold M>
struct DefaultManifoldEmbedding {
    using TrivialEmbedder = riemann::TrivialTypeEmbedding<typename M::TangentVector>;
    using Input           = TrivialEmbedder::Input;
    using Output          = TrivialEmbedder::Output;

    DefaultManifoldEmbedding(const M* manifold) : _m(manifold) {};

    template <typename StateType, typename GoalType>
    Output
    embed(const Input& in, const StateType& x, const GoalType& g) const {
        return _impl.embed(in);
    }

    template <typename StateType, typename GoalType>
    Input
    decode(const Output& out, const StateType& x, const GoalType& g) const {
        return _impl.decode(out);
    }

private:
    TrivialEmbedder _impl;
    const M*        _m;
};

template <
        concepts::manifold                 M,
        concepts::transformation_system<M> TS = dmp::TransformationSystem<M>,
        typename CoordSystem                  = dmp::ExponentialCoordinateSystem,
        concepts::embedding<M> E              = DefaultManifoldEmbedding<M>>
class Dmp {
public:
    // static_assert(concepts::embedding<E, M>);

    using Embedding            = E;
    using Manifold             = M;
    using TransformationSystem = TS;
    using TransfSystem         = TS;
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    static constexpr long embedding_dimension =
            ::mdv::riemann::space_dimension_v<typename Embedding::Output>;
    using Function = dmp::LearnableFunction<embedding_dimension>;

    using MinimumGoalSample = TransformationSystem::MinimumGoalSample;
    using MinimumSample     = TransformationSystem::MinimumSample;

    using basis_size_t = unsigned long;
    using weights_t    = Eigen::MatrixXd;

    double tau;

    Dmp(Logger::SharedPtr         logger   = mdv::get_default_logger(),
        const double              alpha    = 48.0,
        const double              beta     = 12.0,
        const double              gamma    = 3.0,
        const basis_size_t        n_basis  = 12,
        std::shared_ptr<Manifold> manifold = std::make_shared<Manifold>()) :
            _logger(logger),
            _m(std::move(manifold)),
            _cs(std::make_shared<dmp::ExponentialCoordinateSystem>(gamma)),
            _ts(std::make_shared<TransformationSystem>(alpha, beta)),
            tau(1.0),
            _emb(_m.get()) {
        initialise_function(n_basis);

        logger->debug("Initialised DMP object");
#ifdef MDV_VERBOSE_DMP
        logger->debug("  alpha = {}", _ts.alpha());
        logger->debug("  beta  = {}", _ts.beta());
        logger->debug("  gamma = {}", _cs.gamma());
        logger->debug("  number of basis: {}", this->n_basis());
#endif
    }

    Dmp(const Dmp&)            = default;
    Dmp& operator=(const Dmp&) = default;
    Dmp(Dmp&&)                 = default;
    Dmp& operator=(Dmp&&)      = default;

    template <typename Demonstration>
    MDV_NODISCARD Eigen::MatrixXd
                  evaluate_desired_forcing_term(const Demonstration& demo) {
        using mdv::convert::seconds;
        static constexpr bool is_scalar = embedding_dimension == 1;

        Eigen::MatrixXd f_des(demo.size(), embedding_dimension);
        const auto      goal = demo.back();

        for (long i = 0; i < demo.size(); ++i) {
            const TangentVector force = transf_sys().eval_forcing(demo[i], goal, tau);

            if constexpr (is_scalar) f_des(i) = force;
            else f_des.row(i) = embedding().embed(force, demo[i], goal);
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
        assert(f_des.cols() == embedding_dimension);
        fun().learn(phi, f_des);
        logger().info("DMP succesfully learned");


#ifdef MDV_VERBOSE_DMP
        const Eigen::MatrixXd ae = (phi * _fun.weights() - f_des).cwiseAbs();
        logger().info("Linear regression mean absolute error: {}", ae.mean());
        logger().info("Linear regression maximum absolute error: {}", ae.maxCoeff());

        logger().info("Integrating learned dynamics to compare learning outcome...");
        const auto reconstructed_demo =
                integrate(demo.front().y(), demo.back().y(), demo.size(), demo[1].t());

        Eigen::MatrixXd pos_err(f_des.rows(), f_des.cols());
        Eigen::VectorXd pos_err_norm(f_des.rows());
        for (auto i = 0; i < demo.size(); ++i) {
            if constexpr (std::is_scalar_v<Point>) {
                pos_err(i, 0) =
                        _m.logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
                // pos_err_norm(i) = std::abs(pos_err(i, 0));
            } else {
                pos_err.row(i) =
                        _m.logarithmic_map(demo[i].y(), reconstructed_demo[i].y());
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

        Demonstration<M> res =
                Demonstration<M>::builder(n_steps).set_sampling_period(dt).create();
        res.front().y()   = y0;
        res.front().yd()  = manifold().default_tangent_vector();
        res.front().ydd() = manifold().default_tangent_vector();
        typename Demonstration<M>::Sample goal;
        goal.y() = g;

        const auto dts = seconds(dt);
        for (auto i = 0; i < n_steps - 1; ++i) {
            const double                     s    = time_to_s(i * dt);
            const typename Embedding::Output f    = fun()(s, s);
            const TangentVector              f_tv = embedding().decode(f, res[i], goal);
            transf_sys().step(res[i], goal, f_tv, tau, dts, res[i + 1]);
        }
        return res;
    }

    MDV_NODISCARD double
    time_to_s(const double t) const {
        return coord_sys().eval_exact(t, tau);
    }

    template <typename Duration>
    MDV_NODISCARD double
    time_to_s(const Duration t) const {
        return time_to_s(mdv::convert::seconds(t));
    }

    Eigen::VectorXd
    eval_basis(const double s) const {
        return fun().eval_basis(s);
    }

    // clang-format off
    MDV_NODISCARD std::size_t            n_basis() const    { return fun().n_basis(); }
    MDV_NODISCARD const Function::WeightsMatrix& weights() const { return fun().weights(); }
    MDV_NODISCARD Logger&                logger() const     { return *(_logger.get()); }
    MDV_NODISCARD Manifold&              manifold()         { assert(_m); return *_m; }
    MDV_NODISCARD const Manifold&        manifold() const   { assert(_m); return *_m; }
    MDV_NODISCARD CoordSystem&           coord_sys()        { assert(_cs); return *_cs; }
    MDV_NODISCARD const CoordSystem&     coord_sys() const  { assert(_cs); return *_cs; }
    MDV_NODISCARD TransfSystem&          transf_sys()       { assert(_ts); return *_ts; }
    MDV_NODISCARD const TransfSystem&    transf_sys() const { assert(_ts); return *_ts; }
    MDV_NODISCARD Function&              fun()              { assert(_fun); return *_fun; }
    MDV_NODISCARD const Function&        fun() const        { assert(_fun); return *_fun; }
    MDV_NODISCARD Embedding&             embedding()        { return _emb; }
    MDV_NODISCARD const Embedding&       embedding() const  { return _emb; }

    // clang-format on


private:
    // Transformation - Coordinate System
    std::shared_ptr<Manifold>     _m;
    std::shared_ptr<CoordSystem>  _cs;
    std::shared_ptr<TransfSystem> _ts;
    Embedding                     _emb;

    // Basis
    std::unique_ptr<Function> _fun;

    void
    initialise_function(long n_basis) {
        logger().debug("Constructing parameters for a basis of size {}", n_basis);
        Eigen::VectorXd cs = Eigen::VectorXd(n_basis);
        Eigen::VectorXd hs = Eigen::VectorXd(n_basis);

        for (auto i = 0; i < n_basis; ++i)
            cs(i) = coord_sys().eval_exact(double(i) / double(n_basis));

        for (auto i = 0; i < n_basis - 1; ++i) hs(i) = 1 / SQUARE(cs(i + 1) - cs(i));
        hs(n_basis - 1) = hs(n_basis - 2);

        std::vector<dmp::ExponentialBasis> basis;
        basis.reserve(n_basis);
        for (long i = 0; i < n_basis; ++i) basis.emplace_back(cs(i), hs(i));

        _fun = std::make_unique<Function>(std::move(basis));
    }

    mutable Logger::SharedPtr _logger;
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

    template <typename... Args>
    IntegrableDmp(Args... args) : _dmp(std::forward<Args>(args)...) {}

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
        const TangentVector         f = _dmp.fun()(s, s);
        typename Dmp::MinimumSample next_state;
        _dmp.transf_sys().step(curr_state, goal_state, f, _dmp.tau, dt, next_state);
        curr_state = next_state;
    }

    template <typename T>
    void
    set_sampling_period(const T& integration_dt) {
        dt = mdv::convert::seconds(integration_dt);
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
