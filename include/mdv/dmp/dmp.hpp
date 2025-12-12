#ifndef MDV_DMP_HPP
#define MDV_DMP_HPP

#include <gsl/assert>
#include <optional>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/coordinate_system/coordinate_system.hpp"
#include "mdv/dmp/learnable_function.hpp"
#include "mdv/dmp/transformation_system/transformation_system.hpp"
#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/se3.hpp"
#include "mdv/riemann_geometry/utils.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/conversions.hpp"
#include "mdv/utils/logging.hpp"

#define SQUARE(x) ((x) * (x))

namespace mdv {

template <typename T>
static T
cwise_dot(const T& x1, const T& x2) {
    return x1.cwiseProduct(x2);
}

template <>
double
cwise_dot<double>(const double& x1, const double& x2) {
    return x1 * x2;
}

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

    template <typename StateType, typename GoalType>
    Output
    embed_scale(const Input& in, const StateType& x, const GoalType& g) const {
        return embed(in, x, g);
    }

    template <typename StateType, typename GoalType>
    Output
    embed_scale(const Input& in, const StateType& x, const GoalType& g) const
        requires std::same_as<M, riemann::S3>
    {
        return Output::Ones();
    }

    template <typename StateType, typename GoalType>
    Output
    embed_scale(const Input& in, const StateType& x, const GoalType& g) const
        requires std::same_as<M, riemann::SE3>
    {
        // Output res = embed(in, x, g);
        // res.template tail<4>() = Eigen::Vector4d::Ones();
        // return res;
        return Output::Ones();
    }

private:
    TrivialEmbedder _impl;
    const M*        _m;
};

template <concepts::manifold M>
using DefaultTransformationSystem = dmp::TransformationSystem<M>;

template <int Dim>
using DefaultFunction = dmp::LearnableFunction<Dim, dmp::ExponentialBasis>;

template <
        concepts::manifold                 M,
        concepts::transformation_system<M> TS = DefaultTransformationSystem<M>,
        typename CS                           = dmp::ExponentialCoordinateSystem,
        template <int> typename Func          = DefaultFunction,
        concepts::embedding<M> E              = DefaultManifoldEmbedding<M>>
class Dmp {
public:
    // static_assert(concepts::embedding<E, M>);

    using Manifold             = M;
    using Embedding            = E;
    using TransformationSystem = TS;
    using CoordinateSystem     = CS;

    using EmbeddingPtr            = std::shared_ptr<Embedding>;
    using TransformationSystemPtr = std::shared_ptr<TransformationSystem>;
    using CoordinateSystemPtr     = std::shared_ptr<CoordinateSystem>;

    static constexpr long embedding_dimension =
            ::mdv::riemann::space_dimension_v<typename Embedding::Output>;

    using Function    = Func<embedding_dimension>;
    using FunctionPtr = std::unique_ptr<Function>;

    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    using MinimumGoalSample = TransformationSystem::MinimumGoalSample;
    using MinimumSample     = TransformationSystem::MinimumSample;

    using basis_size_t = unsigned long;
    using weights_t    = Eigen::MatrixXd;

    double tau;

    Dmp(long                          n_basis  = 12,
        const TransformationSystemPtr ts       = std::make_shared<TS>(),
        const CoordinateSystemPtr     cs       = std::make_shared<CS>(),
        Logger::SharedPtr             logger   = mdv::get_default_logger(),
        std::shared_ptr<Manifold>     manifold = std::make_shared<M>()) :
            _logger(std::move(logger)),
            _m(std::move(manifold)),
            _cs(std::move(cs)),
            _ts(std::move(ts)),
            tau(1.0),
            _emb(std::make_shared<Embedding>(_m.get())) {
        _logger->debug("Constructing parameters for a basis of size {}", n_basis);
        const Eigen::VectorXd s_places = Eigen::VectorXd::LinSpaced(n_basis, 0.0, 1.0);
        auto basis = dmp::ExponentialBasis::create_from_centers(s_places);
        _fun       = std::make_unique<Function>(std::move(basis));

        _logger->debug("Initialised DMP object");
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
        const auto initial = demo.front();
        const auto      goal = demo.back();

        for (long i = 0; i < demo.size(); ++i) {
            const TangentVector force = transf_sys().eval_forcing(demo[i], goal, tau, initial);

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

        logger().trace("Evaluating desired forcing term");
        const Eigen::MatrixXd f_des = evaluate_desired_forcing_term(demo);

        logger().trace("Evaluating matrix Phi");
        Eigen::MatrixXd phi(demo.size(), n_basis());
        for (auto i = 0; i < demo.size(); ++i)
            phi.row(i) =
                    fun().eval_basis(time_to_s(demo[i].t())) * time_to_s(demo[i].t());

        const auto scale = embedding().embed_scale(
                manifold().logarithmic_map(demo.front().y(), demo.back().y()),
                demo.front(),
                demo.back()
        );

        assert(phi.rows() == demo.size());
        assert(phi.cols() == n_basis());
        assert(f_des.rows() == demo.size());
        assert(f_des.cols() == embedding_dimension);
        fun().learn(phi, f_des, scale);
        logger().info("DMP succesfully learned");
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


        DemonstrationSample<M, 0> y0_sample;
        DemonstrationSample<M, 0> g_sample;
        y0_sample.y() = y0;
        g_sample.y()  = g;

        const auto scale = embedding().embed_scale(
                manifold().logarithmic_map(y0, g), y0_sample, g_sample
        );

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
            const typename Embedding::Output f    = cwise_dot(fun()(s, s), scale);
            const TangentVector              f_tv = embedding().decode(f, res[i], goal);
            transf_sys().step(res[i], goal, f_tv, tau, dts, res[i + 1], y0);
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

    // clang-format off
    MDV_NODISCARD std::size_t                 n_basis() const    { return fun().n_basis(); }
    MDV_NODISCARD const Function::WeightsMatrix& weights() const { return fun().weights(); }
    MDV_NODISCARD Function::WeightsMatrix&     weights() { return fun().weights(); }
    MDV_NODISCARD Logger&                     logger() const     { return *(_logger.get()); }
    MDV_NODISCARD Manifold&                   manifold()         { assert(_m); return *_m; }
    MDV_NODISCARD const Manifold&             manifold() const   { assert(_m); return *_m; }
    MDV_NODISCARD CoordinateSystem&           coord_sys()        { assert(_cs); return *_cs; }
    MDV_NODISCARD const CoordinateSystem&     coord_sys() const  { assert(_cs); return *_cs; }
    MDV_NODISCARD TransformationSystem&       transf_sys()       { assert(_ts); return *_ts; }
    MDV_NODISCARD const TransformationSystem& transf_sys() const { assert(_ts); return *_ts; }
    MDV_NODISCARD Function&                   fun()              { assert(_fun); return *_fun; }
    MDV_NODISCARD const Function&             fun() const        { assert(_fun); return *_fun; }
    MDV_NODISCARD Embedding&                  embedding()        { assert(_emb); return *_emb; }
    MDV_NODISCARD const Embedding&            embedding() const  { assert(_emb); return *_emb; }

    // clang-format on


private:
    // Transformation - Coordinate System
    std::shared_ptr<Manifold> _m;
    CoordinateSystemPtr       _cs;
    TransformationSystemPtr   _ts;
    FunctionPtr               _fun;
    EmbeddingPtr              _emb;

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

    std::optional<MinimumSample> initial_state;
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
        if (!initial_state.has_value())
            initial_state = curr_state;
        s                             = _dmp.coord_sys().step(s, _dmp.tau, dt);
        const auto scale = dmp().embedding().embed_scale(
                dmp().manifold().logarithmic_map(initial_state.value().y(), goal_state.y()),
                initial_state.value(),
                goal_state
        );
        const TangentVector         f = cwise_dot(_dmp.fun()(s, s), scale);
        typename Dmp::MinimumSample next_state;
        _dmp.transf_sys().step(curr_state, goal_state, f, _dmp.tau, dt, next_state, initial_state.value());
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
