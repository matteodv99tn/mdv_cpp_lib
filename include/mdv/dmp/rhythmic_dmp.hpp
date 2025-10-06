#ifndef MDV_RHYTMIC_DMP_HPP
#define MDV_RHYTMIC_DMP_HPP

#include <cmath>
#include <numeric>
#include <optional>

#include "mdv/dmp/concepts.hpp"
#include "mdv/dmp/dmp.hpp"
#include "mdv/dmp/learnable_function.hpp"
#include "mdv/riemann_geometry/concepts.hpp"
#include "mdv/riemann_geometry/fwd.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/riemann_geometry/utils.hpp"

namespace mdv {

class PeriodicCoordinateSystem {
public:
    MDV_NODISCARD double
    eval_exact(const double t, const double tau = 1.0) const {
        assert(t >= 0.0);
        assert(tau > 0.0);
        return std::fmod(2.0 * M_PI * t / tau, 2.0 * M_PI);
    }

    /**
     * @brief Performs one discrete integration timestep
     */
    double
    step(const double current_state, const double tau, const double dt) const {
        return current_state * 2.0 * M_PI * dt / tau;
    }
};

template <concepts::manifold M, concepts::embedding<M> E = DefaultManifoldEmbedding<M>>
struct RhytmicDmp {
    using Manifold             = M;
    using Embedding            = E;
    using TransformationSystem = DefaultTransformationSystem<M>;
    using CoordinateSystem     = PeriodicCoordinateSystem;

    static constexpr long embedding_dimension =
            ::mdv::riemann::space_dimension_v<typename Embedding::Output>;
    using Function =
            dmp::LearnableFunction<embedding_dimension, dmp::PeriodicExponentialBasis>;

    using EmbeddingPtr            = std::shared_ptr<Embedding>;
    using TransformationSystemPtr = std::shared_ptr<TransformationSystem>;
    using FunctionPtr             = std::unique_ptr<Function>;

    MDV_MANIFOLD_TYPENAMES_IMPORT(M);
    using MinimumGoalSample = TransformationSystem::MinimumGoalSample;
    using MinimumSample     = TransformationSystem::MinimumSample;

    double tau;

    RhytmicDmp(
            long                          n_basis = 20,
            const TransformationSystemPtr ts = std::make_shared<TransformationSystem>(),
            Logger::SharedPtr             logger   = mdv::get_default_logger(),
            std::shared_ptr<Manifold>     manifold = std::make_shared<M>()
    ) :
            _logger(std::move(logger)),
            _m(std::move(manifold)),
            _ts(std::move(ts)),
            tau(1.0),
            _emb(std::make_shared<Embedding>(_m.get())) {
        _logger->debug("Constructing parameters for a basis of size {}", n_basis);
        auto basis = dmp::PeriodicExponentialBasis::create_equispaced(n_basis);
        _fun       = std::make_unique<Function>(std::move(basis));
        _logger->debug("Initialised RhytmicDMP object");
    }

    template <typename Demo>
    Point
    compute_average(const Demo& demo) const {
        const auto sum = [](const double sum, const auto& s) -> double {
            return sum + s.y();
        };
        return Point{std::accumulate(demo.begin(), demo.end(), 0.0, sum) / demo.size()};
    }

    template <typename Demo>
    Point
    compute_average(const Demo& demo) const requires std::same_as<Manifold, riemann::MeshManifold> {
        return demo.front().y();
    }

    template <typename Demonstration>
    MDV_NODISCARD Eigen::MatrixXd
    evaluate_desired_forcing_term(const Demonstration& demo, const Point& goal) {
        using mdv::convert::seconds;
        static constexpr bool is_scalar = embedding_dimension == 1;

        Eigen::MatrixXd f_des(demo.size(), embedding_dimension);

        MinimumGoalSample goal_sample;
        goal_sample.y() = goal;
        for (long i = 0; i < demo.size(); ++i) {
            const TangentVector force = ts().eval_forcing(demo[i], goal_sample, tau);

            if constexpr (is_scalar)
                f_des(i) = emb().embed(force, demo[i], goal_sample);
            else f_des.row(i) = emb().embed(force, demo[i], goal_sample);
        }
        assert(!f_des.hasNaN());
        return f_des;
    }

    double
    tv_norm(const TangentVector& tv) const {
        if constexpr (riemann::space_dimension_v<TangentVector> == 1) {
            return std::abs(tv);
        } else {
            return tv.norm();
        }
    }

    double
    construct_default_r(const Point& y0, const Point& g) const {
        const TangentVector log = m().logarithmic_map(y0, g);
        return tv_norm(log);
    }

    template <typename Demonstration>
    double
    construct_default_r(
            const Demonstration& demo, const std::optional<Point> goal = std::nullopt
    ) const {
        const Point g = goal.value_or(compute_average(demo));
        const auto  compute_distance =
                [this, g](const Demonstration::Sample& sample) -> double {
            return tv_norm(m().logarithmic_map(sample.y(), g));
        };

        return std::transform_reduce(
                demo.begin(),
                demo.end(),
                -std::numeric_limits<double>::infinity(),
                std::less<double>{},
                compute_distance
        );
    }

    template <typename Demonstration>
    void
    learn(const Demonstration&       demo,
          const std::optional<double> r_value = std::nullopt,
          const std::optional<Point> goal    = std::nullopt) {
        using mdv::convert::seconds;
        tau = seconds(demo.back().t() - demo.front().t());

        logger().info("Training DMP on a demonstration with {} samples", demo.size());

        logger().trace("Evaluating desired forcing term");
        const Point           g     = goal.value_or(compute_average(demo));
        assert(g != demo.front().y());
        const Eigen::MatrixXd f_des = evaluate_desired_forcing_term(demo, g);

        const double r = r_value.value_or(construct_default_r(demo));

        logger().trace("Evaluating matrix Phi");
        Eigen::MatrixXd phi(demo.size(), n_basis());
        for (auto i = 0; i < demo.size(); ++i) {
            const double t_secs = time_to_s(demo[i].t());
            phi.row(i)          = fun().eval_basis(t_secs) * r;
        }

        assert(phi.rows() == demo.size());
        assert(phi.cols() == n_basis());
        assert(f_des.rows() == demo.size());
        assert(f_des.cols() == embedding_dimension);
        fun().learn(phi, f_des);
        logger().info("DMP succesfully learned");
    }

    Demonstration<M>
    integrate(
            const Point&                 y0,
            const TangentVector&         v0,
            const Point&                 g,
            const double                 r,
            std::size_t                  n_steps,
            const Demonstration<M>::Time dt
    ) const {
        using mdv::condition::are_orthogonal;
        using mdv::convert::seconds;
        logger().info("Performing integration");

        Demonstration<M> res =
                Demonstration<M>::builder(n_steps).set_sampling_period(dt).create();
        res.front().y()   = y0;
        res.front().yd()  = v0;
        res.front().ydd() = manifold().default_tangent_vector();
        typename Demonstration<M>::Sample goal;
        goal.y() = g;

        const auto dts = seconds(dt);
        for (auto i = 0; i < n_steps - 1; ++i) {
            const double                     s    = time_to_s(i * dt);
            const typename Embedding::Output f    = fun()(s, r);
            const TangentVector              f_tv = emb().decode(f, res[i], goal);
            ts().step(res[i], goal, f_tv, tau, dts, res[i + 1]);
        }
        return res;
    }

    MDV_NODISCARD double
    time_to_s(const double t) const {
        return cs().eval_exact(t, tau);
    }

    template <typename Duration>
    MDV_NODISCARD double
    time_to_s(const Duration t) const {
        return time_to_s(mdv::convert::seconds(t));
    }

    // clang-format off
    MDV_NODISCARD std::size_t                 n_basis() const    { return fun().n_basis(); }
    MDV_NODISCARD const Function::WeightsMatrix& weights() const { return fun().weights(); }
    MDV_NODISCARD Logger&                     logger() const     { return *(_logger.get()); }
    MDV_NODISCARD Manifold&                   manifold()         { assert(_m); return *_m; }
    MDV_NODISCARD const Manifold&             manifold() const   { assert(_m); return *_m; }
    MDV_NODISCARD CoordinateSystem            coord_sys() const  { return CoordinateSystem{}; }
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
    TransformationSystemPtr   _ts;
    FunctionPtr               _fun;
    EmbeddingPtr              _emb;

    mutable Logger::SharedPtr _logger;

    //
    // clang-format off
    MDV_NODISCARD Manifold&                   m()          { assert(_m); return *_m; }
    MDV_NODISCARD const Manifold&             m() const    { assert(_m); return *_m; }
    MDV_NODISCARD CoordinateSystem            cs() const   { return CoordinateSystem{}; }
    MDV_NODISCARD TransformationSystem&       ts()         { assert(_ts); return *_ts; }
    MDV_NODISCARD const TransformationSystem& ts() const   { assert(_ts); return *_ts; }
    MDV_NODISCARD Function&                   f()          { assert(_fun); return *_fun; }
    MDV_NODISCARD const Function&             f() const    { assert(_fun); return *_fun; }
    MDV_NODISCARD Embedding&                  emb()        { assert(_emb); return *_emb; }
    MDV_NODISCARD const Embedding&            emb() const  { assert(_emb); return *_emb; }

    // clang-format on
};

}  // namespace mdv


#endif  // MDV_RHYTMIC_DMP_HPP
