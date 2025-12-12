#ifndef MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
#define MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

namespace mdv::dmp {


template <concepts::manifold M>
struct RiemannOdeSolver {
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    using TangentBundle = std::pair<Point, TangentVector>;

    RiemannOdeSolver(const double dt) : _dt(dt) {};

    TangentBundle
    operator()(const TangentBundle& xk, const TangentVector& f, const double tau) {
        const auto& [y, z]   = xk;
        TangentVector z_next = z + f * _dt / tau;
        const Point   y_next = M().exponential_map(y, z_next * _dt / tau);
        return {y_next, M().parallel_transport(y, y_next, z_next)};
    };


private:
    const double _dt;
};

template <concepts::manifold M>
class TransformationSystem {
public:
    using MinimumSample     = DemonstrationSample<M, 1>;
    using MinimumGoalSample = DemonstrationSample<M, 0>;

    using Manifold = M;
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    TransformationSystem(
            const double       alpha    = 48.0,
            const double       beta     = 12.0,
            std::shared_ptr<M> manifold = std::make_shared<M>()
    ) :
            _alpha(alpha), _beta(beta), _m(manifold) {}

    template <manifold_sample<1, M> Sample1, manifold_sample<0, M> Sample2, typename Sample3 = Sample2>
    MDV_NODISCARD TangentVector
    eval_forcing(
            const Sample1& curr_state,
            const Sample2& goal_state,
            const double                      tau,
            const Sample3& /* initial_state */ = {}
    ) const {
        const auto pos_err = manifold().logarithmic_map(curr_state.y(), goal_state.y());
        const auto& vel_err = curr_state.yd();
        const auto& acc_err = curr_state.ydd();
        return tau * tau * acc_err - _alpha * (_beta * pos_err - tau * vel_err);
    }

    template <manifold_sample<1, M> Sample1, manifold_sample<0, M> Sample2, typename Sample3 = Sample2>
    void
    step(const Sample1& curr_state,
         const Sample2& goal_state,
         const TangentVector&              force,
         const double                      tau,
         const double                      dt,
         Sample1&       next_state,
         const Sample3& initial_state = {}) const {
        const TangentVector log_y_g =
                manifold().logarithmic_map(curr_state.y(), goal_state.y());
        const TangentVector dz_dt_original =
                _alpha * (_beta * log_y_g - curr_state.yd()) + force;
        const TangentVector dz_dt =
                manifold().covariant_derivative(curr_state.y(), dz_dt_original);

        RiemannOdeSolver<M> solver(dt);
        const auto next = solver({curr_state.y(), curr_state.yd()}, dz_dt, tau);
        next_state.y()  = next.first;
        next_state.yd() = next.second;
    }

    // clang-format off
    MDV_NODISCARD double   alpha() const noexcept { return _alpha; }
    MDV_NODISCARD double   beta() const noexcept  { return _beta; }
    MDV_NODISCARD M&       manifold()             { assert(_m); return *_m; }
    MDV_NODISCARD const M& manifold() const       { assert(_m); return *_m; }

    // clang-format on

private:
    double             _alpha;
    double             _beta;
    std::shared_ptr<M> _m = nullptr;
};

}  // namespace mdv::dmp


#endif  // MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
