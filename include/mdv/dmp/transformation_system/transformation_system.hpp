#ifndef MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
#define MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/concepts.hpp"
#include "mdv/macros.hpp"
#include "mdv/riemann_geometry/manifold.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

namespace mdv::dmp {


template <riemann::manifold M>
class TransformationSystem {
public:
    using MinimumSample     = DemonstrationSample<M, 1>;
    using MinimumGoalSample = DemonstrationSample<M, 0>;

    using Manifold = M;
    MDV_MANIFOLD_TYPENAMES_IMPORT(M);

    TransformationSystem(const double alpha = 48.0, const double beta = 12.0) :
            _alpha(alpha), _beta(beta) {}

    MDV_NODISCARD TangentVector
    eval_forcing(
            const manifold_sample<1, M> auto& curr_state,
            const manifold_sample<0, M> auto& goal_state,
            const double                      tau
    ) const {
        const auto  pos_err = M::logarithmic_map(curr_state.y(), goal_state.y());
        const auto& vel_err = curr_state.yd();
        const auto& acc_err = curr_state.ydd();
        return tau * tau * acc_err - _alpha * (_beta * pos_err - tau * vel_err);
    }

    void
    step(const manifold_sample<1, M> auto& curr_state,
         const manifold_sample<0, M> auto& goal_state,
         const TangentVector&              force,
         const double                      tau,
         const double                      dt,
         manifold_sample<1, M> auto&       next_state) const {
        const TangentVector log_y_g =
                M::logarithmic_map(curr_state.y(), goal_state.y());
        const TangentVector dz_dt_original =
                _alpha * (_beta * log_y_g - curr_state.yd()) + force;
        const TangentVector dz_dt =
                M::covariant_derivative(curr_state.y(), dz_dt_original);
        const TangentVector z_next = curr_state.yd() + dz_dt * dt / tau;
        next_state.y() = M::exponential_map(curr_state.y(), curr_state.yd() * dt / tau);
        next_state.yd() = M::parallel_transport(curr_state.y(), next_state.y(), z_next);
    }

    // clang-format off
    MDV_NODISCARD double alpha() const noexcept { return _alpha; }
    MDV_NODISCARD double beta() const noexcept  { return _beta; }

    // clang-format on

private:
    double _alpha;
    double _beta;
};

}  // namespace mdv::dmp


#endif  // MDV_DMP_TRANSFORMATION_SYSTEM_INTERFACE_HPP
