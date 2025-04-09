#ifndef MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP
#define MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP

#include <math.h>

#include "mdv/macros.hpp"

namespace mdv::dmp {

class ExponentialCoordinateSystem {
public:
    ExponentialCoordinateSystem(const double gamma = 3.0) : _gamma(gamma) {};

    MDV_NODISCARD double
    eval_exact(const double t, const double tau = 1.0) const {
        return std::exp(-_gamma * t / tau);
    }

    /**
     * @brief Performs one discrete integration timestep
     */
    double
    step(const double current_state, const double tau, const double dt) const {
        return current_state * (1 - _gamma / tau * dt);
    }

    // clang-format off
    MDV_NODISCARD double gamma() const { return _gamma; }

    // clang-format on

private:
    double _gamma;
};


}  // namespace mdv::dmp


#endif  // MDV_DMP_COORDINATE_SYSTEM_INTERFACE_HPP
