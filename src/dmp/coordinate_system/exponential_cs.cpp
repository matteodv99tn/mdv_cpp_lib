#include "mdv/dmp/coordinate_system/exponential_cs.hpp"

#include <cassert>
#include <cmath>

using mdv::dmp::ExponentialCoordinateSystem;

ExponentialCoordinateSystem::ExponentialCoordinateSystem(
        const double gamma, const double* tau
) :
        Base(tau), _gamma(gamma){};

double
ExponentialCoordinateSystem::eval(const double t) const {
    return std::exp(-_gamma / tau() * t);
}
