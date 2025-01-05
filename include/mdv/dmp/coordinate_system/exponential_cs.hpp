#ifndef MDV_DMP_EXPONENTIAL_COORDINATE_SYSTEM_HPP
#define MDV_DMP_EXPONENTIAL_COORDINATE_SYSTEM_HPP

#include <mdv/dmp/coordinate_system/coordinate_system.hpp>
#include <mdv/macros.hpp>

namespace mdv::dmp {

class ExponentialCoordinateSystem : CoordinateSystemBase<ExponentialCoordinateSystem> {
public:
    using Base = CoordinateSystemBase<ExponentialCoordinateSystem>;

    ExponentialCoordinateSystem(double gamma, const double* tau);

    MDV_NODISCARD double eval(double t) const;

private:
    double  _gamma = 0;
};

}  // namespace mdv::dmp


#endif  // MDV_DMP_EXPONENTIAL_COORDINATE_SYSTEM_HPP
