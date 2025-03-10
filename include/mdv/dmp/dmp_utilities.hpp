#ifndef MDV_DMP_UTILITIES_HPP
#define MDV_DMP_UTILITIES_HPP


#include "mdv/containers/demonstration.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/scalar.hpp"

namespace mdv {

Demonstration<riemann::Scalar> build_scalar_demonstration(long n_samples = 1001);
Demonstration<riemann::Scalar> build_exact_scalar_demonstration();
Demonstration<riemann::S3>     build_quaternion_demonstration(long n_samples = 1001);
std::vector<double>            poly_5th(const Eigen::VectorXd& ts);

}  // namespace mdv


#endif  // MDV_DMP_UTILITIES_HPP
