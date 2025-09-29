#ifndef MDV_DMP_UTILITIES_HPP
#define MDV_DMP_UTILITIES_HPP


#include "mdv/containers/demonstration.hpp"
#include "mdv/riemann_geometry/euclidean.hpp"
#include "mdv/riemann_geometry/s3.hpp"
#include "mdv/riemann_geometry/scalar.hpp"
#include "mdv/riemann_geometry/se3.hpp"

namespace mdv {

Demonstration<riemann::Scalar>     build_scalar_demonstration(long n_samples = 1001);
Demonstration<riemann::Scalar>     build_exact_scalar_demonstration();
Demonstration<riemann::Rn<3>>      build_position_demonstration(long n_samples = 1001);
std::vector<riemann::Rn<2>::Point> build_r2_position(
        long            n_samples = 1001,
        Eigen::Vector2d y0        = {1.0, 1.0},
        Eigen::Vector2d g         = {0.0, 0.0}
);
Demonstration<riemann::Rn<2>> build_r2_demonstration(
        long            n_samples = 1001,
        Eigen::Vector2d y0        = {1.0, 1.0},
        Eigen::Vector2d g         = {0.0, 0.0}
);
Demonstration<riemann::SE3> build_se3_demonstration(long n_samples = 1001);
Demonstration<riemann::S3>  build_quaternion_demonstration(long n_samples = 1001);
std::vector<double>         poly_5th(const Eigen::VectorXd& ts);

}  // namespace mdv


#endif  // MDV_DMP_UTILITIES_HPP
