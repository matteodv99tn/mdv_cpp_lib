#ifndef MDV_UTILS_EIGEN_SOLVERS_HPP
#define MDV_UTILS_EIGEN_SOLVERS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mdv {

// Keep the decomposition implementations out of callers' translation units.
Eigen::Quaterniond quaternion_from_two_vectors(
        const Eigen::Vector3d& a, const Eigen::Vector3d& b
);
Eigen::MatrixXd fullpiv_qr_solve(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);

}  // namespace mdv

#endif  // MDV_UTILS_EIGEN_SOLVERS_HPP
