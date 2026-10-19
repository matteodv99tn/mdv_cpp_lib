#include "mdv/utils/eigen_solvers.hpp"

#include <Eigen/QR>

namespace mdv {

Eigen::Quaterniond
quaternion_from_two_vectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return Eigen::Quaterniond::FromTwoVectors(a, b);
}

Eigen::MatrixXd
fullpiv_qr_solve(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    return a.fullPivHouseholderQr().solve(b);
}

}  // namespace mdv
