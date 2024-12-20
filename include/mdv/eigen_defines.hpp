#ifndef MDB_EIGEN_HPP
#define MDB_EIGEN_HPP

#include <Eigen/Dense>

namespace mdv {

// NOLINTBEGIN magic numbers
using Vec2d = Eigen::Vector<double, 2>;
using Vec3d = Eigen::Vector<double, 3>;
using Vec4d = Eigen::Vector<double, 4>;
using Vec5d = Eigen::Vector<double, 5>;
using Vec6d = Eigen::Vector<double, 6>;
using Vec7d = Eigen::Vector<double, 7>;

using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat7d = Eigen::Matrix<double, 7, 7>;
// NOLINTEND

}  // namespace mdv


#endif  // MDB_EIGEN_HPP
