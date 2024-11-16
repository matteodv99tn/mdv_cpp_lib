#ifndef MDV_LOGGING_EXTRAS_HPP
#define MDV_LOGGING_EXTRAS_HPP

#include <Eigen/Dense>

#include <mdv/utils/logging.hpp>

namespace mdv {

/**
 * @brief Creates a pretty-print version of an Eigen vector
 *
 */
std::string eigen_to_str(const Eigen::VectorXd& vec);

std::string array3d_to_str(const std::array<double, 3>& arr);


}  // namespace mdv


#endif  // MDV_LOGGING_EXTRAS_HPP
