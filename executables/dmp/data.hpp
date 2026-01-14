#ifndef EXPONENTIALMAP_DATA_HPP
#define EXPONENTIALMAP_DATA_HPP

#include <Eigen/Dense>
#include <vector>

std::vector<Eigen::Vector3d> get_demonstration_position();
std::vector<Eigen::Vector3d> get_demonstration_velocity();
std::vector<Eigen::Vector3d> get_demonstration_acceleration();
Eigen::Vector3d              get_demonstration_centre();
std::vector<Eigen::Vector3d> get_integrated_position();
std::vector<Eigen::Vector3d> get_integrated_velocity();
Eigen::Vector3d              get_bunny_centre();


#endif  // EXPONENTIALMAP_DATA_HPP
