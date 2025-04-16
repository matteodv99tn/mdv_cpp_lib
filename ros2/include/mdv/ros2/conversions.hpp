#ifndef MDV_ROS2_CONVERSIONS_HPP
#define MDV_ROS2_CONVERSIONS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "mdv/ros2/se3.hpp"

namespace mdv::ros2 {

// Pose-related messages
Eigen::Vector3d    get_position(const geometry_msgs::msg::Pose&);
Eigen::Vector3d    get_position(const geometry_msgs::msg::PoseStamped&);
Eigen::Quaterniond get_orientation(const geometry_msgs::msg::Pose&);
Eigen::Quaterniond get_orientation(const geometry_msgs::msg::PoseStamped&);
SE3                get_pose(const geometry_msgs::msg::Pose&);
SE3Framed          get_pose(const geometry_msgs::msg::PoseStamped&);

geometry_msgs::msg::Pose to_pose_message(
        const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation
);
geometry_msgs::msg::Pose        to_pose_message(const SE3& pose);
geometry_msgs::msg::PoseStamped to_pose_message(const SE3Framed& pose);

// Transform-related messages
Eigen::Vector3d    get_translation(const geometry_msgs::msg::Transform&);
Eigen::Vector3d    get_translation(const geometry_msgs::msg::TransformStamped&);
Eigen::Quaterniond get_rotation(const geometry_msgs::msg::Transform&);
Eigen::Quaterniond get_rotation(const geometry_msgs::msg::TransformStamped&);
Eigen::Affine3d    get_transform(const geometry_msgs::msg::Transform&);
Eigen::Affine3d    get_transform(const geometry_msgs::msg::TransformStamped&);

geometry_msgs::msg::Transform to_transform_message(
        const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation
);
geometry_msgs::msg::Transform to_transform_message(const riemann::SE3Point& pose);

}  // namespace mdv::ros2


#endif  // MDV_ROS2_CONVERSIONS_HPP
