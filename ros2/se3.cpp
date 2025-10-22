#include "mdv/ros2/se3.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>

namespace mdv::ros2 {

std::string
describe(const SE3& pose) {
    return fmt::format(
            "position ({}, {}, {}), orientation {} + ({}, {}, {})",
            pose.pos(0),
            pose.pos(1),
            pose.pos(2),
            pose.ori.w(),
            pose.ori.x(),
            pose.ori.y(),
            pose.ori.z()
    );
}

std::string
describe(const SE3Framed& pose) {
    return fmt::format(
            "{}, ref. frame: {}",
            describe(static_cast<const SE3&>(pose)),
            pose.frame_name
    );
}


}  // namespace mdv::ros2

mdv::ros2::SE3
operator*(const Eigen::Affine3d& transform, const mdv::ros2::SE3& pose) {
    Eigen::Quaterniond ori(transform.rotation() * pose.ori);
    return {transform * pose.pos, ori};
}

mdv::ros2::SE3Framed
operator*(const Eigen::Affine3d& transform, const mdv::ros2::SE3Framed& pose) {
    return {transform * static_cast<const mdv::ros2::SE3&>(pose), pose.frame_name};
}
