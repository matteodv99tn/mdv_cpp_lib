#ifndef MDV_ROS2_SE3_HPP
#define MDV_ROS2_SE3_HPP

#include <Eigen/Geometry>
#include <string>
#include <utility>

#include "mdv/riemann_geometry/se3.hpp"

namespace mdv::ros2 {

using SE3 = ::mdv::riemann::SE3Point;

/**
 * struct SE3Framed - SE3 object with additional information about the name of the
 * reference frame.
 */
struct SE3Framed : SE3 {  // NOLINT
    SE3Framed() = default;

    SE3Framed(const SE3& pose, std::string frame) :
            SE3(pose), frame_name(std::move(frame)) {}

    SE3Framed(
            const Eigen::Vector3d&    pos,
            const Eigen::Quaterniond& ori,
            const std::string&        frame
    ) :
            SE3(pos, ori), frame_name(frame) {}

    std::string frame_name;
};

std::string describe(const SE3& pose);
std::string describe(const SE3Framed& pose);

}  // namespace mdv::ros2

mdv::ros2::SE3 operator*(const Eigen::Affine3d& transform, const mdv::ros2::SE3& pose);
mdv::ros2::SE3Framed operator*(
        const Eigen::Affine3d& transform, const mdv::ros2::SE3Framed& pose
);


#endif  // MDV_ROS2_SE3_HPP
