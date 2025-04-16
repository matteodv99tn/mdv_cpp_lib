#include "mdv/ros2/conversions.hpp"

#include "mdv/riemann_geometry/se3.hpp"

namespace mdv::ros2 {

//  ____
// |  _ \ ___  ___  ___
// | |_) / _ \/ __|/ _ \
// |  __/ (_) \__ \  __/
// |_|   \___/|___/\___|
//
Eigen::Vector3d
get_position(const geometry_msgs::msg::Pose& msg) {
    return Eigen::Vector3d({msg.position.x, msg.position.y, msg.position.z});
}

Eigen::Vector3d
get_position(const geometry_msgs::msg::PoseStamped& msg) {
    return get_position(msg.pose);
}

Eigen::Quaterniond
get_orientation(const geometry_msgs::msg::Pose& msg) {
    return Eigen::Quaterniond(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
    );
}

Eigen::Quaterniond
get_orientation(const geometry_msgs::msg::PoseStamped& msg) {
    return get_orientation(msg.pose);
}

SE3
get_pose(const geometry_msgs::msg::Pose& msg) {
    return SE3(get_position(msg), get_orientation(msg));
}

SE3Framed
get_pose(const geometry_msgs::msg::PoseStamped& msg) {
    return {get_pose(msg.pose), msg.header.frame_id};
}

geometry_msgs::msg::Pose
to_pose_message(
        const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation
) {
    geometry_msgs::msg::Pose msg;
    msg.position.x    = position(0);
    msg.position.y    = position(1);
    msg.position.z    = position(2);
    msg.orientation.w = orientation.w();
    msg.orientation.x = orientation.x();
    msg.orientation.y = orientation.y();
    msg.orientation.z = orientation.z();
    return msg;
}

geometry_msgs::msg::Pose
to_pose_message(const SE3& pose) {
    return to_pose_message(pose.pos, pose.ori);
}

geometry_msgs::msg::PoseStamped
to_pose_message(const SE3Framed& pose) {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose            = to_pose_message(static_cast<const SE3&>(pose));
    msg.header.frame_id = pose.frame_name;
    return msg;
}

//  _____                     __
// |_   _| __ __ _ _ __  ___ / _| ___  _ __ _ __ __
//   | || '__/ _` | '_ \/ __| |_ / _ \| '__| '_ ` _ \
//   | || | | (_| | | | \__ \  _| (_) | |  | | | | | |
//   |_||_|  \__,_|_| |_|___/_|  \___/|_|  |_| |_| |_|
//
Eigen::Vector3d
get_translation(const geometry_msgs::msg::Transform& msg) {
    return Eigen::Vector3d({msg.translation.x, msg.translation.y, msg.translation.z});
}

Eigen::Vector3d
get_translation(const geometry_msgs::msg::TransformStamped& msg) {
    return get_translation(msg.transform);
}

Eigen::Quaterniond
get_rotation(const geometry_msgs::msg::Transform& msg) {
    return Eigen::Quaterniond(
            msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z
    );
}

Eigen::Quaterniond
get_rotation(const geometry_msgs::msg::TransformStamped& msg) {
    return get_rotation(msg.transform);
}

Eigen::Affine3d
get_transform(const geometry_msgs::msg::Transform& msg) {
    Eigen::Affine3d tf = Eigen::Affine3d::Identity();
    tf.translate(get_translation(msg));
    tf.rotate(get_rotation(msg));
    return tf;
}

Eigen::Affine3d
get_transform(const geometry_msgs::msg::TransformStamped& msg) {
    return get_transform(msg.transform);
}

geometry_msgs::msg::Transform
to_transform_message(
        const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation
) {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = translation(0);
    msg.translation.y = translation(1);
    msg.translation.z = translation(2);
    msg.rotation.w    = rotation.w();
    msg.rotation.x    = rotation.x();
    msg.rotation.y    = rotation.y();
    msg.rotation.z    = rotation.z();
    return msg;
}

geometry_msgs::msg::Transform
to_transform_message(const SE3& pose) {
    return to_transform_message(pose.pos, pose.ori);
}


}  // namespace mdv::ros2
