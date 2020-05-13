
#include "nwu_transform.h"

NwuTransform::NwuTransform() {
    // 180 degree rotation around the X-axis to go from NED to NWU
    ned_to_nwu_quat_ =
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
}

Eigen::Vector3d NwuTransform::toNwu(const Eigen::Vector3d& ned_position) {
    return ned_to_nwu_quat_ * ned_position;
}

Eigen::Quaterniond NwuTransform::toNwu(const Eigen::Quaterniond& ned_orientation) {
    // NWU inverts Y and Z axes
    return Eigen::Quaterniond(
      ned_orientation.w(),
      ned_orientation.x(),
      -ned_orientation.y(),
      -ned_orientation.z());
}

geometry_msgs::Twist NwuTransform::toNwu(const msr::airlib::Twist& twist) {
    // Invert Y and Z axes
    geometry_msgs::Twist nwu_twist;
    nwu_twist.linear.x = twist.linear.x();
    nwu_twist.linear.y = -twist.linear.y();
    nwu_twist.linear.z = -twist.linear.z();
    nwu_twist.angular.x = twist.angular.x();
    nwu_twist.angular.y = -twist.angular.y();
    nwu_twist.angular.z = -twist.angular.z();
    return nwu_twist;
}
