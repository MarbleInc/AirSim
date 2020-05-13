#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Twist.h>
#include "common/CommonStructs.hpp"

class NwuTransform {
public:
    NwuTransform();

    Eigen::Vector3d toNwu(const Eigen::Vector3d& ned_position);

    Eigen::Quaterniond toNwu(const Eigen::Quaterniond& ned_orientation);

    geometry_msgs::Twist toNwu(const msr::airlib::Twist& twist);

private:
    Eigen::Quaterniond ned_to_nwu_quat_;
};
