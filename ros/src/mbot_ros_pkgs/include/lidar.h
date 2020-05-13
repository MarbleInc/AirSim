
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor.h"

class Lidar : public Sensor {
public:
    Lidar(
      ros::NodeHandle& nh,
      const std::string& vehicle_name,
      const std::string& sensor_name,
      double update_interval);

protected:
    virtual void update();

private:

    sensor_msgs::PointCloud2 convertToPointCloud(const msr::airlib::LidarData& lidar_data) const;

    std::string vehicle_name_;
    std::string sensor_name_;
    ros::Publisher lidar_pub_;
};
