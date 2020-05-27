
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <vehicles/car/api/CarRpcLibClient.hpp>
#include "sensor.h"

class Gps : public Sensor {
public:
    Gps(
      ros::NodeHandle& nh,
      const std::string& vehicle_name,
      const std::string& gps_name,
      double update_interval);

protected:
    virtual void update();

private:

    sensor_msgs::NavSatFix convertToNavSatFix(const msr::airlib::GeoPoint& geo_point, double timestamp) const;

    std::string vehicle_name_;
    std::string gps_name_;
    ros::Publisher gps_pub_;
    std::shared_ptr<msr::airlib::CarRpcLibClient> airsim_client_;
};
