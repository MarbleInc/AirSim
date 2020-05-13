
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
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

    sensor_msgs::NavSatFix convertToNavSatFix(const msr::airlib::GeoPoint& geo_point) const;

    std::string vehicle_name_;
    std::string gps_name_;
    ros::Publisher gps_pub_;
};
