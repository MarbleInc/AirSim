
#pragma once

#include <memory>
#include <ros/ros.h>

class Sensor {
public:
    Sensor(ros::NodeHandle nh, double update_interval);

    virtual ~Sensor() = default;

    virtual void tick(double timestamp);

protected:
    virtual void update() = 0;

    double update_interval_;
    double last_update_timestamp_;
};
