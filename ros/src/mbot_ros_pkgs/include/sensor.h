
#pragma once

#include <memory>
#include <ros/ros.h>
#include <vehicles/car/api/CarRpcLibClient.hpp>

class Sensor {
public:
    Sensor(ros::NodeHandle nh, double update_interval);

    virtual ~Sensor() = default;

protected:
    virtual void update() = 0;

    std::shared_ptr<msr::airlib::CarRpcLibClient> getAirSimClient() const;

private:
    void timerCallback(const ros::TimerEvent&);

    ros::Timer pub_timer_;
    std::shared_ptr<msr::airlib::CarRpcLibClient> airsim_client_;
};
