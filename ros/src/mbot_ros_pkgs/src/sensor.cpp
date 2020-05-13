
#include "sensor.h"
#include "airsim_client_factory.h"

Sensor::Sensor(ros::NodeHandle nh, double update_interval)
  : airsim_client_(AirSimClientFactory::instance()->getClient()) {
    pub_timer_ =
      nh.createTimer(ros::Duration(update_interval), &Sensor::timerCallback, this);
}

std::shared_ptr<msr::airlib::CarRpcLibClient> Sensor::getAirSimClient() const {
    return airsim_client_;
}

void Sensor::timerCallback(const ros::TimerEvent&) {
    update();
}
