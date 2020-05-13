
#include "airsim_client_factory.h"
#include <ros/ros.h>

std::weak_ptr<AirSimClientFactory> AirSimClientFactory::instance_;

AirSimClientFactory::AirSimClientFactory() {
    ros::NodeHandle nh("~");
    nh.getParam("", host_ip_);
}

std::shared_ptr<AirSimClientFactory> AirSimClientFactory::instance() {
    if (auto ptr = instance_.lock()) {
        return ptr;
    }
    auto instance = std::shared_ptr<AirSimClientFactory>(new AirSimClientFactory());
    instance_ = instance;
    return instance;
}

std::shared_ptr<msr::airlib::CarRpcLibClient> AirSimClientFactory::getClient() {
    return std::make_shared<msr::airlib::CarRpcLibClient>(host_ip_);
};
