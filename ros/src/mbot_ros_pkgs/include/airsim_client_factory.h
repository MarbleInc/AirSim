
#pragma once

#include <memory>
#include <vehicles/car/api/CarRpcLibClient.hpp>

class AirSimClientFactory {
public:
    static std::shared_ptr<AirSimClientFactory> instance();

    std::shared_ptr<msr::airlib::CarRpcLibClient> getClient();

private:
    AirSimClientFactory();

    std::string host_ip_;
    static std::weak_ptr<AirSimClientFactory> instance_;
};
