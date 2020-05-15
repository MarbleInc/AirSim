
#pragma once

#include <vector>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <vehicles/car/api/CarRpcLibClient.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "gps.h"
#include "lidar.h"
#include "camera.h"
#include "nwu_transform.h"
#include "airsim_settings_parser.h"

class MbotSim {
public:
    MbotSim();

    void start();

private:
    struct Vehicle {
        std::string name;
        std::vector<std::shared_ptr<Camera>> cameras;
        std::vector<std::shared_ptr<Lidar>> lidars;
        std::shared_ptr<Gps> gps;
        ros::Publisher odom_pub;
        ros::Publisher imu_pub;
        std::string imu_name;
    };

    void connectToAirSim();

    void parseSettings();

    void doControlCallback(const ros::TimerEvent&);

    void doGroundTruthCallback(const ros::TimerEvent&);

    geometry_msgs::TransformStamped getSensorStaticTf(
      const std::string& vehicle_name,
      const std::string& sensor_frame_id,
      const Vector3r& position,
      const AirSimSettings::Rotation& rotation,
      bool is_ned = true);

    void addCameraStaticTf(
      const std::string& vehicle_name,
      const std::string& camera_name,
      const msr::airlib::AirSimSettings::CameraSetting& setting);

    void updateOdometry(
      Vehicle& vehicle,
      const msr::airlib::CarApiBase::CarState& state);

    void updateImu(Vehicle& vehicle);

    AirSimSettingsParser settings_parser_;
    ros::NodeHandle nh_;
    std::vector<Vehicle> vehicles_;
    std::vector<std::string> actors_;
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Timer vehicle_update_timer_;
    ros::Timer ground_truth_update_timer_;
    ros::Publisher tracked_objects_pub_;

    std::shared_ptr<msr::airlib::CarRpcLibClient> airsim_client_;
    std::mutex client_mutex_;

    Eigen::Vector3d initial_position_;

    NwuTransform nwu_transform_;
};
