
#include "mbot_sim.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include "math_common.h"
#include "airsim_client_factory.h"

static std::string base_link_frame = "base_link";
static std::string base_link_ned_frame = "base_link_ned";

MbotSim::MbotSim()
  : nh_("~")
  , airsim_client_(AirSimClientFactory::instance()->getClient()) {
    connectToAirSim();
}

void MbotSim::start() {
    // Parse settings and construct vehicle and sensor wrappers
    parseSettings();

    // Get all actors we care to track in the scene
    auto actor_names = airsim_client_->simListSceneObjects("(Pedestrian|Vehicle)_.*");
    for (auto actor_name : actor_names) {
        Actor actor;
        actor.name = actor_name;
        actor.pose_pub = nh_.advertise<geometry_msgs::PoseStamped>(actor_name + "/pose", 1);
        actors_.push_back(actor);
    }

    // Begin updating vehicles periodically
    double update_airsim_control_every_n_sec;
    nh_.getParam("update_airsim_control_every_n_sec", update_airsim_control_every_n_sec);
    vehicle_update_timer_ = nh_.createTimer(ros::Duration(update_airsim_control_every_n_sec), &MbotSim::doControlCallback, this);

    // Begin updating ground truth poses periodically
    double update_airsim_ground_truth_every_n_sec;
    nh_.getParam("update_airsim_ground_truth_every_n_sec", update_airsim_ground_truth_every_n_sec);
    ground_truth_update_timer_ = nh_.createTimer(ros::Duration(update_airsim_ground_truth_every_n_sec), &MbotSim::doGroundTruthCallback, this);
}

void MbotSim::parseSettings() {
    image_transport::ImageTransport image_transporter(nh_);

    for (const auto& vehicle_map : AirSimSettings::singleton().vehicles) {
        auto& vehicle_name = vehicle_map.first;
        auto& vehicle_setting = vehicle_map.second;

        std::cout << "Processing vehicle: " << vehicle_name << std::endl;

        // Create our vehicle wrapper
        Vehicle vehicle;
        vehicle.name = vehicle_name;

        for (auto& camera : vehicle_setting->cameras)
        {
            auto& camera_name = camera.first;
            auto& camera_setting = camera.second;

            for (const auto& capture_element : camera_setting.capture_settings)
            {
                auto& capture_setting = capture_element.second;

                // Skip capture settings that are missing a FOV setting
                if (!std::isnan(capture_setting.fov_degrees)) {
                    std::cout << "Adding camera " << camera_name << std::endl;
                    double update_img_response_every_n_sec;
                    nh_.getParam("update_img_response_every_n_sec", update_img_response_every_n_sec);
                    vehicle.cameras.push_back(
                      std::make_shared<Camera>(
                        nh_, image_transporter, vehicle_name, camera_name, capture_setting, update_img_response_every_n_sec));
                    addCameraStaticTf(vehicle_name, camera_name, camera_setting);
                }
            }
        }

        for (auto& sensor : vehicle_setting->sensors) {
            auto& sensor_name = sensor.first;
            auto& sensor_setting = sensor.second;

            if (sensor_setting->sensor_type == SensorBase::SensorType::Lidar) {
                std::cout << "Adding lidar named: " << sensor_name << std::endl;
                double update_lidar_every_n_sec;
                nh_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
                vehicle.lidars.push_back(std::make_shared<Lidar>(nh_, vehicle_name, sensor_name, update_lidar_every_n_sec));
                auto lidar_setting = *static_cast<msr::airlib::AirSimSettings::LidarSetting*>(sensor_setting.get());
                auto static_tf = getSensorStaticTf(vehicle_name, sensor_name, lidar_setting.position, lidar_setting.rotation);
                static_tf_pub_.sendTransform(static_tf);
            }
            else if (sensor_setting->sensor_type == SensorBase::SensorType::Gps && !vehicle.gps) {
                std::cout << "Adding GPS named: " << sensor_name << std::endl;
                double update_gps_every_n_sec;
                nh_.getParam("update_gps_every_n_sec", update_gps_every_n_sec);
                vehicle.gps = std::make_shared<Gps>(nh_, vehicle_name, sensor_name, update_gps_every_n_sec);
            }
            else if (sensor_setting->sensor_type == SensorBase::SensorType::Imu) {
                std::cout << "Adding IMU named: " << sensor_name << std::endl;
                vehicle.imu_name = sensor_name;
                vehicle.imu_pub = nh_.advertise<sensor_msgs::Imu>(vehicle_name + "/imu", 10);
                // The IMU has no position so assume it sits at base link with no RPY
                auto static_tf = getSensorStaticTf(vehicle_name, sensor_name, {0, 0, 0}, {0, 0, 0}, false);
                static_tf_pub_.sendTransform(static_tf);
            }
        }

        // All vehicles have odometry
        vehicle.odom_pub = nh_.advertise<nav_msgs::Odometry>(vehicle_name + "/odom", 10);

        // Create a base link frame in NWU
        geometry_msgs::TransformStamped local_nwu_tf;
        local_nwu_tf.header.frame_id = vehicle_name + "/" + base_link_frame;
        local_nwu_tf.child_frame_id = vehicle_name + "/" + base_link_ned_frame;
        tf2::Quaternion quat;
        quat.setRPY(M_PI, 0, 0);
        local_nwu_tf.transform.rotation.x = quat.x();
        local_nwu_tf.transform.rotation.y = quat.y();
        local_nwu_tf.transform.rotation.z = quat.z();
        local_nwu_tf.transform.rotation.w = quat.w();
        static_tf_pub_.sendTransform(local_nwu_tf);

        vehicles_.push_back(vehicle);
    }
}

void MbotSim::doGroundTruthCallback(const ros::TimerEvent&) {
    for (auto& actor : actors_) {
        client_mutex_.lock();
        auto pose = airsim_client_->simGetObjectPose(actor.name);
        client_mutex_.unlock();

        // Pose is in NED so transform to NWU
        Eigen::Isometry3d nwu_pose;
        nwu_pose = nwu_transform_.toNwu(pose.orientation.cast<double>());
        nwu_pose.translation() = nwu_transform_.toNwu(pose.position.cast<double>());

        // Publish the pose
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose = tf2::toMsg(nwu_pose);
        actor.pose_pub.publish(pose_msg);
    }
}

void MbotSim::doControlCallback(const ros::TimerEvent&) {
    for (auto& vehicle : vehicles_) {
        try {
            client_mutex_.lock();
            auto state = airsim_client_->getCarState(vehicle.name);
            client_mutex_.unlock();

            updateOdometry(vehicle, state);
            updateImu(vehicle);
            // TODO send velocity commands to AirSim
        }
        catch (rpc::rpc_error& e)
        {
            std::string msg = e.get_error().as<std::string>();
            std::cout << "Exception raised by the API:" << std::endl << msg << std::endl;
        }
    }
}

void MbotSim::updateOdometry(Vehicle& vehicle, const msr::airlib::CarApiBase::CarState& state) {
    auto& position = state.kinematics_estimated.pose.position;
    auto& orientation = state.kinematics_estimated.pose.orientation;

    // Convert AirSim pose from NED to NWU
    Eigen::Isometry3d pose;
    pose = nwu_transform_.toNwu(orientation.cast<double>());
    pose.translation() = nwu_transform_.toNwu(position.cast<double>());

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = vehicle.name + "/odom";
    odom_msg.child_frame_id = vehicle.name + "/rear_axle";
    odom_msg.pose.pose = tf2::toMsg(pose);
    odom_msg.twist.twist = nwu_transform_.toNwu(state.kinematics_estimated.twist);
    vehicle.odom_pub.publish(odom_msg);

    // Publish odom transform
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header = odom_msg.header;
    odom_tf.header.frame_id = vehicle.name + "/rear_axle";
    odom_tf.child_frame_id = vehicle.name + "/odom";
    // Invert the pose because the TF is from rear_axle to odom
    pose = pose.inverse();
    odom_tf.transform = tf2::eigenToTransform(pose).transform;
    tf_broadcaster_.sendTransform(odom_tf);
}

void MbotSim::updateImu(Vehicle& vehicle) {
    auto imu_data = airsim_client_->getImuData(vehicle.imu_name, vehicle.name);

    // Convert IMU data from NED to NWU
    auto orientation = nwu_transform_.toNwu(imu_data.orientation.cast<double>());
    auto angular_velocity = nwu_transform_.toNwu(imu_data.angular_velocity.cast<double>());
    auto linear_acceleration = nwu_transform_.toNwu(imu_data.linear_acceleration.cast<double>());

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = vehicle.imu_name;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation.x = orientation.x();
    imu_msg.orientation.y = orientation.y();
    imu_msg.orientation.z = orientation.z();
    imu_msg.orientation.w = orientation.w();
    imu_msg.angular_velocity.x = angular_velocity.x();
    imu_msg.angular_velocity.y = angular_velocity.y();
    imu_msg.angular_velocity.z = angular_velocity.z();
    imu_msg.linear_acceleration.x = linear_acceleration.x();
    imu_msg.linear_acceleration.y = linear_acceleration.y();
    imu_msg.linear_acceleration.z = linear_acceleration.z();
    vehicle.imu_pub.publish(imu_msg);
}

void MbotSim::connectToAirSim() {
    try {
        airsim_client_->confirmConnection();
    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }
}

geometry_msgs::TransformStamped MbotSim::getSensorStaticTf(
  const std::string& vehicle_name,
  const std::string& sensor_frame_id,
  const Vector3r& position,
  const AirSimSettings::Rotation& rotation,
  bool is_ned) {

    geometry_msgs::TransformStamped static_tf;
    static_tf.header.stamp = ros::Time::now();
    if (is_ned) {
        static_tf.header.frame_id = vehicle_name + "/" + base_link_ned_frame;
    }
    else {
        static_tf.header.frame_id = vehicle_name + "/" + base_link_frame;
    }
    static_tf.child_frame_id = sensor_frame_id;

    // Don't add NaNs to the TF
    if (!std::isnan(position.x())) {
        static_tf.transform.translation.x = position.x();
    }
    if (!std::isnan(position.y())) {
        static_tf.transform.translation.y = position.y();
    }
    if (!std::isnan(position.z())) {
        static_tf.transform.translation.z = position.z();
    }

    // Clean up any NaNs in the rotation
    double roll = rotation.roll;
    if (std::isnan(roll)) {
        roll = 0.0;
    }
    double pitch = rotation.pitch;
    if (std::isnan(pitch)) {
        pitch = 0.0;
    }
    double yaw = rotation.yaw;
    if (std::isnan(yaw)) {
        yaw = 0.0;
    }

    tf2::Quaternion quat;
    quat.setRPY(math_common::deg2rad(roll), math_common::deg2rad(pitch), math_common::deg2rad(yaw));
    static_tf.transform.rotation.x = quat.x();
    static_tf.transform.rotation.y = quat.y();
    static_tf.transform.rotation.z = quat.z();
    static_tf.transform.rotation.w = quat.w();

    return static_tf;
}

void MbotSim::addCameraStaticTf(
  const std::string& vehicle_name,
  const std::string& camera_name,
  const msr::airlib::AirSimSettings::CameraSetting& setting) {

    auto static_tf_body = getSensorStaticTf(vehicle_name, camera_name + "_body", setting.position, setting.rotation);

    geometry_msgs::TransformStamped static_tf_optical = static_tf_body;
    static_tf_optical.header.stamp = ros::Time::now();
    static_tf_optical.child_frame_id = camera_name + "_optical";

    tf2::Quaternion quat_cam_body;
    tf2::Quaternion quat_cam_optical;
    tf2::convert(static_tf_body.transform.rotation, quat_cam_body);
    tf2::Matrix3x3 mat_cam_body(quat_cam_body);
    tf2::Matrix3x3 mat_cam_optical;
    mat_cam_optical.setValue(mat_cam_body.getColumn(1).getX(), mat_cam_body.getColumn(2).getX(), mat_cam_body.getColumn(0).getX(),
                             mat_cam_body.getColumn(1).getY(), mat_cam_body.getColumn(2).getY(), mat_cam_body.getColumn(0).getY(),
                             mat_cam_body.getColumn(1).getZ(), mat_cam_body.getColumn(2).getZ(), mat_cam_body.getColumn(0).getZ());
    mat_cam_optical.getRotation(quat_cam_optical);
    quat_cam_optical.normalize();
    tf2::convert(quat_cam_optical, static_tf_optical.transform.rotation);

    static_tf_pub_.sendTransform(static_tf_body);
    static_tf_pub_.sendTransform(static_tf_optical);
}
