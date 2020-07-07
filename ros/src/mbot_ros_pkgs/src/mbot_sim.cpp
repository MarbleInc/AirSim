#include "mbot_sim.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include "math_common.h"
#include "airsim_client_factory.h"

static std::string world_frame = "world";
static std::string base_link_frame = "base_link";
static std::string base_link_ned_frame = "base_link_ned";

MbotSim::MbotSim()
  : nh_("~")
  , airsim_client_(AirSimClientFactory::instance()->getClient()) {
    connectToAirSim();
}

void MbotSim::start() {
    // Pause the simulation -- it will be stepped
    airsim_client_->simPause(true);

    // Parse settings and construct vehicle and sensor wrappers
    parseSettings();

    // Get all actors we care to track in the scene
    actors_ = airsim_client_->simListSceneObjects("(Pedestrian|Vehicle)_.*");

    start_recording_sub = nh_.subscribe("/record_gt_data", 1, &MbotSim::start_recording_ground_truth, this);
    tracked_objects_pub_ = nh_.advertise<mbot_base::TrackedObjectArray>("tracked_objects", 1);
    clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);
}

void MbotSim::start_recording_ground_truth(const std_msgs::Bool::ConstPtr& status){
    if(status->data && !recording_ground_truth_data){
        // start recording ground truth data
        std::cout<<"Starting to write ground truth data"<<std::endl;
        recording_ground_truth_data = true;
    }
    else if(recording_ground_truth_data){
        // stop recording ground truth data and write to disk
        recording_ground_truth_data = false;
        vector<shared_ptr<mbot_base::TrackedObjectArray>> arr = getTrackedObjectArray();  
        std::cout<<"Ending writing ground truth data, "<< arr.size()<< " timestamps recorded"<<std::endl;

    }
}

void MbotSim::step(double step_time) {
  double timestamp;
  for (auto& vehicle : vehicles_) {
      try {
          auto state = airsim_client_->getCarState(vehicle.name);

          // Publish unreal clock
          timestamp = (state.timestamp) / 1e9; // nanoseconds to seconds
          rosgraph_msgs::Clock clock_msg;
          clock_msg.clock = ros::Time(timestamp);
          clock_pub_.publish(clock_msg);

          updateOdometry(vehicle, state);
          updateImu(vehicle);

          for (auto& sensor : vehicle.sensors) {
              sensor->tick(timestamp);
          }

          updateGroundTruth(timestamp);

          // TODO send velocity commands to AirSim
      }
      catch (rpc::rpc_error& e)
      {
          std::string msg = e.get_error().as<std::string>();
          std::cout << "Exception raised by the API:" << std::endl << msg << std::endl;
      }
  }

  for (auto& static_tf : static_tfs_) {
      static_tf.header.stamp = ros::Time(timestamp) + ros::Duration(step_time);
      tf_broadcaster_.sendTransform(static_tf);
  }

  airsim_client_->simContinueForTime(step_time);
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

        if (vehicles_.empty()) {
            // Set initial position from first vehicle
            initial_position_ = nwu_transform_.toNwu(vehicle_setting->position.cast<double>());
        }

        double update_img_response_every_n_sec;
        nh_.getParam("update_img_response_every_n_sec", update_img_response_every_n_sec);
        auto capture_devices =
          std::make_shared<CaptureDevices>(nh_, image_transporter, vehicle_name, update_img_response_every_n_sec);

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
                    capture_devices->addCamera(camera_name, capture_setting);
                    addCameraStaticTf(vehicle_name, camera_name, camera_setting);
                }
            }
        }

        vehicle.sensors.push_back(capture_devices);

        for (auto& sensor : vehicle_setting->sensors) {
            auto& sensor_name = sensor.first;
            auto& sensor_setting = sensor.second;

            if (sensor_setting->sensor_type == SensorBase::SensorType::Lidar) {
                std::cout << "Adding lidar named: " << sensor_name << std::endl;
                double update_lidar_every_n_sec;
                nh_.getParam("update_lidar_every_n_sec", update_lidar_every_n_sec);
                vehicle.sensors.push_back(std::make_shared<Lidar>(nh_, vehicle_name, sensor_name, update_lidar_every_n_sec));
                auto lidar_setting = *static_cast<msr::airlib::AirSimSettings::LidarSetting*>(sensor_setting.get());
                auto static_tf = getSensorStaticTf(vehicle_name, sensor_name, lidar_setting.position, lidar_setting.rotation);
                static_tfs_.push_back(static_tf);
            }
            else if (sensor_setting->sensor_type == SensorBase::SensorType::Gps) {
                std::cout << "Adding GPS named: " << sensor_name << std::endl;
                double update_gps_every_n_sec;
                nh_.getParam("update_gps_every_n_sec", update_gps_every_n_sec);
                vehicle.sensors.push_back(std::make_shared<Gps>(nh_, vehicle_name, sensor_name, update_gps_every_n_sec));
            }
            else if (sensor_setting->sensor_type == SensorBase::SensorType::Imu) {
                std::cout << "Adding IMU named: " << sensor_name << std::endl;
                vehicle.imu_name = sensor_name;
                vehicle.imu_pub = nh_.advertise<sensor_msgs::Imu>(vehicle_name + "/imu", 10);
                // The IMU has no position so assume it sits at base link with no RPY
                auto static_tf = getSensorStaticTf(vehicle_name, sensor_name, {0, 0, 0}, {0, 0, 0}, false);
                static_tfs_.push_back(static_tf);
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
        static_tfs_.push_back(local_nwu_tf);

        vehicles_.push_back(vehicle);
    }
}

void MbotSim::updateGroundTruth(double timestamp) {
    shared_ptr<mbot_base::TrackedObjectArray> tracks = std::make_shared<mbot_base::TrackedObjectArray>();
    tracks->header.stamp = ros::Time(timestamp);
    tracks->header.frame_id = world_frame;


    for (auto& actor : actors_) {
        client_mutex_.lock();
        auto pose = airsim_client_->simGetObjectPose(actor);
        auto twist = airsim_client_->simGetObjectTwist(actor);
        client_mutex_.unlock();

        // Convert to NED to NWU
        Eigen::Vector3d position = nwu_transform_.toNwu(pose.position.cast<double>());
        Eigen::Quaterniond orientation = nwu_transform_.toNwu(pose.orientation.cast<double>());
        Eigen::Vector3d velocity = nwu_transform_.toNwu(twist.linear.cast<double>());

        shared_ptr<mbot_base::TrackedObject> track = std::make_shared<mbot_base::TrackedObject>();
        track->header.stamp = ros::Time(timestamp);
        track->header.frame_id = world_frame;
        track->id = std::hash<std::string>{}(actor);
        track->position.x = position.x();
        track->position.y = position.y();
        track->position.z = position.z();
        track->velocity.x = velocity.x();
        track->velocity.y = velocity.y();
        track->velocity.z = velocity.z();
        track->yaw_rate = -twist.angular.z();
        track->orientation = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[0];
        track->orientation_known = true;

        if (actor.find("Pedestrian_") == 0) {
            track->classification = "pedestrian";
            track->radius = 0.25;
        }
        else {
            track->classification = "vehicle";
            track->radius = 1.5;
        }

        track->shape.height = 2.0;

        // TODO This should be using geometry utils in marble_structs
        const int SHAPE_POINTS = 24;
        for (int i = 0; i < SHAPE_POINTS; ++i) {
            double theta = 2 * M_PI * i / SHAPE_POINTS;

            geometry_msgs::Point ros_pt;
            ros_pt.x = track->position.x + track->radius * cos(theta);
            ros_pt.y = track->position.y + track->radius * sin(theta);
            ros_pt.z = track->position.z;
            track->shape.footprint.push_back(ros_pt);
        }

        tracks->tracks.push_back(*track);
    }

    if(recording_ground_truth_data){
        std::cout<<"TrackedObjectArray for this timestamp pushed back"<<std::endl;
        array_to_publish.push_back(tracks);
    }

    // tracked_object_array.push_back(track);
    // tracked_objects_pub_.publish(tracks);
}

void MbotSim::updateOdometry(Vehicle& vehicle, const msr::airlib::CarApiBase::CarState& state) {
    auto& position = state.kinematics_estimated.pose.position;
    auto& orientation = state.kinematics_estimated.pose.orientation;

    // Convert AirSim pose from NED to NWU
    Eigen::Isometry3d pose;
    pose = nwu_transform_.toNwu(orientation.cast<double>());
    pose.translation() = nwu_transform_.toNwu(position.cast<double>());

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time(state.timestamp / 1e9);
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
    odom_tf.transform = tf2::eigenToTransform(pose.inverse()).transform;
    tf_broadcaster_.sendTransform(odom_tf);

    // Compute world pose
    Eigen::Isometry3d world_pose = pose;
    pose.translation() -= initial_position_;

    // Publish world transform to vehicle
    geometry_msgs::TransformStamped world_tf;
    world_tf.header = odom_msg.header;
    world_tf.header.frame_id = world_frame;
    world_tf.child_frame_id = vehicle.name + "/" + base_link_frame;
    world_tf.transform = tf2::eigenToTransform(world_pose).transform;
    tf_broadcaster_.sendTransform(world_tf);
}

void MbotSim::updateImu(Vehicle& vehicle) {
    auto imu_data = airsim_client_->getImuData(vehicle.imu_name, vehicle.name);

    // Convert IMU data from NED to NWU
    auto orientation = nwu_transform_.toNwu(imu_data.orientation.cast<double>());
    auto angular_velocity = nwu_transform_.toNwu(imu_data.angular_velocity.cast<double>());
    auto linear_acceleration = nwu_transform_.toNwu(imu_data.linear_acceleration.cast<double>());

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = vehicle.imu_name;
    imu_msg.header.stamp = ros::Time(imu_data.time_stamp / 1e9);
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

    static_tfs_.push_back(static_tf_body);
    static_tfs_.push_back(static_tf_optical);
}
