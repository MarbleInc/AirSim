
#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vehicles/car/api/CarRpcLibClient.hpp>
#include <image_transport/image_transport.h>
#include "sensor.h"

class Camera : public Sensor {
public:
    Camera(
      ros::NodeHandle& nh,
      image_transport::ImageTransport& image_transporter,
      const std::string& vehicle_name,
      const std::string& camera_name,
      const msr::airlib::AirSimSettings::CaptureSetting setting,
      double update_interval);

protected:
    void update();

private:

    sensor_msgs::ImagePtr convertToImageMsg(
      const msr::airlib::ImageCaptureBase::ImageResponse& img_response,
      const std::string& camera_frame_id);

    sensor_msgs::ImagePtr convertToDepthMsg(
      const msr::airlib::ImageCaptureBase::ImageResponse& img_response,
      const std::string& camera_frame_id);

    cv::Mat decodeDepth(const msr::airlib::ImageCaptureBase::ImageResponse& img_response) const;

    sensor_msgs::CameraInfo generateCameraInfo(
      const std::string& camera_name,
      const msr::airlib::AirSimSettings::CaptureSetting& setting) const;

    std::string vehicle_name_;
    std::string camera_name_;
    sensor_msgs::CameraInfo camera_info_;
    msr::airlib::ImageCaptureBase::ImageRequest image_request_;
    image_transport::Publisher image_pub_;
    ros::Publisher cam_info_pub_;
};
