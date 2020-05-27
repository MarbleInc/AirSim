
#pragma once

#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <vehicles/car/api/CarRpcLibClient.hpp>
#include <image_transport/image_transport.h>
#include "sensor.h"

class CaptureDevices : public Sensor {
public:
    CaptureDevices(
      ros::NodeHandle& nh,
      image_transport::ImageTransport& image_transporter,
      const std::string& vehicle_name,
      double update_interval);

    void addCamera(
      const std::string& camera_name,
      const msr::airlib::AirSimSettings::CaptureSetting& setting);

protected:
    void update();

private:
    struct Camera {
        image_transport::Publisher image_pub;
        ros::Publisher cam_info_pub;
        sensor_msgs::CameraInfo camera_info;
    };

    void doImageRequest(
      msr::airlib::CarRpcLibClient* client,
      const msr::airlib::ImageCaptureBase::ImageRequest& image_request);

    sensor_msgs::CameraInfo generateCameraInfo(
      const std::string& camera_name,
      const msr::airlib::AirSimSettings::CaptureSetting& setting) const;

    sensor_msgs::ImagePtr convertToImageMsg(
      const msr::airlib::ImageCaptureBase::ImageResponse& img_response,
      const std::string& camera_frame_id);

    sensor_msgs::ImagePtr convertToDepthMsg(
      const msr::airlib::ImageCaptureBase::ImageResponse& img_response,
      const std::string& camera_frame_id);

    cv::Mat decodeDepth(const msr::airlib::ImageCaptureBase::ImageResponse& img_response) const;

    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    std::string vehicle_name_;
    std::unordered_map<std::string, Camera> cameras_;
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> image_requests_;
    std::vector<std::shared_ptr<msr::airlib::CarRpcLibClient>> clients_;
};
