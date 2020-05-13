
#include "camera.h"
#include <cv_bridge/cv_bridge.h>
#include "math_common.h"
#include "airsim_client_factory.h"

using namespace msr::airlib;

// Mapping of image type to a string suffix for the topic name
const std::unordered_map<int, std::string> image_type_map = {
    { 0, "Scene" },
    { 1, "DepthPlanner" },
    { 2, "DepthPerspective" },
    { 3, "DepthVis" },
    { 4, "DisparityNormalized" },
    { 5, "Segmentation" },
    { 6, "SurfaceNormals" },
    { 7, "Infrared" }
};

Camera::Camera(
  ros::NodeHandle& nh,
  image_transport::ImageTransport& image_transporter,
  const std::string& vehicle_name,
  const std::string& camera_name,
  const msr::airlib::AirSimSettings::CaptureSetting setting,
  double update_interval)
  : Sensor(nh, update_interval)
  , vehicle_name_(vehicle_name)
  , camera_name_(camera_name) {

    msr::airlib::ImageCaptureBase::ImageType image_type
      = msr::airlib::Utils::toEnum<msr::airlib::ImageCaptureBase::ImageType>(setting.image_type);

    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
    if (setting.image_type == 0 || setting.image_type == 5 ||
          setting.image_type == 6 || setting.image_type == 7) {
        image_request_ = msr::airlib::ImageCaptureBase::ImageRequest(camera_name, image_type, false, false);
    }
    // if {DepthPlanner, DepthPerspective,DepthVis, DisparityNormalized}, get float image
    else {
        image_request_ = msr::airlib::ImageCaptureBase::ImageRequest(camera_name, image_type, true);
    }

    camera_info_ = generateCameraInfo(camera_name, setting);

    image_pub_ =
      image_transporter.advertise(vehicle_name + "/" + camera_name + "/" + image_type_map.at(setting.image_type), 5);
    cam_info_pub_ =
      nh.advertise<sensor_msgs::CameraInfo>(vehicle_name + "/" + camera_name + "/camera_info", 10);
}

void Camera::update() {
    std::vector<ImageCaptureBase::ImageRequest> requests;
    requests.push_back(image_request_);

    auto img_response = getAirSimClient()->simGetImages(requests, vehicle_name_);

    if (img_response.size() == 1) {
        auto image = img_response.at(0);
        if (image.time_stamp == 0) {
            std::cerr << "Render request failed!" << std::endl;
            return;
        }

        //auto image_ptr = convertToImageMsg(image, image_request_.camera_name + "_optical");
        sensor_msgs::ImagePtr image_ptr;

        // DepthPlanner / DepthPerspective / DepthVis / DisparityNormalized
        if (image.pixels_as_float)
        {
            image_ptr = convertToDepthMsg(image, image_request_.camera_name + "_optical");
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else
        {
            image_ptr = convertToImageMsg(image, image_request_.camera_name + "_optical");
        }

        image_pub_.publish(image_ptr);

        // Publish camera info too
        camera_info_.header.stamp = ros::Time::now();
        cam_info_pub_.publish(camera_info_);
    }
}

sensor_msgs::ImagePtr Camera::convertToImageMsg(
  const ImageCaptureBase::ImageResponse& img_response,
  const std::string& camera_frame_id) {
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = ros::Time::now();
    img_msg_ptr->header.frame_id = camera_frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr Camera::convertToDepthMsg(
  const ImageCaptureBase::ImageResponse& img_response,
  const std::string& camera_frame_id) {
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = decodeDepth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = ros::Time::now();
    depth_img_msg->header.frame_id = camera_frame_id;
    return depth_img_msg;
}

cv::Mat Camera::decodeDepth(const ImageCaptureBase::ImageResponse& img_response) const {
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++) {
        for (int col = 0; col < img_width; col++) {
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
        }
    }
    return mat;
}

sensor_msgs::CameraInfo Camera::generateCameraInfo(
  const std::string& camera_name,
  const msr::airlib::AirSimSettings::CaptureSetting& setting) const {
    sensor_msgs::CameraInfo cam_info_msg;
    cam_info_msg.header.frame_id = camera_name + "_optical";
    cam_info_msg.height = setting.height;
    cam_info_msg.width = setting.width;
    float f_x = (setting.width / 2.0) / tan(math_common::deg2rad(setting.fov_degrees / 2.0));
    // todo focal length in Y direction should be same as X it seems. this can change in future a scene capture component which exactly correponds to a cine camera
    // float f_y = (capture_setting.height / 2.0) / tan(math_common::deg2rad(fov_degrees / 2.0));
    cam_info_msg.K = {f_x, 0.0, setting.width / 2.0,
                        0.0, f_x, setting.height / 2.0,
                        0.0, 0.0, 1.0};
    cam_info_msg.P = {f_x, 0.0, setting.width / 2.0, 0.0,
                        0.0, f_x, setting.height / 2.0, 0.0,
                        0.0, 0.0, 1.0, 0.0};
    return cam_info_msg;
}
