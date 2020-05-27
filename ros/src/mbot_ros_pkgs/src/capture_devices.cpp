
#include "capture_devices.h"
#include "math_common.h"
#include "airsim_client_factory.h"
#include <cv_bridge/cv_bridge.h>

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


CaptureDevices::CaptureDevices(
  ros::NodeHandle& nh,
  image_transport::ImageTransport& image_transporter,
  const std::string& vehicle_name,
  double update_interval)
  : Sensor(nh, update_interval)
  , nh_(nh)
  , image_transport_(image_transporter)
  , vehicle_name_(vehicle_name) {
}

void CaptureDevices::addCamera(
  const std::string& camera_name,
  const msr::airlib::AirSimSettings::CaptureSetting& setting) {
    auto camera_info = generateCameraInfo(camera_name, setting);

    Camera camera;
    camera.image_pub = image_transport_.advertise(vehicle_name_ + "/" + camera_name + "/" + image_type_map.at(setting.image_type), 5);
    camera.cam_info_pub = nh_.advertise<sensor_msgs::CameraInfo>(vehicle_name_ + "/" + camera_name + "/camera_info", 10);
    camera.camera_info = camera_info;
    cameras_[camera_name] = camera;

    msr::airlib::ImageCaptureBase::ImageType image_type
      = msr::airlib::Utils::toEnum<msr::airlib::ImageCaptureBase::ImageType>(setting.image_type);

    // if scene / segmentation / surface normals / infrared, get uncompressed image with pixels_as_floats = false
    msr::airlib::ImageCaptureBase::ImageRequest image_request;
    if (setting.image_type == 0 || setting.image_type == 5 ||
          setting.image_type == 6 || setting.image_type == 7) {
        image_request = msr::airlib::ImageCaptureBase::ImageRequest(camera_name, image_type, false, false);
    }
    // if {DepthPlanner, DepthPerspective,DepthVis, DisparityNormalized}, get float image
    else {
        image_request = msr::airlib::ImageCaptureBase::ImageRequest(camera_name, image_type, true);
    }

    image_requests_.push_back(image_request);

    clients_.push_back(AirSimClientFactory::instance()->getClient());
}

void CaptureDevices::update() {
    // Get all images in parallel
    int i = 0;
    std::vector<std::thread> threads;
    for (auto& image_request : image_requests_) {
        threads.emplace_back(&CaptureDevices::doImageRequest, this, clients_.at(i).get(), image_request);
        i++;
    }

    // Wait for all images to be processed and published
    for (auto& t : threads) {
        t.join();
    }
}

void CaptureDevices::doImageRequest(
  msr::airlib::CarRpcLibClient* client,
  const msr::airlib::ImageCaptureBase::ImageRequest& image_request) {
    std::vector<msr::airlib::ImageCaptureBase::ImageRequest> requests;
    requests.push_back(image_request);
    auto image_responses = client->simGetImages(requests, vehicle_name_);
    if (image_responses.size() > 0) {
        auto image = image_responses.at(0);

        if (image.time_stamp == 0) {
            std::cerr << "Render request failed for camera: " << image.camera_name << std::endl;
            return;
        }

        sensor_msgs::ImagePtr image_ptr;

        // DepthPlanner / DepthPerspective / DepthVis / DisparityNormalized
        if (image.pixels_as_float)
        {
            image_ptr = convertToDepthMsg(image, image.camera_name + "_optical");
        }
        // Scene / Segmentation / SurfaceNormals / Infrared
        else
        {
            image_ptr = convertToImageMsg(image, image.camera_name + "_optical");
        }

        auto& camera = cameras_[image.camera_name];
        camera.image_pub.publish(image_ptr);
        camera.camera_info.header.stamp = ros::Time(image.time_stamp / 1e9);
        camera.cam_info_pub.publish(camera.camera_info);
    }
}

sensor_msgs::CameraInfo CaptureDevices::generateCameraInfo(
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

sensor_msgs::ImagePtr CaptureDevices::convertToImageMsg(
  const ImageCaptureBase::ImageResponse& img_response,
  const std::string& camera_frame_id) {
    sensor_msgs::ImagePtr img_msg_ptr = boost::make_shared<sensor_msgs::Image>();
    img_msg_ptr->data = img_response.image_data_uint8;
    img_msg_ptr->step = img_response.width * 3; // todo un-hardcode. image_width*num_bytes
    img_msg_ptr->header.stamp = ros::Time(img_response.time_stamp / 1e9);
    img_msg_ptr->header.frame_id = camera_frame_id;
    img_msg_ptr->height = img_response.height;
    img_msg_ptr->width = img_response.width;
    img_msg_ptr->encoding = "bgr8";
    img_msg_ptr->is_bigendian = 0;
    return img_msg_ptr;
}

sensor_msgs::ImagePtr CaptureDevices::convertToDepthMsg(
  const ImageCaptureBase::ImageResponse& img_response,
  const std::string& camera_frame_id) {
    // todo using img_response.image_data_float direclty as done get_img_msg_from_response() throws an error,
    // hence the dependency on opencv and cv_bridge. however, this is an extremely fast op, so no big deal.
    cv::Mat depth_img = decodeDepth(img_response);
    sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    depth_img_msg->header.stamp = ros::Time(img_response.time_stamp / 1e9);
    depth_img_msg->header.frame_id = camera_frame_id;
    return depth_img_msg;
}

cv::Mat CaptureDevices::decodeDepth(const ImageCaptureBase::ImageResponse& img_response) const {
    cv::Mat mat(img_response.height, img_response.width, CV_32FC1, cv::Scalar(0));
    int img_width = img_response.width;

    for (int row = 0; row < img_response.height; row++) {
        for (int col = 0; col < img_width; col++) {
            mat.at<float>(row, col) = img_response.image_data_float[row * img_width + col];
        }
    }
    return mat;
}
