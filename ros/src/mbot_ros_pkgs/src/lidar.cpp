
#include "lidar.h"
#include "airsim_client_factory.h"

using namespace std;

Lidar::Lidar(
  ros::NodeHandle& nh,
  const string& vehicle_name,
  const string& sensor_name,
  double update_interval)
  : Sensor(nh, update_interval)
  , vehicle_name_(vehicle_name)
  , sensor_name_(sensor_name)
  , lidar_pub_(nh.advertise<sensor_msgs::PointCloud2>(vehicle_name + "/lidar/" + sensor_name, 10)) {
}

void Lidar::update() {
  auto lidar_data = getAirSimClient()->getLidarData(sensor_name_, vehicle_name_);
  sensor_msgs::PointCloud2 lidar_msg = convertToPointCloud(lidar_data);
  lidar_pub_.publish(lidar_msg);
}

// https://docs.ros.org/jade/api/sensor_msgs/html/point__cloud__conversion_8h_source.html#l00066
// look at UnrealLidarSensor.cpp UnrealLidarSensor::getPointCloud() for math
// read this carefully https://docs.ros.org/kinetic/api/sensor_msgs/html/msg/PointCloud2.html
sensor_msgs::PointCloud2 Lidar::convertToPointCloud(const msr::airlib::LidarData& lidar_data) const
{
    sensor_msgs::PointCloud2 lidar_msg;
    lidar_msg.header.frame_id = sensor_name_;
    lidar_msg.header.stamp = ros::Time::now();

    if (lidar_data.point_cloud.size() > 3)
    {
        lidar_msg.height = 1;
        lidar_msg.width = lidar_data.point_cloud.size() / 3;

        lidar_msg.fields.resize(3);
        lidar_msg.fields[0].name = "x";
        lidar_msg.fields[1].name = "y";
        lidar_msg.fields[2].name = "z";
        int offset = 0;

        for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4)
        {
            lidar_msg.fields[d].offset = offset;
            lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
            lidar_msg.fields[d].count  = 1;
        }

        lidar_msg.is_bigendian = false;
        lidar_msg.point_step = offset; // 4 * num fields
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

        lidar_msg.is_dense = true; // todo
        std::vector<float> data_std = lidar_data.point_cloud;

        const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
        vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
        lidar_msg.data = std::move(lidar_msg_data);
    }

    return lidar_msg;
}
