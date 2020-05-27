
#include "gps.h"
#include "airsim_client_factory.h"

Gps::Gps(
  ros::NodeHandle& nh,
  const std::string& vehicle_name,
  const std::string& gps_name,
  double update_interval)
  : Sensor(nh, update_interval)
  , vehicle_name_(vehicle_name)
  , gps_name_(gps_name)
  , airsim_client_(AirSimClientFactory::instance()->getClient()) {
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>(vehicle_name + "/global_gps", 10);
}

void Gps::update() {
  auto gps_data = airsim_client_->getGpsData(gps_name_, vehicle_name_);
  auto nav_sat_msg = convertToNavSatFix(gps_data.gnss.geo_point, gps_data.time_stamp / 1e9);
  gps_pub_.publish(nav_sat_msg);
}

sensor_msgs::NavSatFix Gps::convertToNavSatFix(const msr::airlib::GeoPoint& geo_point, double timestamp) const {
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.stamp = ros::Time(timestamp);
    gps_msg.latitude = geo_point.latitude;
    gps_msg.longitude = geo_point.longitude;
    gps_msg.altitude = geo_point.altitude;
    return gps_msg;
}
