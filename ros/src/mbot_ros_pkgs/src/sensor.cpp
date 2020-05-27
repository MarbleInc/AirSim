
#include "sensor.h"

Sensor::Sensor(ros::NodeHandle nh, double update_interval)
  : update_interval_(update_interval)
  , last_update_timestamp_(0.0) {

}

void Sensor::tick(double timestamp) {
    double dt = timestamp - last_update_timestamp_;
    if (dt >= update_interval_ || last_update_timestamp_ == 0.0) {
        last_update_timestamp_ = timestamp;
        update();
    }
}
