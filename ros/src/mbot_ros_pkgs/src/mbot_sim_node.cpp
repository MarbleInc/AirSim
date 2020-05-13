
#include <ros/ros.h>
#include "mbot_sim.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbot_sim_node");
    ros::MultiThreadedSpinner spinner(20);

    MbotSim mbot_sim;
    mbot_sim.start();
    spinner.spin();

    return 0;
}
