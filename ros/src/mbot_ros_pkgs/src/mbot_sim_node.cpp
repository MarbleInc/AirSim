#include <signal.h>

#include <ros/ros.h>
#include "mbot_sim.h"





int main(int argc, char** argv) {
    ros::init(argc, argv, "mbot_sim_node");
    MbotSim mbot_sim;
    mbot_sim.start();

    while (ros::ok()) {
        // Blocks for step duration
        mbot_sim.step(0.01);
    }

    return 0;
}
