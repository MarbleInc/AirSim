#include <signal.h>
#include <memory>

#include <ros/ros.h>
#include "mbot_sim.h"
#include <mbot_base/TrackedObject.h>
#include <mbot_base/TrackedObjectArray.h>

class Handler {
    public:
        Handler(shared_ptr<MbotSim> mbot_sim_){
            mbot_sim = mbot_sim_;
        }

        void myHandler(int sig){
            vector<shared_ptr<mbot_base::TrackedObjectArray>> arr = mbot_sim->getTrackedObjectArray();
        //MbotSim mbot_sim, int sig

        }

    private:
     shared_ptr<MbotSim> mbot_sim;


};


int main(int argc, char** argv) {
    ros::init(argc, argv, "mbot_sim_node");
    shared_ptr<MbotSim> mbot_sim = std::make_shared<MbotSim>();
    Handler h(mbot_sim);
    signal(SIGINT, h.myHandler);
    mbot_sim->start();

    while (ros::ok()) {
        // Blocks for step duration
        mbot_sim->step(0.01);
    }

    return 0;
}
