#include "physics_bridge/physics_bridge.h"

#include "ros_env/BoxSpace.h"

class WeBotsBridge : PhysicsBridge {

private:

    bool register_objects(ros_env::Register::Request &request, ros_env::Register::Response &response) {

    }

    bool step(ros_env::StepEnv::Request &request, ros_env::StepEnv::Response &response) {

    }

    bool reset(ros_env::ResetEnv::Request &request, ros_env::ResetEnv::Response &response) {
        
    }

    bool close(ros_env::CloseEnv::Request &request, ros_env::CloseEnv::Response &response) {
        
    }
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "physics_bridge");

    WeBotsBridge pb;

    ros::spin();
}