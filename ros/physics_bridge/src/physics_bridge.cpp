#include "physics_bridge/physics_bridge.h"

#include <string>
#include "ros_env/BoxSpace.h"

class WeBotsBridge : PhysicsBridge {
public:

    WeBotsBridge() {
        
    }

private:

    bool register_objects(ros_env::Register::Request &request, ros_env::Register::Response &response) {

        for (const auto &sensor : request.sensors) {
            ROS_INFO("Sensor: %s", &sensor[0]);
            //TODO: Enable and subscribe to sensor in WeBots
            obs_services.push_back(nh.advertiseService(name + "/objects/" + sensor, &WeBotsBridge::handle_obs, this));
        }
        for (const auto &actuator : request.actuators) {
            ROS_INFO("Actuator: %s", &actuator[0]);
            act_services.push_back(nh.serviceClient<ros_env::BoxSpace>(name + "/objects/" + actuator));
        }

        return true;
    }

    bool step(ros_env::StepEnv::Request &request, ros_env::StepEnv::Response &response) {
        return true;
    }

    bool reset(ros_env::ResetEnv::Request &request, ros_env::ResetEnv::Response &response) {
        return true;
    }

    bool close(ros_env::CloseEnv::Request &request, ros_env::CloseEnv::Response &response) {
        return true;
    }

    bool handle_obs(ros_env::BoxSpace::Request &request, ros_env::BoxSpace::Response &response) {
        response.value = std::vector<float>(7, 0.0);
        return true;
    }

    std::vector<ros::ServiceServer> obs_services;
    std::vector<ros::ServiceClient> act_services;
};


int main(int argc, char **argv) {

    ros::init(argc, argv, "physics_bridge");

    WeBotsBridge pb;

    ros::spin();
}