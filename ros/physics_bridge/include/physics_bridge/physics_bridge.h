
#pragma once

#include <string>
#include "ros/ros.h"

#include "ros_env/Register.h"
#include "ros_env/StepEnv.h"
#include "ros_env/ResetEnv.h"
#include "ros_env/CloseEnv.h"

class PhysicsBridge {
public:
    PhysicsBridge() : name("physics_bridge") {
        register_service = nh.advertiseService(name + "/register", &PhysicsBridge::register_objects, this);
        step_service = nh.advertiseService(name + "/step", &PhysicsBridge::step, this);
        reset_service = nh.advertiseService(name + "/reset", &PhysicsBridge::reset, this);
        close_service = nh.advertiseService(name + "/close", &PhysicsBridge::close, this);
    }

protected:
    ros::NodeHandle nh;
    std::string name;

private:

    virtual bool register_objects(ros_env::Register::Request &request, ros_env::Register::Response &response) = 0;

    virtual bool step(ros_env::StepEnv::Request &request, ros_env::StepEnv::Response &response) = 0;

    virtual bool reset(ros_env::ResetEnv::Request &request, ros_env::ResetEnv::Response &response) = 0;

    virtual bool close(ros_env::CloseEnv::Request &request, ros_env::CloseEnv::Response &response) = 0;

    ros::ServiceServer register_service;
    ros::ServiceServer step_service;
    ros::ServiceServer reset_service;
    ros::ServiceServer close_service;

};