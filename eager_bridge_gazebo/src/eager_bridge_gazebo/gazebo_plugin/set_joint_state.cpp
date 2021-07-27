/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

  /** Partially copied from:
  *  https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp
  */

 /** Modifications:
 *  \author Jelle Luijkx
 *  \desc   Set Joint State Plugin for Gazebo
 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <eager_bridge_gazebo/set_joint_state.h>
#include <ros/ros.h>
#include <eager_bridge_gazebo/SetJointState.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboSetJointState);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboSetJointState::GazeboSetJointState() : WorldPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboSetJointState::~GazeboSetJointState()
{
	queue_.clear();
  queue_.disable();
	nh_->shutdown();
	callback_queue_thread_.join();

	delete nh_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboSetJointState::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("set_joint_state", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  world_ = _world;

 	nh_ =  new ros::NodeHandle("~");

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<eager_bridge_gazebo::SetJointState>("set_joint_state",
    																																				boost::bind(&GazeboSetJointState::setJointState,this, _1, _2),
    																																				ros::VoidPtr(),
    																																				&queue_);
  srv_ = nh_->advertiseService(aso);
  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboSetJointState::QueueThread,this ) );
  ROS_INFO_NAMED("set_joint_state","Loaded gazebo_set_joint_state_plugin.");
}

bool GazeboSetJointState::setJointState(eager_bridge_gazebo::SetJointState::Request &req, eager_bridge_gazebo::SetJointState::Response &res)
{
  std::string gazebo_model_name = req.model_name;

  // search for model with name
  #if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr gazebo_model = world_->ModelByName(req.model_name);
  #else
  gazebo::physics::ModelPtr gazebo_model = world_->GetModel(req.model_name);
  #endif

  if (!gazebo_model)
  {
    ROS_ERROR_NAMED("set_joint_state", "SetJointState: model [%s] does not exist",gazebo_model_name.c_str());
    res.success = false;
    res.status_message = "SetJointState: model does not exist";
    return true;
  }

  if (req.joint_names.size() == req.joint_positions.size() || req.joint_names.size() == req.joint_velocities.size())
  {
		// make the service call to pause gazebo
		bool is_paused = world_->IsPaused();
		if (!is_paused) world_->SetPaused(true);

		if (req.joint_names.size() == req.joint_positions.size())
		{
			std::map<std::string, double> joint_position_map;
			for (unsigned int i = 0; i < req.joint_names.size(); i++)
			{
				joint_position_map[req.joint_names[i]] = req.joint_positions[i];
			}
			gazebo_model->SetJointPositions(joint_position_map);
		}

		if (req.joint_names.size() == req.joint_velocities.size())
		{
			for (unsigned int i = 0; i < req.joint_names.size(); i++)
			{
				gazebo_model->GetJoint(req.joint_names[i])->SetVelocity(0, req.joint_velocities[i]);
			}
		}

    // resume paused state before this call
    world_->SetPaused(is_paused);

    res.success = true;
    res.status_message = "SetJointState: success";
    return true;
  }
  else
  {
    res.success = false;
    res.status_message = "SetJointState: joint name and position list or velocity list have different lengths";
    return true;
  }
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboSetJointState::QueueThread()
{
  static const double timeout = 0.01;

  while (this->nh_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
