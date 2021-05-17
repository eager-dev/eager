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
 
 /** Modifications:
 *  \author Jelle Luijkx
 *  \desc   Step World Plugin for Gazebo
 */
 
#include <gazebo/common/Plugin.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo_step_world_plugin/step_world.h>
#include <ros/ros.h>
#include <gazebo_step_world_plugin/SetInt.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboStepWorld);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboStepWorld::GazeboStepWorld() : WorldPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboStepWorld::~GazeboStepWorld()
{
	queue_.clear();
  queue_.disable();
	nh_->shutdown();
	callback_queue_thread_.join();
	
	delete nh_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboStepWorld::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("step_world", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  world_ = _world;
  
 	nh_ =  new ros::NodeHandle("~");
  nh_->advertiseService<gazebo_step_world_plugin::SetInt::Request, gazebo_step_world_plugin::SetInt::Response>("step_world", boost::bind(&GazeboStepWorld::stepWorld, this, _1, _2));
  
  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<gazebo_step_world_plugin::SetInt>("step_world", 
    																																				boost::bind(&GazeboStepWorld::stepWorld,this, _1, _2), 
    																																				ros::VoidPtr(), 
    																																				&queue_);
  srv_ = nh_->advertiseService(aso);
  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboStepWorld::QueueThread,this ) );
  ROS_INFO_NAMED("step_world","Loaded gazebo_step_world_plugin.");
}

bool GazeboStepWorld::stepWorld(gazebo_step_world_plugin::SetInt::Request &req, gazebo_step_world_plugin::SetInt::Response &res)
{
	if (!world_->IsPaused())
	{
		ROS_WARN_NAMED("step_world", "Cannot step, pause physics first.");
		return true;
	}
	ROS_INFO_NAMED("step_world", "Stepping %d iterations.", req.data);
	world_->Step(req.data);
	res.success = true;
  return true;
}


// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboStepWorld::QueueThread()
{
  static const double timeout = 0.01;

  while (this->nh_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
