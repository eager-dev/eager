#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_step_world_plugin/SetInt.h>
#include <std_srvs/Empty.h>

namespace gazebo
{
  /// \brief A Bumper controller
  class GazeboStepWorld : public WorldPlugin
  {
    /// Constructor
    public: GazeboStepWorld();

    /// Destructor
    public: virtual ~GazeboStepWorld();

    /// \brief Load the plugin
    /// \param take in SDF root element
    protected: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		
		private: void QueueThread();
		
		private: physics::WorldPtr world_;
		
		private: ros::ServiceServer srv_;
		
		/// Update the controller
    private: bool stepWorld(gazebo_step_world_plugin::SetInt::Request &req, 
    												gazebo_step_world_plugin::SetInt::Response &res);

    /// \brief pointer to ros node
    private: ros::NodeHandle* nh_;
    
    // Custom Callback Queue
  	private: ros::CallbackQueue queue_;
  	
  	/// \brief Thead object for the running callback Thread.
  	private: boost::thread callback_queue_thread_;

  };
}
