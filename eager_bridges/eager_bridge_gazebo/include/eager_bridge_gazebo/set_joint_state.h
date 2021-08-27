#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <eager_bridge_gazebo/SetJointState.h>

namespace gazebo
{
  /// \brief A joint state controller
  class GazeboSetJointState : public WorldPlugin
  {
    /// Constructor
    public: GazeboSetJointState();

    /// Destructor
    public: virtual ~GazeboSetJointState();

    /// \brief Load the plugin
    /// \param take in SDF root element
    protected: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		
		private: void QueueThread();
		
		private: physics::WorldPtr world_;
		
		private: ros::ServiceServer srv_;
		
		/// Update the controller
    private: bool setJointState(eager_bridge_gazebo::SetJointState::Request &req, 
    														eager_bridge_gazebo::SetJointState::Response &res);

    /// \brief pointer to ros node
    private: ros::NodeHandle* nh_;
    
    // Custom Callback Queue
  	private: ros::CallbackQueue queue_;
  	
  	/// \brief Thead object for the running callback Thread.
  	private: boost::thread callback_queue_thread_;

  };
}
