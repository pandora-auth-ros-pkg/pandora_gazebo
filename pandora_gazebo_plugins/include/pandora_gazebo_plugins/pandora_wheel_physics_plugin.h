#ifndef GAZEBO_ROS_WHEEL_PHYSICS_HH
#define GAZEBO_ROS_WHEEL_PHYSICS_HH


#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <dynamic_reconfigure/server.h>
#include <pandora_gazebo_plugins/WheelPhysicsConfig.h>

namespace gazebo
{
  class PhysicsReconfigure : public ModelPlugin
  {

  	/*#################    Class Variables    #################*/
  	private:

	  	// Pointer to the model
	  	physics::ModelPtr model_;
	  	sdf::ElementPtr sdf;

      //Robot Namespace : Used for topics
      std::string robot_namespace_;

	  	//Robot Wheels (to  be initialized with Proper Function)
			physics ::LinkPtr left_front_wheel_link_ ;
	    physics ::LinkPtr left_rear_wheel_link_ ; 
	    physics ::LinkPtr right_front_wheel_link_ ; 
	    physics ::LinkPtr right_rear_wheel_link_ ;


      //Robot Wheels Parameters  
      physics ::SurfaceParamsPtr left_front_wheel_params_ ;
      physics ::SurfaceParamsPtr left_rear_wheel_params_ ;
      physics ::SurfaceParamsPtr right_front_wheel_params_ ;
      physics ::SurfaceParamsPtr right_rear_wheel_params_ ;

      //Dynamic Reconfigure Attributes:
      dynamic_reconfigure::Server<pandora_gazebo_plugins::WheelPhysicsConfig> *reconfig_server;
      dynamic_reconfigure::Server<pandora_gazebo_plugins::WheelPhysicsConfig>::CallbackType f;


  	/*#################    Class Methods    #################*/
  	public:
      PhysicsReconfigure(){};
  		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  		bool LoadParameters();
      void reconfigCallback(pandora_gazebo_plugins::WheelPhysicsConfig &config, uint32_t level);
  	
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PhysicsReconfigure)
}


#endif