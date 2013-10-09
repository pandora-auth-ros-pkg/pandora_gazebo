#include <boost/bind.hpp>
#include <stdio.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorManager.hh"
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/gzmath.hh>
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

static float Vlin=0.0, Vang=0.0; // set as global 

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {   
    public: ROSModelPlugin()
    {
      // Start up ROS
      std::string name1 = "gazebo"; // this is what appears in the rostopics
      int argc = 0;
      ros::init(argc, NULL, name1);      
    }
    public: ~ROSModelPlugin()
    {
      delete this->nh;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

	// ROS Nodehandle
      this->nh = new ros::NodeHandle("~");
      // ROS Subscriber
      this->subvel = this->nh->subscribe<geometry_msgs::Twist>("cmd_vel", 1000, &ROSModelPlugin::ROSCallback_Vel, this );

	  this->nh = new ros::NodeHandle("~");
      this->pubpose = this->nh->advertise<geometry_msgs::PoseStamped>("pose",1000);

	 printf("parent name: %s\n",_parent->GetName().c_str());

//***********************************************

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // for gazebo 1.4 or lower it should be ConnectWorldUpdateStart
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
            static double qw,qx,qy,qz, Rrad, Prad, Yrad;

// ************* QUATERNION / POSE DATA ******************
      math::Vector3 p = model->GetWorldPose().pos;
      math::Quaternion r = model->GetWorldPose().rot;
      //from quaternion to Roll Pitch Yaw, in radianos
      qw=r.w;   qx=r.x;     qy=r.y;     qz=r.z;
      Rrad=atan2(  2*(qw*qx+qy*qz),  1-2*(qx*qx+qy*qy)  );  //Roll
      Prad=asin(2*(qw*qy-qz*qx));               //Pitch
      Yrad=atan2(  2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz)  );  //Yaw

// ***************************************************
            ros::Time current_time;
            current_time = ros::Time::now();
            
            tf::Vector3 translation(p.x, p.y, p.z);
            tf::Quaternion rotation(r.x, r.y, r.z, r.w);
            
            tf::Transform transform(rotation, translation);
            
//~             _poseFrameBroadcaster.sendTransform(tf::StampedTransform(transform, current_time,"world", "base_link"));
            
            
// ******************************************
        //Pub Pose msg
            geometry_msgs::Point p2ros;
            p2ros.x=p.x; p2ros.y=p.y; p2ros.z=p.z;

            geometry_msgs::Quaternion r2ros;
            r2ros.x=r.x; r2ros.y=r.y; r2ros.z=r.z; r2ros.w=r.w;

            geometry_msgs::PoseStamped Pose2ros;
            Pose2ros.header.stamp = current_time;//ros::Time::now();
            Pose2ros.header.frame_id="base_link";
            Pose2ros.pose.position=p2ros;
            Pose2ros.pose.orientation=r2ros;

            this->pubpose.publish(Pose2ros);

// *********************************************** 
            //set velocities
            float velx,vely;
            velx=Vlin*cos(Yrad);
            vely=Vlin*sin(Yrad);    
            
            //~ if(Vlin||Vang){
				//~ ROS_INFO("Setting velx = %f , vely = %f . vang = %f",velx,vely,Vang);
			//~ }
            
            this->model->SetLinearVel(math::Vector3(velx, vely, 0));
            this->model->SetAngularVel(math::Vector3(0, 0, Vang));

// ***********************************************
            ros::spinOnce();
    }


   // callback functions run every time data is published to the topic
    void ROSCallback_Vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
            Vlin=msg->linear.x;
            Vang=msg->angular.z;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    private: sensors::RaySensorPtr raysensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* nh;
    // ROS Subscriber
    ros::Subscriber subvel;

        // ROS Publisher
        ros::Publisher pubpose;
        
        //*************
        
        tf::TransformBroadcaster _poseFrameBroadcaster;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
