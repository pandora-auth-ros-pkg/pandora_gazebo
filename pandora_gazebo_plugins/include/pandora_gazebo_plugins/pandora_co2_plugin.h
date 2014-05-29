
#ifndef PANDORA_CO2_PLUGIN_HH
#define PANDORA_CO2_PLUGIN_HH
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <string>
// boost stuff
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// ros messages stuff
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>

// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "pandora_arm_hardware_interface/Co2Msg.h"

namespace gazebo
{

  class PandoraCo2Plugin : public CameraPlugin
  {
	
	  public: PandoraCo2Plugin(){}
	
	  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
	
	  private: physics::WorldPtr world_;
	
    /// \brief The parent sensor
    private: sensors::SensorPtr parent_sensor_;
    private: sensors::CameraSensorPtr parent_camera_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: ros::Publisher pub_viz;
    private: ros::Publisher camera_info_pub_;
   
    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock;

    /// update rate of this sensor
    private: double update_rate_;
    
    private: int camera_connect_count_;
    private: void CameraConnect();
    private: void CameraDisconnect();

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;
    
    private: common::Time last_update_time_;
    
    private: ros::CallbackQueue camera_queue_;
    private: void CameraQueueThread();
    private: boost::thread callback_camera_queue_thread_;
    
    private: transport::NodePtr node_;
    private: common::Time sim_time_;
    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

    protected: virtual void OnNewFrame(const unsigned char *_image,
                     unsigned int _width, unsigned int _height,
                     unsigned int _depth, const std::string &_format);
                   
    private: void PutCo2Data(common::Time &_updateTime);
    
    private: pandora_arm_hardware_interface::Co2Msg co2Msg_ ;
    
    private: bool publish_msg_ ; 

	};
  GZ_REGISTER_SENSOR_PLUGIN(PandoraCo2Plugin)
}
#endif
