/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/



#include "pandora_gazebo_plugins/pandora_thermal_plugin.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PandoraThermalPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  CameraPlugin::Load(_parent, _sdf);

  // Get then name of the parent sensor
  this->parent_sensor_ = _parent;
  
  // Get the world name.
  std::string worldName = _parent->GetWorldName();
  this->world_ = physics::get_world(worldName);

  last_update_time_ = this->world_->GetSimTime();

  this->node_ = transport::NodePtr(new transport::Node());
  this->node_->Init(worldName);

  this->parent_camera_sensor_ = boost::dynamic_pointer_cast<sensors::CameraSensor>(this->parent_sensor_);

  if (!this->parent_camera_sensor_)
    gzthrow("PandoraThermalPlugin controller requires a Camera Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Thermal plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Thermal plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
  {
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  this->camera_connect_count_ = 0;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);
  
  // set size of cloud message, starts at 0!! FIXME: not necessary
  //~ this->cloud_msg_.points.clear();
  //~ this->cloud_msg_.channels.clear();
  //~ this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

  if (this->topic_name_ != "")
  {
    // Custom Callback Queue
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<controllers_and_sensors_communications::tpaMsg>(
      this->topic_name_,1,
      boost::bind( &PandoraThermalPlugin::CameraConnect,this),
      boost::bind( &PandoraThermalPlugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
    
    ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<sensor_msgs::Image>(
      (this->topic_name_+"/viz/image"),1,
      boost::bind( &PandoraThermalPlugin::CameraConnect,this),
      boost::bind( &PandoraThermalPlugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
    this->pub_viz = this->rosnode_->advertise(ao2);
    
    ros::AdvertiseOptions cio =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    (this->topic_name_+"/viz/camera_info"), 2,
    boost::bind(&PandoraThermalPlugin::CameraConnect, this),
    boost::bind(&PandoraThermalPlugin::CameraDisconnect, this),
    ros::VoidPtr(), &this->camera_queue_);
    this->camera_info_pub_ = this->rosnode_->advertise(cio);
  }


  // Initialize the controller

  // sensor generation off by default
  this->parent_camera_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_camera_queue_thread_ = boost::thread( boost::bind( &PandoraThermalPlugin::CameraQueueThread,this ) );
}

// Increment count
void PandoraThermalPlugin::CameraConnect()
{
  this->camera_connect_count_++;
  this->parent_camera_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void PandoraThermalPlugin::CameraDisconnect()
{
  this->camera_connect_count_--;

  if (this->camera_connect_count_ == 0)
    this->parent_camera_sensor_->SetActive(false);
}

void PandoraThermalPlugin::CameraQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PandoraThermalPlugin::OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
    if (last_update_time_ < sensor_update_time)
    {
      this->PutThermalData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_thermal topic name not set");
  }
}

void PandoraThermalPlugin:: PutThermalData ( common:: Time & _updateTime ) {   
  
  double hfov = this ->parent_camera_sensor_ 
                      ->GetCamera ( ) 
                       ->GetHFOV ( ) 
                        .Radian ( ) ; 

  int width = this ->parent_camera_sensor_ 
                    ->GetImageWidth ( ) ; 

  int height = this ->parent_camera_sensor_ 
                     ->GetImageHeight ( ) ; 

  const unsigned char * data = this ->parent_camera_sensor_ 
                                     ->GetImageData ( ) ; 
  
  //----------------------------------------------------------------------
  
  sensor_msgs:: Image imgviz ; 

  imgviz .header .stamp = ros:: Time:: now ( ) ; 
  imgviz .header .frame_id = this ->frame_name_ ; 

  imgviz .height = height ; 
  imgviz .width = width ; 
  imgviz .step = width * 3 ; 
  imgviz .encoding = "bgr8" ;  
  
  for ( unsigned int i = 0 ; i < width ; i++ ) { 

    for ( unsigned int j = 0 ; j < height ; j++ ) { 
      
      float currentTemp = 0 ; 

      float R = data [ ( ( i * height ) + j ) * 3 + 0 ] ; 
      float G = data [ ( ( i * height ) + j ) * 3 + 1 ] ; 
      float B = data [ ( ( i * height ) + j ) * 3 + 2 ] ; 

      // temperature is represented by red
      float R1 = ( R - G ) ; 
      float R2 = ( R - B ) ; 

      float positiveDiff = 0 ; 

      if ( R1 > 0 ) { 

        currentTemp += pow ( R1 , 2 ) ; 

        ++ positiveDiff ; 
      
      }

      if ( R2 > 0 ) { 

        currentTemp += pow ( R2 , 2 ) ; 

        ++ positiveDiff ; 

      }
      
      currentTemp = sqrt ( currentTemp ) ; 

      if ( positiveDiff == 1 ) 

        currentTemp /= 255.0 ; 

      else if ( positiveDiff == 2 )     

        currentTemp /= sqrt ( pow ( 255.0 , 2 ) 
                              + pow ( 255.0 , 2 ) ) ; 

      for ( unsigned int k = 0 ; k < 3 ; k++ ) 

        imgviz 
         .data 
          .push_back ( ( char ) ( currentTemp * 255.0 ) ) ; 
      
    }

  }

  this -> pub_viz .publish ( imgviz ) ; 

  //----------------------------------------------------------------------
  
  int msgWidth = 8 ; 

  int msgHeight = 8 ; 

  int ambientTemp = 25.0 ; 

  long * pixelTemp ; 

  pixelTemp = new long [ msgWidth ] ; 

  for ( unsigned int i = 0 ; i < msgWidth ; i++ ) 

    pixelTemp [ i ] = 0 ;
  
  for ( unsigned int i = 0 ; i < msgWidth ; i++ ) { 

    float maxTemp = 0 ; 

    for ( unsigned int j = 0 ; j < msgHeight ; j++ ) { 
      
      float currentTemp = 0 ; 

      float R = data [ ( ( i * msgHeight ) + j ) * 3 + 0 ] ; 
      float G = data [ ( ( i * msgHeight ) + j ) * 3 + 1 ] ; 
      float B = data [ ( ( i * msgHeight ) + j ) * 3 + 2 ] ; 

      // temperature is represented by red
      float R1 = ( R - G ) ; 
      float R2 = ( R - B ) ; 

      float positiveDiff = 0 ; 

      if ( R1 > 0 ) { 

        currentTemp += pow ( R1 , 2 ) ; 

        ++ positiveDiff ; 
      
      }

      if ( R2 > 0 ) { 

        currentTemp += pow ( R2 , 2 ) ; 

        ++ positiveDiff ; 

      }
      
      currentTemp = sqrt ( currentTemp ) ; 

      if ( positiveDiff == 1 ) 

        currentTemp /= 255.0 ; 

      else if ( positiveDiff == 2 )     

        currentTemp /= sqrt ( pow ( 255.0 , 2 ) 
                              + pow ( 255.0 , 2 ) ) ; 

      if ( maxTemp < currentTemp ) 

        maxTemp = currentTemp ; 
      
    }

    // min temperature = ambient = 25
    // max temperature = ambient + 15 = 40
    pixelTemp [ i ] = ambientTemp + ( maxTemp * 15.0 ) ; 

  }

  tmsg .header .stamp = ros:: Time:: now ( ) ; 
  tmsg .ambientTemp = ambientTemp ; 

  for ( int i = 0 ; i < msgWidth ; i++ ) 
  
    tmsg .pixelTemp [ i ] = pixelTemp [ i ] ; 
    
  this ->pub_ .publish ( this ->tmsg ) ; 
  
  //----------------------------------------------------------------------
  
  usleep ( 100000 ) ; 

}

}
