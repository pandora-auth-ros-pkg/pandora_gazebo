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



#include "pandora_gazebo_plugins/pandora_co2_plugin.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PandoraCo2Plugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
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
    gzthrow("PandoraCo2Plugin controller requires a Camera Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Co2 plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Co2 plugin missing <topicName>, defaults to /world");
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
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<pandora_arm_hardware_interface::Co2Msg>(
      this->topic_name_,1,
      boost::bind( &PandoraCo2Plugin::CameraConnect,this),
      boost::bind( &PandoraCo2Plugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
    
    
    ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<sensor_msgs::Image>(
      (this->topic_name_+"/viz/image"),1,
      boost::bind( &PandoraCo2Plugin::CameraConnect,this),
      boost::bind( &PandoraCo2Plugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
    this->pub_viz = this->rosnode_->advertise(ao2);
  // Initialize the controller
  }

  // sensor generation off by default
  this->parent_camera_sensor_->SetActive(false);
  // start custom queue for laser
  this->callback_camera_queue_thread_ = boost::thread( boost::bind( &PandoraCo2Plugin::CameraQueueThread,this ) );
}

// Increment count
void PandoraCo2Plugin::CameraConnect()
{
  this->camera_connect_count_++;
  this->parent_camera_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void PandoraCo2Plugin::CameraDisconnect()
{
  this->camera_connect_count_--;

  if (this->camera_connect_count_ == 0)
    this->parent_camera_sensor_->SetActive(false);
}

void PandoraCo2Plugin::CameraQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PandoraCo2Plugin::OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
    if (last_update_time_ < sensor_update_time)
    {
      this->PutCo2Data(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_co2 topic name not set");
  }
}

void PandoraCo2Plugin:: PutCo2Data ( common:: Time & _updateTime ) { 

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

  int ppm ; 

  int ambientPpm = 1000 ; 

  int maxPpm = 0 ; 
  
  for ( unsigned int i = 0 ; i < width ; i++ ) { 

    for ( unsigned int j = 0 ; j < height ; j++ ) { 
      
      float currentPpm = 0 ; 

      float R = data [ ( ( i * height ) + j ) * 3 + 0 ] ; 
      float G = data [ ( ( i * height ) + j ) * 3 + 1 ] ; 
      float B = data [ ( ( i * height ) + j ) * 3 + 2 ] ; 

      // co2 is represented by green
      float G1 = ( G - R ) ; 
      float G2 = ( G - B ) ; 

      float positiveDiff = 0 ; 

      if ( G1 > 0 ) { 

        currentPpm += pow ( G1 , 2 ) ; 

        ++ positiveDiff ; 
      
      }

      if ( G2 > 0 ) { 

        currentPpm += pow ( G2 , 2 ) ; 

        ++ positiveDiff ; 

      }
      
      currentPpm = sqrt ( currentPpm ) ; 

      if ( positiveDiff == 1 ) 

        currentPpm /= 255.0 ; 

      else if ( positiveDiff == 2 )     

        currentPpm /= sqrt ( pow ( 255.0 , 2 ) 
                             + pow ( 255.0 , 2 ) ) ;

      for ( unsigned int k = 0 ; k < 3 ; k++ ) 

        imgviz 
         .data 
          .push_back ( ( char ) ( currentPpm * 255.0 ) ) ; 

      if ( maxPpm < currentPpm ) 

        maxPpm = currentPpm ; 
      
    }

  }

  // min ppm = ambient = 1000
  // max ppm = ambient + 700 = 1700
  ppm = ambientPpm + ( maxPpm * 700.0 ) ; 

  this ->pub_viz .publish ( imgviz ) ; 

  //----------------------------------------------------------------------

  co2Msg_ .header .stamp = ros:: Time:: now ( ) ; 
  co2Msg_ .header .frame_id = this ->frame_name_ ; 
  
  co2Msg_ .ppm = ppm ; 
  
  // TODO
  // co2Msg_ .co2_percentage = maxPpm ; 
    
  this ->pub_ .publish ( this ->co2Msg_ ) ; 
  
  //----------------------------------------------------------------------
  
  usleep ( 100000 ) ; 

}

}
