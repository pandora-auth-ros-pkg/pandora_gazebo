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



#include "pandora_gazebo_plugins/pandora_microphone_plugin.h"

namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void PandoraMicrophonePlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
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
    gzthrow("PandoraMicrophonePlugin controller requires a Camera Sensor as its parent");

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("frameName"))
  {
    ROS_INFO("Microphone plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_INFO("Microphone plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
  {
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("publishMsg"))
  {
    ROS_INFO("Microphone plugin missing <publishMsg>, defaults to true");
    this->publish_msg_ = true;
  }
  else
  {
    this->publish_msg_ = _sdf ->Get < std ::string > ( "publishMsg" ) == "true" ; 
  }

  if (!_sdf->HasElement("publishViz"))
  {
    ROS_INFO("Microphone plugin missing <publishViz>, defaults to true");
    this->publish_viz_ = true;
  }
  else
  {
    this->publish_viz_ = _sdf ->Get < std ::string > ( "publishViz" ) == "true" ; 
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
  
    if ( this->publish_msg_ ) { 
    
      // Custom Callback Queue
      ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
        this->topic_name_,1,
        boost::bind( &PandoraMicrophonePlugin::CameraConnect,this),
        boost::bind( &PandoraMicrophonePlugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
      this->pub_ = this->rosnode_->advertise(ao);
    
    }
  
    if ( this->publish_viz_ ) { 
    
      ros::AdvertiseOptions ao2 = ros::AdvertiseOptions::create<sensor_msgs::Image>(
        (this->topic_name_+"/viz/image/"+this->frame_name_),1,
        boost::bind( &PandoraMicrophonePlugin::CameraConnect,this),
        boost::bind( &PandoraMicrophonePlugin::CameraDisconnect,this), ros::VoidPtr(), &this->camera_queue_);
      this->pub_viz = this->rosnode_->advertise(ao2);
    
    }
    
  }
  
  if ( this->publish_msg_ ) 
  
    // sensor generation off by default
    this->parent_camera_sensor_->SetActive(false);

  // start custom queue for laser
  this->callback_camera_queue_thread_ = boost::thread( boost::bind( &PandoraMicrophonePlugin::CameraQueueThread,this ) );
}

// Increment count
void PandoraMicrophonePlugin::CameraConnect()
{
  this->camera_connect_count_++;
  this->parent_camera_sensor_->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void PandoraMicrophonePlugin::CameraDisconnect()
{
  this->camera_connect_count_--;

  if (this->camera_connect_count_ == 0)
    this->parent_camera_sensor_->SetActive(false);
}

void PandoraMicrophonePlugin::CameraQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->camera_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void PandoraMicrophonePlugin::OnNewFrame(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
{
  if (this->topic_name_ != "")
  {
    common::Time sensor_update_time = this->parent_sensor_->GetLastUpdateTime();
    if (last_update_time_ < sensor_update_time)
    {
      this->PutMicrophoneData(sensor_update_time);
      last_update_time_ = sensor_update_time;
    }
  }
  else
  {
    ROS_INFO("gazebo_ros_Microphone topic name not set");
  }
}

void PandoraMicrophonePlugin ::PutMicrophoneData ( common:: Time & _updateTime ) 

{ 
  
  if ( this->publish_msg_ || this->publish_viz_ ) { 

    unsigned int width = this ->parent_camera_sensor_ 
                      ->GetImageWidth ( ) ; 

    unsigned int height = this ->parent_camera_sensor_ 
                       ->GetImageHeight ( ) ; 

    const unsigned char * data = this ->parent_camera_sensor_ 
                                       ->GetImageData ( ) ; 
      
    if ( data == NULL ) 
    
      return ; 
    
    //----------------------------------------------------------------------
    
    if ( this->publish_viz_ ) { 

      imgviz_ .header .stamp = ros:: Time:: now ( ) ; 
      imgviz_ .header .frame_id = this ->frame_name_ ; 

      imgviz_ .height = height ; 
      imgviz_ .width = width ; 
      imgviz_ .step = width ; 
      imgviz_ .encoding = "mono8" ; 
      
      imgviz_ .data .clear ( ) ; 
    
    }
    
    //----------------------------------------------------------------------

    double totalCert = 0.0 ; 
    
    for ( unsigned int i = 0 ; i < width ; i++ ) { 

      for ( unsigned int j = 0 ; j < height ; j++ ) { 
        
        double currentCert = 0 ; 

        double R = data [ ( ( i * height ) + j ) * 3 + 0 ] ; 
        double G = data [ ( ( i * height ) + j ) * 3 + 1 ] ; 
        double B = data [ ( ( i * height ) + j ) * 3 + 2 ] ; 

        // sound is represented by blue
        double B1 = ( B - R ) ; 
        double B2 = ( B - G ) ; 

        double positiveDiff = 0 ; 

        if ( B1 > 0 ) { 

          currentCert += pow ( B1 , 2 ) ; 

          ++ positiveDiff ; 
        
        }

        if ( B2 > 0 ) { 

          currentCert += pow ( B2 , 2 ) ; 

          ++ positiveDiff ; 

        }
        
        currentCert = sqrt ( currentCert ) ; 

        if ( positiveDiff == 1 ) 

          currentCert /= 255.0 ; 

        else if ( positiveDiff == 2 )     

          currentCert /= sqrt ( pow ( 255.0 , 2 ) 
                                + pow ( 255.0 , 2 ) ) ; 
    
        if ( this->publish_viz_ ) 

          imgviz_ .data .push_back ( ( char ) ( currentCert  * 255.0 ) ) ; 

        totalCert += currentCert ; 
        
      }

    }
    
    double certainty = totalCert / ( width * height ) ; 
    
    //----------------------------------------------------------------------
    
    if ( this->publish_viz_ ) 

      this ->pub_viz .publish ( imgviz_ ) ; 

    //----------------------------------------------------------------------
    
    if ( this->publish_msg_ ) { 
      
      //soundMsg_ .header .stamp = ros:: Time:: now ( ) ; 
      //soundMsg_ .header .frame_id = this ->frame_name_ ; 

      // Sound detection condition
      if ( certainty > 0.5 ) 

        soundMsg_ .data = true ; 

      else 

        soundMsg_ .data = false ; 
      
      //soundMsg_ .certainty = certainty ; 
        
      this ->pub_ .publish ( this ->soundMsg_ ) ; 
      
    }
  
  }
  
  //----------------------------------------------------------------------
  
  usleep ( 100000 ) ; 

}

}
