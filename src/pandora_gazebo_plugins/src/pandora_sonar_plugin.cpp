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
/*
   Desc: GazeboRosDepthCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <pandora_sonar_plugin.h>

#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>


#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosDepthCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosDepthCamera::GazeboRosDepthCamera()
{
  this->depth_info_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosDepthCamera::~GazeboRosDepthCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDepthCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  DepthCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;
  
  ROS_ERROR("Depth: %d", this->depth_);

//~   // using a different default
//~   if (!_sdf->GetElement("imageTopicName"))
//~     this->image_topic_name_ = "ir/image_raw";
//~   if (!_sdf->HasElement("cameraInfoTopicName"))
//~     this->camera_info_topic_name_ = "ir/camera_info";


  // depth image stuff
  if (!_sdf->GetElement("topicName"))
    this->depth_image_topic_name_ = "/sonar";
  else
    this->depth_image_topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

//~   if (!_sdf->GetElement("depthImageCameraInfoTopicName"))
//~     this->depth_image_camera_info_topic_name_ = "depth/camera_info";
//~   else
//~     this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();


  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboRosDepthCamera::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboRosDepthCamera::Advertise()
{
  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Range >(
      this->depth_image_topic_name_,1,
      boost::bind( &GazeboRosDepthCamera::DepthImageConnect,this),
      boost::bind( &GazeboRosDepthCamera::DepthImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

}


////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosDepthCamera::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosDepthCamera::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
  if (this->parentSensor->IsActive())
  {
    if (this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
        this->FillDepthImage(_image);
    }
  }
  else
  {
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewRGBPointCloud(const float *_pcd,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDepthCamera::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  //ROS_ERROR("camera_ new frame %s %s",this->parentSensor_->GetName().c_str(),this->frame_name_.c_str());
  this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
      this->PutCameraData(_image);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboRosDepthCamera::FillDepthImage(const float *_src)
{
  this->lock_.lock();
  // copy data into image
  
  this->sonar_msg_.header.frame_id = this->frame_name_;
  this->sonar_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->sonar_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
  this->sonar_msg_.field_of_view = this->parentSensor->GetDepthCamera()->GetHFOV().Radian();

  ///copy from depth to depth image message
  FillDepthImageHelper(this->sonar_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->depth_image_pub_.publish(this->sonar_msg_);

  this->lock_.unlock();
}

// Fill depth information
bool GazeboRosDepthCamera::FillDepthImageHelper(
    sensor_msgs::Range& sonar_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  sonar_msg.range = toCopyFrom[index];

  // convert depth to point cloud
  for (uint32_t j = 0; j < rows_arg; j++)
  {
    for (uint32_t i = 0; i < cols_arg; i++)
    {
      float depth = toCopyFrom[index++];
      
      if (sonar_msg.range > toCopyFrom[index++]) {
		  sonar_msg.range = toCopyFrom[index];
	  }
    }
  }
  return true;
}

}
