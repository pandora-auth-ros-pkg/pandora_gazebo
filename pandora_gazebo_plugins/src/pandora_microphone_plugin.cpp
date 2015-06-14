/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 *
 * Author: Geromichalos Dimitrios Panagiotis <geromidg@gmail.com>
 *********************************************************************/

#include "pandora_gazebo_plugins/pandora_microphone_plugin.h"

namespace gazebo
{

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
    {
      gzthrow("PandoraMicrophonePlugin controller requires a Camera Sensor as its parent");
    }

    this->robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
    {
      this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

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
    // this->cloud_msg_.points.clear();
    // this->cloud_msg_.channels.clear();
    // this->cloud_msg_.channels.push_back(sensor_msgs::ChannelFloat32());

    if (this->topic_name_ != "")
    {
      // Custom Callback Queue
      ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
                                   this->topic_name_, 1,
                                   boost::bind(&PandoraMicrophonePlugin::CameraConnect, this),
                                   boost::bind(&PandoraMicrophonePlugin::CameraDisconnect, this),
                                   ros::VoidPtr(), &this->camera_queue_);
      this->pub_ = this->rosnode_->advertise(ao);
    }

    // sensor generation off by default
    this->parent_camera_sensor_->SetActive(false);

    // start custom queue for laser
    this->callback_camera_queue_thread_ = boost::thread(boost::bind(&PandoraMicrophonePlugin::CameraQueueThread, this));
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
    {
      this->parent_camera_sensor_->SetActive(false);
    }
  }

  void PandoraMicrophonePlugin::CameraQueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
      this->camera_queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void PandoraMicrophonePlugin::OnNewFrame(const unsigned char* _image,
      unsigned int _width, unsigned int _height,
      unsigned int _depth, const std::string& _format)
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

  void PandoraMicrophonePlugin::PutMicrophoneData(common::Time& _updateTime)
  {
    // soundMsg_.header.stamp = ros::Time::now ( );
    // soundMsg_.header.frame_id = this->frame_name_;

    unsigned int width = this->parent_camera_sensor_
                        ->GetImageWidth();

    unsigned int height = this->parent_camera_sensor_
                         ->GetImageHeight();

    const unsigned char* data = this->parent_camera_sensor_
                                ->GetImageData();

    if (data == NULL)
    {
      return;
    }

    double totalCert = 0.0;

    for (unsigned int i = 0; i < width; i++)
    {
      for (unsigned int j = 0; j < height; j++)
      {
        double currentCert = 0;

        double R = data [((i * height) + j) * 3 + 0 ];
        double G = data [((i * height) + j) * 3 + 1 ];
        double B = data [((i * height) + j) * 3 + 2 ];

        // sound is represented by blue
        double B1 = (B - R);
        double B2 = (B - G);

        double positiveDiff = 0;

        if (B1 > 0)
        {
          currentCert += pow(B1, 2);

          ++positiveDiff;
        }

        if (B2 > 0)
        {
          currentCert += pow(B2, 2);

          ++positiveDiff;
        }

        currentCert = sqrt(currentCert);

        if (positiveDiff == 1)
        {
          currentCert /= 255.0;
        }
        else if (positiveDiff == 2)
        {
          currentCert /= sqrt(pow(255.0, 2)
                              + pow(255.0, 2));
        }

        totalCert += currentCert;
      }
    }

    double certainty = totalCert / (width * height);
    // soundMsg_.certainty = certainty;

    // Sound detection condition
    if (certainty > 0.5)
    {
      soundMsg_.data = true;
    }
    else
    {
      soundMsg_.data = false;
    }

    this->pub_.publish(this->soundMsg_);

    usleep(100000);
  }
}  // namespace gazebo
