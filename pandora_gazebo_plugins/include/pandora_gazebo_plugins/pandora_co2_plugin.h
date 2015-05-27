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

#ifndef PANDORA_GAZEBO_PLUGINS_PANDORA_CO2_PLUGIN_H
#define PANDORA_GAZEBO_PLUGINS_PANDORA_CO2_PLUGIN_H

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

#include "pandora_sensor_msgs/Co2Msg.h"

namespace gazebo
{

class PandoraCo2Plugin : public CameraPlugin
{
public:
  PandoraCo2Plugin() {}

public:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  physics::WorldPtr world_;

  /// \brief The parent sensor
private:
  sensors::SensorPtr parent_sensor_;
private:
  sensors::CameraSensorPtr parent_camera_sensor_;

  /// \brief pointer to ros node
private:
  ros::NodeHandle* rosnode_;
private:
  ros::Publisher pub_;
private:
  ros::Publisher pub_viz;
private:
  ros::Publisher camera_info_pub_;

  /// \brief topic name
private:
  std::string topic_name_;

  /// \brief frame transform name, should match link name
private:
  std::string frame_name_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
private:
  boost::mutex lock;

  /// update rate of this sensor
private:
  double update_rate_;

private:
  int camera_connect_count_;
private:
  void CameraConnect();
private:
  void CameraDisconnect();

  /// \brief for setting ROS name space
private:
  std::string robot_namespace_;

private:
  common::Time last_update_time_;

private:
  ros::CallbackQueue camera_queue_;
private:
  void CameraQueueThread();
private:
  boost::thread callback_camera_queue_thread_;

private:
  transport::NodePtr node_;
private:
  common::Time sim_time_;
public:
  void OnStats(const boost::shared_ptr<msgs::WorldStatistics const>& _msg);

protected:
  virtual void OnNewFrame(const unsigned char* _image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string& _format);

private:
  void PutCo2Data(common::Time& _updateTime);

private:
  sensor_msgs::Image imgviz_;

private:
  pandora_sensor_msgs::Co2Msg co2Msg_;

private:
  bool publish_msg_;
private:
  bool publish_viz_;
};
GZ_REGISTER_SENSOR_PLUGIN(PandoraCo2Plugin)
}  // namespace gazebo
#endif  // PANDORA_GAZEBO_PLUGINS_PANDORA_CO2_PLUGIN_H
