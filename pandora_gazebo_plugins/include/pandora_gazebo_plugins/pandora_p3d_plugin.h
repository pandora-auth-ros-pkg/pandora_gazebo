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

#ifndef PANDORA_GAZEBO_PLUGINS_PANDORA_P3D_PLUGIN_H
#define PANDORA_GAZEBO_PLUGINS_PANDORA_P3D_PLUGIN_H

#include <string>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
class GazeboRosP3D : public ModelPlugin
{
  /// \brief Constructor
public:
  GazeboRosP3D();

  /// \brief Destructor
public:
  virtual ~GazeboRosP3D();

  /// \brief Load the controller
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Update the controller
protected:
  virtual void UpdateChild();

private:
  physics::WorldPtr world_;
private:
  physics::ModelPtr model_;

  /// \brief The parent Model
private:
  physics::LinkPtr link_;

  /// \brief The body of the frame to display pose, twist
private:
  physics::LinkPtr reference_link_;


  /// \brief pointer to ros node
private:
  ros::NodeHandle* rosnode_;
private:
  ros::Publisher pub_;
private:
  PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

  /// \brief ros message
private:
  nav_msgs::Odometry pose_msg_;

private:
  tf::TransformBroadcaster* transform_broadcaster_;

  /// \brief store bodyname
private:
  std::string link_name_;

  /// \brief topic name
private:
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  /// FIXME: extract link name directly?
private:
  std::string frame_name_;
private:
  std::string tf_frame_name_;

  /// \brief allow specifying constant xyz and rpy offsets
private:
  math::Pose offset_;

  /// \brief mutex to lock access to fields used in message callbacks
private:
  boost::mutex lock;

  /// \brief save last_time
private:
  common::Time last_time_;
private:
  math::Vector3 last_vpos_;
private:
  math::Vector3 last_veul_;
private:
  math::Vector3 apos_;
private:
  math::Vector3 aeul_;
private:
  math::Vector3 last_frame_vpos_;
private:
  math::Vector3 last_frame_veul_;
private:
  math::Vector3 frame_apos_;
private:
  math::Vector3 frame_aeul_;

  // rate control
private:
  double update_rate_;

  /// \brief Gaussian noise
private:
  double gaussian_noise_;

  /// \brief Gaussian noise generator
private:
  double GaussianKernel(double mu, double sigma);

  /// \brief for setting ROS name space
private:
  std::string robot_namespace_;

private:
  bool broadcast_tf_;

private:
  ros::CallbackQueue p3d_queue_;
private:
  void P3DQueueThread();
private:
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
private:
  event::ConnectionPtr update_connection_;

private:
  unsigned int seed;

  // ros publish multi queue, prevents publish() blocking
private:
  PubMultiQueue pmq;
};
}  // namespace gazebo
#endif  // PANDORA_GAZEBO_PLUGINS_PANDORA_P3D_PLUGIN_H
