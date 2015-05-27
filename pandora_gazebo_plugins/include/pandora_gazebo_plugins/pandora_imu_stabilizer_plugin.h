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

#ifndef PANDORA_GAZEBO_PLUGINS_PANDORA_IMU_STABILIZER_PLUGIN_H
#define PANDORA_GAZEBO_PLUGINS_PANDORA_IMU_STABILIZER_PLUGIN_H

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
class GazeboRosIMU : public ModelPlugin
{
  /// \brief Constructor
public:
  GazeboRosIMU();

  /// \brief Destructor
public:
  virtual ~GazeboRosIMU();

  /// \brief Load the controller
  /// \param node XML config node
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Update the controller
protected:
  virtual void UpdateChild();

  /// \brief The parent World
private:
  physics::WorldPtr world_;

  /// \brief The link referred to by this plugin
private:
  physics::LinkPtr link;

  /// \brief pointer to ros node
private:
  ros::NodeHandle* rosnode_;
private:
  ros::Publisher pub_;
private:
  ros::Publisher joint_state_pub_;
private:
  PubQueue<sensor_msgs::Imu>::Ptr pub_Queue;
private:
  PubQueue<sensor_msgs::JointState>::Ptr joint_state_pub_Queue;

  /// \brief ros Imu message
private:
  sensor_msgs::Imu imu_msg_;

  /// \brief ros JointState message
private:
  sensor_msgs::JointState joint_state_msg_;

  /// \brief store link name
private:
  std::string link_name_;

  /// \brief topic name
private:
  std::string topic_name_;

  /// \brief allow specifying constant xyz and rpy offsets
private:
  math::Pose offset_;

  /// \brief A mutex to lock access to fields
  /// that are used in message callbacks
private:
  boost::mutex lock_;

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

  /// \brief: keep initial pose to offset orientation in imu message
private:
  math::Pose initial_pose_;

  /// \brief Gaussian noise
private:
  double gaussian_noise_;

  /// \brief Gaussian noise generator
private:
  double GaussianKernel(double mu, double sigma);

  /// \brief for setting ROS name space
private:
  std::string robot_namespace_;

  /// \brief call back when using service
private:
  bool ServiceCallback(std_srvs::Empty::Request& req,
                       std_srvs::Empty::Response& res);

private:
  ros::ServiceServer srv_;
private:
  std::string service_name_;

private:
  ros::CallbackQueue imu_queue_;
private:
  void IMUQueueThread();
private:
  boost::thread callback_queue_thread_;

  // Pointer to the update event connection
private:
  event::ConnectionPtr update_connection_;

  // deferred load in case ros is blocking
private:
  sdf::ElementPtr sdf;
private:
  void LoadThread();
private:
  boost::thread deferred_load_thread_;
private:
  unsigned int seed;

  // ros publish multi queue, prevents publish() blocking
private:
  PubMultiQueue pmq;

private:
  gazebo::physics::JointPtr jointRoll_;
private:
  gazebo::physics::JointPtr jointPitch_;
private:
  gazebo::physics::ModelPtr model_;
};
}  // namespace gazebo
#endif  // PANDORA_GAZEBO_PLUGINS_PANDORA_IMU_STABILIZER_PLUGIN_H
