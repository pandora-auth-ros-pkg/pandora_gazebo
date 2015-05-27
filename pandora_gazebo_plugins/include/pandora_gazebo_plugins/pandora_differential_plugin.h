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

#ifndef PANDORA_GAZEBO_PLUGINS_PANDORA_DIFFERENTIAL_PLUGIN_H
#define PANDORA_GAZEBO_PLUGINS_PANDORA_DIFFERENTIAL_PLUGIN_H

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/PubQueue.h>

#include <dynamic_reconfigure/server.h>
#include <pandora_gazebo_plugins/DifferentialConfig.h>

namespace gazebo
{

  class GazeboRosDifferential : public ModelPlugin
  {
    public:
      /// \brief Constructor
      GazeboRosDifferential();

      /// \brief Destructor
      virtual ~GazeboRosDifferential();

      /// \brief Load the controller
      /// \param Node XML config node
      void Load(
          physics::ModelPtr _parent,
          sdf::ElementPtr _sdf);

    protected:
      /// \brief Update the controller
      virtual void UpdateChild();

    private:
      /// \brief Load parameters from sdf
      bool LoadParameters();

      /// \brief Load the controller
      void LoadThread();

      /// \brief Initialize the dynamic reconfigure services
      void LoadReconfigureThread();

      /// \brief Callback when using service
      bool ServiceCallback(
          std_srvs::Empty::Request& req,
          std_srvs::Empty::Response& res);

      /// \brief Callback which reconfigures the variables when the
      ///        dynamic_reconfigure sents a new configuration
      void ConfigCallback(
          pandora_gazebo_plugins::DifferentialConfig& config,
          uint32_t level);

      /// \brief Publishes the joint states to ROS
      void PublishJointStates();

      /// \brief Get the real time update rate of the physics engine
      double GetUpdateRate();

      /// \brief Implements the PID Algorithm
      double PIDAlgorithm(
          double error,
          double& previous_error,
          double& integral,
          double k_p,
          double k_i,
          double k_d,
          double i_clamp_min,
          double i_clamp_max);
      double PIDAlgorithm();

      /// \brief Update the angles of the side joints
      void UpdateAngles();

      /// \brief Add forces at the rear wheels in y and z axis due to
      ///        differential activity
      void AddDifferentialForces();

      /// \brief Add downforces at the wheels due to the side joint damping
      void AddDownforces();

      /// \brief Add force at the base link to correct its angle
      void AddBaseCorrectionForce();

      /// \brief Add force at the side joints to correct base's angle
      void AddSideCorrectionForce();

      /// \brief Add forces at the robot to improve physics
      void AddPhysicsForces();

    private:
      /// \brief Pointer to ros node
      ros::NodeHandle* rosnode_;

      ros::Publisher joint_state_pub_;
      PubQueue <sensor_msgs::JointState>::Ptr joint_state_pub_Queue;
      bool publish_joint_states_;

      // ROS publish multi queue, prevents publish() blocking
      PubMultiQueue pmq;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      ros::ServiceServer srv_;
      ros::CallbackQueue callback_queue_;

      /// \brief A mutex to lock access to fields
      ///        that are used in message callbacks
      boost::mutex lock_;

      /// \brief ROS JointState message
      sensor_msgs::JointState joint_state_msg_;

      /// \brief For setting ROS name space
      std::string robot_namespace_;

      boost::thread deferred_load_thread_;
      unsigned int seed;

      // Dynamic reconfigure
      boost::shared_ptr<boost::thread> reconfigure_thread_;
      boost::shared_ptr <dynamic_reconfigure::Server<pandora_gazebo_plugins::DifferentialConfig> > reconfigure_srv_;
      dynamic_reconfigure::Server <pandora_gazebo_plugins::DifferentialConfig>::CallbackType reconfigure_callback_;

      // Deferred load in case ros is blocking
      sdf::ElementPtr sdf;

      /// \brief The parent World
      physics::WorldPtr world_;

      /// \brief Robot's model
      physics::ModelPtr model_;

      // Robot's links
      physics::LinkPtr base_link_;
      physics::LinkPtr left_front_wheel_link_;
      physics::LinkPtr left_rear_wheel_link_;
      physics::LinkPtr right_front_wheel_link_;
      physics::LinkPtr right_rear_wheel_link_;

      // Robot's joints
      physics::JointPtr left_side_joint_;
      physics::JointPtr right_side_joint_;

      // Joint angles and maximum angle
      double left_angle_;
      double right_angle_;
      double max_angle_;

      // Maximum manual forces and side joint damping
      double max_downforce_;
      double max_differential_force_z_;
      double max_differential_force_y_;
      double side_joint_damping_;

      // PID parameters
      double k_p_;
      double k_i_;
      double k_d_;

      // Stored variables of the PID algorithm
      double previous_error_;
      double integral_;

      // Correction force modifier and correction force
      double correction_force_modifier_;
  };

}  // namespace gazebo
#endif  // PANDORA_GAZEBO_PLUGINS_PANDORA_DIFFERENTIAL_PLUGIN_H
