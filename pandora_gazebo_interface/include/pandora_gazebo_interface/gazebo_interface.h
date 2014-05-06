/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  Evangelos Apostolidis
*********************************************************************/
#ifndef PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H
#define PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H

#include <time.h>
#include "ros/ros.h"
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo_ros_control/robot_hw_sim.h>

namespace pandora_gazebo_interface
{
  class GazeboInterface :
    public gazebo_ros_control::RobotHWSim
  {
    private:
      ros::NodeHandle nodeHandle_;
      hardware_interface::ImuSensorInterface imuSensorInterface_;
      hardware_interface::ImuSensorHandle::Data imuData_;
      double imuOrientation[4];

      hardware_interface::JointStateInterface jointStateInterface_;
      hardware_interface::VelocityJointInterface velocityJointInterface_;
      hardware_interface::PositionJointInterface positionJointInterface_;
      std::vector<std::string> jointNames_;
      double jointCommand_[8];
      double jointPosition_[8];
      double jointVelocity_[8];
      double jointEffort_[8];

      std::vector<gazebo::physics::JointPtr> gazeboJoints_;
      gazebo::physics::LinkPtr gazeboLink_;

      std::vector<std::string> getJointNameFromParamServer();
      void registerInterfaces();

    public:
      ~GazeboInterface();
      bool initSim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);

      void readSim(ros::Time time, ros::Duration period);

      void writeSim(ros::Time time, ros::Duration period);
  };
}  // namespace pandora_gazebo_interface

PLUGINLIB_EXPORT_CLASS
(
  pandora_gazebo_interface::GazeboInterface,
  gazebo_ros_control::RobotHWSim)

#endif  // PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H







