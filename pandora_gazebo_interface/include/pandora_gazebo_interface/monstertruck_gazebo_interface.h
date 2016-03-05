/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, P.A.N.D.O.R.A. Team.
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
 * Author: George Kouros <gkourosg@yahoo.gr>
 *********************************************************************/

#ifndef PANDORA_GAZEBO_INTERFACE_MONSTERTRUCK_GAZEBO_INTERFACE_H
#define PANDORA_GAZEBO_INTERFACE_MONSTERTRUCK_GAZEBO_INTERFACE_H

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// pandora_ros_control
#include <imu_hardware_interface/imu_rpy_interface.h>

// URDF
#include <urdf/model.h>

namespace pandora_gazebo_interface
{

  class MonstertruckGazeboInterface : public gazebo_ros_control::RobotHWSim
  {
    public:
      ~MonstertruckGazeboInterface();

      bool initSim(
          const std::string& robotnamespace,
          ros::NodeHandle modelNh,
          gazebo::physics::ModelPtr parentModel,
          const urdf::Model* const urdfModel,
          std::vector<transmission_interface::TransmissionInfo> transmissions);

      void readSim(
          ros::Time time,
          ros::Duration period);

      void writeSim(
          ros::Time time,
          ros::Duration period);

    private:
      void registerJointInterface();
      void registerImuInterface();

      void readJoints();
      void readImu();

      void writeJoints();

      enum ControlMethod
      {
        NONE,
        EFFORT,
        POSITION,
        POSITION_PID,
        VELOCITY,
        VELOCITY_PID
      };

      // Sim arguments
      std::string robotnamespace_;
      ros::NodeHandle modelNh_;
      gazebo::physics::ModelPtr parentModel_;
      const urdf::Model* urdfModel_;
      std::vector<transmission_interface::TransmissionInfo> transmissions_;
      gazebo::physics::WorldPtr world_;

      // Physical properties
      double wheelRadius_;
      double wheelbase_;
      double wheelTrack_;

      // Read and write times and rates
      gazebo::common::Time readTime_;
      ros::Duration readPeriod_;
      gazebo::common::Time writeTime_;
      ros::Duration writePeriod_;

      // Joints
      unsigned int jointNum_;
      std::vector<std::string> jointName_;
      std::vector<int> jointType_;
      std::vector<ControlMethod> jointControlMethod_;
      std::vector<control_toolbox::Pid> pidController_;
      std::vector<double> jointLowerLimit_;
      std::vector<double> jointUpperLimit_;
      std::vector<double> jointEffortLimit_;
      std::vector<double> jointEffort_;
      std::vector<double> jointPosition_;
      std::vector<double> jointVelocity_;
      std::vector<double> jointCommand_;
      hardware_interface::JointStateInterface jointStateInterface_;
      hardware_interface::PositionJointInterface positionJointInterface_;
      hardware_interface::VelocityJointInterface velocityJointInterface_;
      hardware_interface::EffortJointInterface effortJointInterface_;
      gazebo::common::Time jointReadRate_;
      gazebo::common::Time jointLastReadTime_;
      gazebo::common::Time jointWriteRate_;
      gazebo::common::Time jointLastWriteTime_;
      std::vector<gazebo::physics::JointPtr> gazeboJoint_;

      // Imu sensor
      std::string imuLinkName_;
      double imuOrientation_[4];
      hardware_interface::ImuSensorHandle::Data imuSensorData_;
      hardware_interface::ImuSensorInterface imuSensorInterface_;
      double* imuRoll_;
      double* imuPitch_;
      double* imuYaw_;
      pandora_hardware_interface::imu::ImuRPYHandle::Data imuRPYData_;
      pandora_hardware_interface::imu::ImuRPYInterface imuRPYInterface_;
      gazebo::common::Time imuReadRate_;
      gazebo::common::Time imuLastReadTime_;
      gazebo::physics::LinkPtr gazeboImuLink_;
  };
}  // namespace pandora_gazebo_interface

PLUGINLIB_EXPORT_CLASS(pandora_gazebo_interface::MonstertruckGazeboInterface,
                       gazebo_ros_control::RobotHWSim)
#endif  // PANDORA_GAZEBO_INTERFACE_MONSTERTRUCK_GAZEBO_INTERFACE_H
