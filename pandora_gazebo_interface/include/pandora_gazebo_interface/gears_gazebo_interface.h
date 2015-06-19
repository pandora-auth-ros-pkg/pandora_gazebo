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

#ifndef PANDORA_GAZEBO_INTERFACE_GEARS_GAZEBO_INTERFACE_H
#define PANDORA_GAZEBO_INTERFACE_GEARS_GAZEBO_INTERFACE_H

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/plugins/RayPlugin.hh>

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
#include <arm_hardware_interface/co2_sensor_interface.h>
#include <arm_hardware_interface/thermal_sensor_interface.h>
#include <xmega_hardware_interface/battery_interface.h>
#include <xmega_hardware_interface/range_sensor_interface.h>

// URDF
#include <urdf/model.h>

// Messages
#include <sensor_msgs/Range.h>
#include <pandora_sensor_msgs/Co2Msg.h>
#include <sensor_msgs/Image.h>

namespace pandora_gazebo_interface
{

  class GearsGazeboInterface : public gazebo_ros_control::RobotHWSim
  {
    public:
      ~GearsGazeboInterface();

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
      void registerArmInterface();
      void registerXmegaInterface();

      void readJoints();
      void readImu();
      void readArm();
      void readXmega();

      void writeJoints();

      void adjustWheelVelocityCommands();

      void rangeSensorCallback(const sensor_msgs::RangeConstPtr& msg);
      void co2SensorCallback(const pandora_sensor_msgs::Co2MsgConstPtr& msg);
      void thermalSensorCallback(const sensor_msgs::ImageConstPtr& msg);

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

      // Gazebo plugin sensors subscribers
      ros::Subscriber rangeSensorSubscriber_;
      ros::Subscriber co2SensorSubscriber_;
      ros::Subscriber thermalSensorSubscriber_;

      // Physical properties
      double wheelRadius_;
      double wheelSeparation_;

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

      // CO2 sensors
      unsigned int co2SensorNum_;
      std::vector<std::string> co2SensorName_;
      std::vector<std::string> co2SensorFrameID_;
      std::vector<float> co2SensorCo2PercentageStored_;
      std::vector<float> co2SensorCo2Percentage_;
      std::vector<pandora_hardware_interface::arm::Co2SensorHandle::Data> co2SensorData_;
      pandora_hardware_interface::arm::Co2SensorInterface co2SensorInterface_;
      gazebo::common::Time co2SensorReadRate_;
      gazebo::common::Time co2SensorLastReadTime_;

      // Thermal sensors
      unsigned int thermalSensorNum_;
      std::vector<std::string> thermalSensorName_;
      std::vector<std::string> thermalSensorFrameID_;
      std::vector<int> thermalSensorHeight_;
      std::vector<int> thermalSensorWidth_;
      std::vector<int> thermalSensorStep_;
      std::vector<std::vector<uint8_t> > thermalSensorVectorStored_;
      std::vector<std::vector<uint8_t> > thermalSensorVector_;
      std::vector<pandora_hardware_interface::arm::ThermalSensorHandle::Data> thermalSensorData_;
      pandora_hardware_interface::arm::ThermalSensorInterface thermalSensorInterface_;
      gazebo::common::Time thermalSensorReadRate_;
      gazebo::common::Time thermalSensorLastReadTime_;

      // Battery sensors
      unsigned int batteryNum_;
      std::vector<std::string> batteryName_;
      std::vector<double> batteryVoltage_;
      std::vector<double> batteryVoltageMax_;
      std::vector<double> batteryVoltageMin_;
      std::vector<double> batteryDuration_;
      std::vector<pandora_hardware_interface::xmega::BatteryHandle::Data> batteryData_;
      pandora_hardware_interface::xmega::BatteryInterface batteryInterface_;
      gazebo::common::Time batteryReadRate_;
      gazebo::common::Time batteryLastReadTime_;

      // Range sensors
      unsigned int rangeSensorNum_;
      std::vector<std::string> rangeSensorName_;
      std::vector<std::string> rangeSensorFrameID_;
      std::vector<int> rangeSensorRadiationType_;
      std::vector<double> rangeSensorFOV_;
      std::vector<double> rangeSensorMinRange_;
      std::vector<double> rangeSensorMaxRange_;
      std::vector<double> rangeSensorRangeStored_;
      std::vector<std::vector<double> > rangeSensorRange_;
      std::vector<int> rangeSensorBufferCounter_;
      std::vector<pandora_hardware_interface::xmega::RangeSensorHandle::Data> rangeSensorData_;
      pandora_hardware_interface::xmega::RangeSensorInterface rangeSensorInterface_;
      gazebo::common::Time rangeSensorReadRate_;
      gazebo::common::Time rangeSensorLastReadTime_;
  };
}  // namespace pandora_gazebo_interface

PLUGINLIB_EXPORT_CLASS(pandora_gazebo_interface::GearsGazeboInterface,
                       gazebo_ros_control::RobotHWSim
                      )

#endif  // PANDORA_GAZEBO_INTERFACE_GEARS_GAZEBO_INTERFACE_H
