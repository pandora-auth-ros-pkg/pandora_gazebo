
#ifndef PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H
#define PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H

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
#include <xmega_hardware_interface/battery_interface.h>
#include <xmega_hardware_interface/range_sensor_interface.h>
#include <arm_hardware_interface/co2_sensor_interface.h>
#include <arm_hardware_interface/thermal_sensor_interface.h>

// URDF
#include <urdf/model.h>

namespace pandora_gazebo_interface
{

  class GazeboInterface : public gazebo_ros_control::RobotHWSim
  {
    public:
      ~GazeboInterface();

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
      bool registerInterfaces();

      bool initLinks();
      bool initIMU();
      bool initJoints();
      bool initWheels();
      bool initSides();
      bool initLinear();
      bool initLaser();
      bool initKinect();
      bool initXMEGA();
      bool initBatteries();
      bool initRangeSensors();
      bool initARM();
      bool initCO2Sensors();
      bool initThermalSensors();
      bool initMicrophoneSensors();

      void readLinks();
      void readJoints();
      void readXMEGA();
      void readBatteries();
      void readRangeSensors();
      void readARM();
      void readCO2Sensors();
      void readThermalSensors();
      void readMicrophoneSensors();

      void writeLinks();
      void writeJoints();

      void adjustWheelVelocityCommands();

      enum ControlMethod
      {
        NONE,
        EFFORT,
        POSITION,
        POSITION_PID,
        VELOCITY,
        VELOCITY_PID
      };

      enum RadiationType
      {
        ULTRASOUND,
        INFRARED
      };

      std::string robotnamespace_;
      ros::NodeHandle modelNh_;
      gazebo::physics::ModelPtr parentModel_;
      const urdf::Model* urdfModel_;
      std::vector<transmission_interface::TransmissionInfo> transmissions_;

      gazebo::physics::WorldPtr world_;

      gazebo::common::Time readTime_;
      ros::Duration readPeriod_;
      gazebo::common::Time writeTime_;
      ros::Duration writePeriod_;

      hardware_interface::ImuSensorHandle::Data imuSensorData_;
      pandora_hardware_interface::imu::ImuRPYHandle::Data imuRPYData_;
      std::vector<pandora_hardware_interface::xmega::BatteryHandle::Data> batteryData_;
      std::vector<pandora_hardware_interface::xmega::RangeSensorHandle::Data> rangeSensorData_;
      std::vector<pandora_hardware_interface::arm::Co2SensorHandle::Data> co2SensorData_;
      std::vector<pandora_hardware_interface::arm::ThermalSensorHandle::Data> thermalSensorData_;

      hardware_interface::JointStateInterface jointStateInterface_;
      hardware_interface::PositionJointInterface positionJointInterface_;
      hardware_interface::VelocityJointInterface velocityJointInterface_;
      hardware_interface::ImuSensorInterface imuSensorInterface_;
      pandora_hardware_interface::imu::ImuRPYInterface imuRPYInterface_;
      pandora_hardware_interface::xmega::BatteryInterface batteryInterface_;
      pandora_hardware_interface::xmega::RangeSensorInterface rangeSensorInterface_;
      pandora_hardware_interface::arm::Co2SensorInterface co2SensorInterface_;
      pandora_hardware_interface::arm::ThermalSensorInterface thermalSensorInterface_;

      unsigned int linkNum_;
      gazebo::common::Time linkUpdateRate_;
      gazebo::common::Time linkLastUpdateTime_;
      std::vector<gazebo::physics::LinkPtr> gazeboLink_;
      std::vector<std::string> linkName_;

      unsigned int jointNum_;
      gazebo::common::Time jointUpdateRate_;
      gazebo::common::Time jointLastUpdateTime_;
      std::vector<gazebo::physics::JointPtr> gazeboJoint_;
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

      double wheelVelocityMultiplier_;
      double wheelRadius_;
      double wheelSeparation_;

      double imuOrientation_[4];
      double* imuRoll_;
      double* imuPitch_;
      double* imuYaw_;

      unsigned int batteryNum_;
      gazebo::common::Time batteryUpdateRate_;
      gazebo::common::Time batteryLastUpdateTime_;
      std::vector<std::string> batteryName_;
      std::vector<double> batteryVoltage_;
      std::vector<double> batteryVoltageMax_;
      std::vector<double> batteryVoltageMin_;
      std::vector<double> batteryDuration_;

      unsigned int rangeSensorNum_;
      gazebo::common::Time rangeSensorUpdateRate_;
      gazebo::common::Time rangeSensorLastUpdateTime_;
      std::vector<std::string> rangeSensorName_;
      std::vector<std::string> rangeSensorFrameID_;
      std::vector<int> rangeSensorRadiationType_;
      std::vector<double> rangeSensorFOV_;
      std::vector<double> rangeSensorMinRange_;
      std::vector<double> rangeSensorMaxRange_;
      std::vector<std::vector<double> > rangeSensorRange_;
      std::vector<int> rangeSensorBufferCounter_;
      std::vector<gazebo::sensors::RaySensorPtr> rangeSensorRay_;

      unsigned int co2SensorNum_;
      gazebo::common::Time co2SensorUpdateRate_;
      gazebo::common::Time co2SensorLastUpdateTime_;
      std::vector<std::string> co2SensorName_;
      std::vector<std::string> co2SensorFrameID_;
      std::vector<float> co2SensorCo2Percentage_;
      std::vector<gazebo::sensors::CameraSensorPtr>co2SensorCamera_;

      unsigned int thermalSensorNum_;
      gazebo::common::Time thermalSensorUpdateRate_;
      gazebo::common::Time thermalSensorLastUpdateTime_;
      std::vector<std::string> thermalSensorName_;
      std::vector<std::string> thermalSensorFrameID_;
      std::vector<int> thermalSensorHeight_;
      std::vector<int> thermalSensorWidth_;
      std::vector<int> thermalSensorStep_;
      std::vector<std::vector<uint8_t> > thermalSensorVector_;
      std::vector<gazebo::sensors::CameraSensorPtr> thermalSensorCamera_;

      unsigned int microphoneSensorNum_;
      gazebo::common::Time microphoneSensorUpdateRate_;
      gazebo::common::Time microphoneSensorLastUpdateTime_;
      std::vector<std::string> microphoneSensorName_;
      std::vector<std::string> microphoneSensorFrameID_;
      std::vector<double> microphoneSensorSoundCertainty_;
      std::vector<gazebo::sensors::CameraSensorPtr> microphoneSensorCamera_;
  };
}  // namespace pandora_gazebo_interface

PLUGINLIB_EXPORT_CLASS(pandora_gazebo_interface::GazeboInterface,
                       gazebo_ros_control::RobotHWSim
                      )

#endif  // PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H
