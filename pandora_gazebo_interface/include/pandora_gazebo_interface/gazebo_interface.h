/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  Copyright (c) 2013, Open Source Robotics Foundation
*  Copyright (c) 2013, The Johns Hopkins University
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
* Author:  Dave Coleman, Johnathan Bohren , Evangelos Apostolidis
*********************************************************************/
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
#include <gazebo_ros_control/robot_hw_sim.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// pandora_ros_control
#include <xmega_hardware_interface/battery_interface.h>
#include <xmega_hardware_interface/range_sensor_interface.h>
#include <arm_hardware_interface/co2_sensor_interface.h>
#include <arm_hardware_interface/thermal_sensor_interface.h>

// URDF
#include <urdf/model.h>

namespace pandora_gazebo_interface

{

  class GazeboInterface :
    public gazebo_ros_control ::RobotHWSim
    
  {

    public:
    
      ~GazeboInterface ( void ) ; 
      
      bool initSim ( const std ::string & robotnamespace , 
                     ros ::NodeHandle modelNh , 
                     gazebo ::physics ::ModelPtr parentModel , 
                     const urdf ::Model * const urdfModel , 
                     std ::vector < transmission_interface ::TransmissionInfo > 
                      transmissions ) ; 

      void readSim ( ros ::Time time , ros ::Duration period ) ; 

      void writeSim (ros ::Time time , ros ::Duration period ) ; 
  
    private: 
                      
      bool registerInterfaces ( void ) ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
                      
      bool initLinks ( void ) ; 
                      
      bool initIMU ( void ) ; 
    
      // ----------------------------------------------------------------------
                      
      bool initJoints ( void ) ; 
                      
      bool initWheels ( void ) ; 
                      
      bool initSides ( void ) ; 
                      
      bool initLinear ( void ) ; 
                      
      bool initLaser ( void ) ; 
                      
      bool initKinect ( void ) ; 
    
      // ----------------------------------------------------------------------
                      
      bool initXMEGA ( void ) ; 
                      
      bool initBatteries ( void ) ; 
                      
      bool initRangeSensors ( void ) ; 
    
      // ----------------------------------------------------------------------
                      
      bool initARM ( void ) ; 
                      
      bool initCO2Sensors ( void ) ; 
                      
      bool initThermalSensors ( void ) ; 
                      
      bool initMicrophoneSensors ( void ) ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////

      void readLinks ( void ) ; 
    
      // ----------------------------------------------------------------------

      void readJoints ( void ) ; 
    
      // ----------------------------------------------------------------------

      void readXMEGA ( void ) ; 

      void readBatteries ( void ) ; 

      void readRangeSensors ( void ) ; 
    
      // ----------------------------------------------------------------------

      void readARM ( void ) ; 

      void readCO2Sensors ( void ) ; 

      void readThermalSensors ( void ) ; 

      void readMicrophoneSensors ( void ) ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////

      void writeLinks ( void ) ; 

      void writeJoints ( void ) ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      void adjustWheelVelocityCommands ( void ) ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
    
      enum ControlMethod { NONE , 
                           EFFORT , 
                           POSITION , 
                           POSITION_PID , 
                           VELOCITY , 
                           VELOCITY_PID } ;
    
      // ---------------------------------------------------------------------- 
    
      enum RadiationType { ULTRASOUND , 
                           INFRARED } ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
                           
      std ::string robotnamespace_ ; 
      ros ::NodeHandle modelNh_ ; 
      gazebo ::physics ::ModelPtr parentModel_ ; 
      const urdf ::Model * urdfModel_ ; 
      std ::vector < transmission_interface ::TransmissionInfo > 
       transmissions_ ; 
    
      // ----------------------------------------------------------------------
       
      gazebo ::physics ::WorldPtr world_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      gazebo ::common ::Time readTime_ ; 
      ros ::Duration readPeriod_ ; 
    
      // ----------------------------------------------------------------------
      
      gazebo ::common ::Time writeTime_ ; 
      ros ::Duration writePeriod_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      hardware_interface ::ImuSensorHandle ::Data imuSensorData_ ; 
      
      std ::vector 
      < pandora_hardware_interface ::xmega ::BatteryHandle ::Data > 
       batteryData_ ; 
       
      std ::vector 
      < pandora_hardware_interface ::xmega ::RangeSensorHandle ::Data > 
       rangeSensorData_ ; 
      
      std ::vector 
      < pandora_hardware_interface ::arm ::Co2SensorHandle ::Data > 
       co2SensorData_ ; 
       
      std ::vector 
      < pandora_hardware_interface ::arm ::ThermalSensorHandle ::Data > 
       thermalSensorData_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      hardware_interface ::JointStateInterface jointStateInterface_ ; 
      hardware_interface ::PositionJointInterface positionJointInterface_ ; 
      hardware_interface ::VelocityJointInterface velocityJointInterface_ ; 

      hardware_interface ::ImuSensorInterface imuSensorInterface_ ; 
      
      pandora_hardware_interface ::xmega ::BatteryInterface 
       batteryInterface_ ; 
      pandora_hardware_interface ::xmega ::RangeSensorInterface 
       rangeSensorInterface_ ; 
      
      pandora_hardware_interface ::arm ::Co2SensorInterface 
       co2SensorInterface_ ; 
      pandora_hardware_interface ::arm ::ThermalSensorInterface 
       thermalSensorInterface_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      unsigned int linkNum_ ; 
      
      gazebo ::common ::Time linkUpdateRate_ ; 
      gazebo ::common ::Time linkLastUpdateTime_ ; 
      
      std ::vector < gazebo ::physics ::LinkPtr > gazeboLink_ ; 
      std ::vector < std ::string > linkName_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      unsigned int jointNum_ ; 
      
      gazebo ::common ::Time jointUpdateRate_ ; 
      gazebo ::common ::Time jointLastUpdateTime_ ; 
      
      std ::vector < gazebo ::physics ::JointPtr > gazeboJoint_ ; 
      std ::vector < std ::string > jointName_ ; 
      std ::vector < int > jointType_ ; 
      
      std ::vector < ControlMethod > jointControlMethod_ ; 
      std ::vector < control_toolbox ::Pid > pidController_ ; 
      
      std ::vector < double > jointLowerLimit_ ; 
      std ::vector < double > jointUpperLimit_ ; 
      std ::vector < double > jointEffortLimit_ ; 
      
      std ::vector < double > jointEffort_ ; 
      std ::vector < double > jointPosition_ ; 
      std ::vector < double > jointVelocity_ ; 
      
      std ::vector < double > jointCommand_ ; 
      
      double wheelVelocityMultiplier_ ; 
      double wheelRadius_ ; 
      double wheelSeparation_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      double imuOrientation_ [ 4 ] ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      unsigned int batteryNum_ ; 
      
      gazebo ::common ::Time batteryUpdateRate_ ; 
      gazebo ::common ::Time batteryLastUpdateTime_ ; 
      
      std ::vector < std ::string > batteryName_ ; 
      std ::vector < double > batteryVoltage_ ; 
      
      std ::vector < double > batteryVoltageMax_ ; 
      std ::vector < double > batteryVoltageMin_ ; 
      std ::vector < double > batteryDuration_ ; 
    
      // ----------------------------------------------------------------------
      
      unsigned int rangeSensorNum_ ; 
      
      gazebo ::common ::Time rangeSensorUpdateRate_ ; 
      gazebo ::common ::Time rangeSensorLastUpdateTime_ ; 
      
      std ::vector < std ::string > rangeSensorName_ ; 
      std ::vector < std ::string > rangeSensorFrameID_ ; 
      
      std ::vector < int > rangeSensorRadiationType_ ; 
      
      std ::vector < double > rangeSensorFOV_ ; 
      std ::vector < double > rangeSensorMinRange_ ; 
      std ::vector < double > rangeSensorMaxRange_ ; 
      
      std ::vector < std ::vector <  double > > rangeSensorRange_ ; 
      std ::vector < int > rangeSensorBufferCounter_ ; 
      
      std ::vector < gazebo ::sensors ::RaySensorPtr > 
       rangeSensorRay_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
      unsigned int co2SensorNum_ ; 
      
      gazebo ::common ::Time co2SensorUpdateRate_ ; 
      gazebo ::common ::Time co2SensorLastUpdateTime_ ; 
      
      std ::vector < std ::string > co2SensorName_ ; 
      std ::vector < std ::string > co2SensorFrameID_ ; 
      
      std ::vector < float > co2SensorCo2Percentage_ ; 
      
      std ::vector < gazebo ::sensors ::CameraSensorPtr > 
       co2SensorCamera_ ; 
    
      // ----------------------------------------------------------------------
      
      unsigned int thermalSensorNum_ ; 
      
      gazebo ::common ::Time thermalSensorUpdateRate_ ; 
      gazebo ::common ::Time thermalSensorLastUpdateTime_ ; 
      
      std ::vector < std ::string > thermalSensorName_ ; 
      std ::vector < std ::string > thermalSensorFrameID_ ; 
      
      std ::vector < int > thermalSensorHeight_ ; 
      std ::vector < int > thermalSensorWidth_ ; 
      std ::vector < int > thermalSensorStep_ ; 
      
      std ::vector < std ::vector < uint8_t > > thermalSensorVector_ ; 
      
      std ::vector < gazebo ::sensors ::CameraSensorPtr > 
       thermalSensorCamera_ ; 
    
      // ----------------------------------------------------------------------
      
      unsigned int microphoneSensorNum_ ; 
      
      gazebo ::common ::Time microphoneSensorUpdateRate_ ; 
      gazebo ::common ::Time microphoneSensorLastUpdateTime_ ; 
      
      std ::vector < std ::string > microphoneSensorName_ ; 
      std ::vector < std ::string > microphoneSensorFrameID_ ; 
      
      std ::vector < double > microphoneSensorSoundCertainty_ ; 
      
      std ::vector < gazebo ::sensors ::CameraSensorPtr > 
       microphoneSensorCamera_ ; 
    
      /////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////
      
  } ; 
  
}  // namespace pandora_gazebo_interface

PLUGINLIB_EXPORT_CLASS( pandora_gazebo_interface::GazeboInterface,
                        gazebo_ros_control::RobotHWSim
                      )

#endif  // PANDORA_GAZEBO_INTERFACE_GAZEBO_INTERFACE_H
