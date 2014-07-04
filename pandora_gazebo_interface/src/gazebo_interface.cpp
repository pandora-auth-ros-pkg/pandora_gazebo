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
#include "pandora_gazebo_interface/gazebo_interface.h"

namespace
{

  double clamp ( const double val , 
                 const double min_val , 
                 const double max_val) {
                 
    return std ::min ( std ::max ( val , min_val ) , max_val ) ; 
     
  }

}

namespace pandora_gazebo_interface

{

  GazeboInterface ::~GazeboInterface ( ) { 
  
    
  
  }

  bool GazeboInterface ::initSim ( const std ::string & robotnamespace , 
                                   ros ::NodeHandle modelNh , 
                                   gazebo ::physics ::ModelPtr parentModel , 
                                   const urdf ::Model * const urdfModel , 
                                   std ::vector 
                                   < transmission_interface ::TransmissionInfo > 
                                    transmissions ) { 
    
    // ------------------------------------------------------------------------
                                    
    robotnamespace_ = robotnamespace ; 
    modelNh_ = modelNh ; 
    parentModel_ = parentModel ; 
    urdfModel_ = urdfModel ; 
    transmissions_ = transmissions ; 
    
    // ------------------------------------------------------------------------
    
    world_ = parentModel_ ->GetWorld ( ) ; 
    
    // ------------------------------------------------------------------------
  
    if ( ! initLinks ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize links . "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initJoints ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize joints. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initXMEGA ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize XMEGA. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initARM ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize ARM. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! registerInterfaces ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to register interfaces. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    ROS_INFO ( "pandora_gazebo_interface initialized successfully!" ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readSim ( ros ::Time time , ros ::Duration period ) { 
    
    // ------------------------------------------------------------------------
  
    readTime_ = gazebo ::common ::Time ( time .sec , time .nsec ) ; 
    readPeriod_ = period ; 
    
    // ------------------------------------------------------------------------
                  
    readLinks ( ) ; 
    
    readJoints ( ) ; 
    
    readXMEGA ( ) ; 
    
    readARM ( ) ; 
    
    // ------------------------------------------------------------------------
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::writeSim ( ros ::Time time , ros ::Duration period ) { 
    
    // ------------------------------------------------------------------------
  
    writeTime_ = gazebo ::common ::Time ( time .sec , time .nsec ) ; 
    writePeriod_ = period ; 
    
    // ------------------------------------------------------------------------
  
    writeLinks ( ) ; 
  
    writeJoints ( ) ; 
    
    // ------------------------------------------------------------------------
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initLinks ( void ) { 
    
    // ------------------------------------------------------------------------
  
    // Number of links
    linkNum_ = 1 ; //FIXME
    
    linkUpdateRate_ = gazebo ::common ::Time ( 1 / 50.0 ) ; //FIXME
    linkLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    // ------------------------------------------------------------------------
    
    // Resize vectors
    gazeboLink_ .resize ( linkNum_ ) ; 
    
    linkName_ .resize ( linkNum_ ) ; 
    
    // ------------------------------------------------------------------------
    
    // Initialize link data
    if ( ! initIMU ( ) ) { 
    
      ROS_FATAL ( "Could not initialize IMU sensor." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------

    // Load gazebo links
    for ( unsigned int i = 0 ; i < linkNum_ ; i ++ ) 
    
      gazeboLink_ [ i ] = parentModel_ ->GetLink ( linkName_ [ i ] ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initIMU ( void ) { 
    
    // ------------------------------------------------------------------------
    
    linkName_ [ 0 ] = "base_link" ; //FIXME
    
    // ------------------------------------------------------------------------
    
    imuSensorData_ .name= "/sensors/imu" ; //FIXME
    imuSensorData_ .frame_id = linkName_ [ 0 ] ; 
    
    // ------------------------------------------------------------------------
    
    imuOrientation_ [ 0 ] = 0 ; //FIXME
    imuOrientation_ [ 1 ] = 0 ; //FIXME
    imuOrientation_ [ 2 ] = 0 ; //FIXME
    imuOrientation_ [ 3 ] = 1 ; //FIXME
    
    imuSensorData_. orientation = imuOrientation_ ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initJoints ( void ) { 
    
    // ------------------------------------------------------------------------
  
    // Number of joints
    jointNum_ = 13 ; //FIXME
    //jointNum_ = transmissions .size ( ) ; 
    
    jointUpdateRate_ = gazebo ::common ::Time ( 1 / 50.0 ) ; //FIXME
    jointLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    // ------------------------------------------------------------------------
    
    // Resize vectors
    gazeboJoint_ .resize ( jointNum_ ) ; 
    
    jointName_ . resize ( jointNum_ ) ; 
    jointType_ . resize ( jointNum_ ) ; 
    
    jointControlMethod_ . resize ( jointNum_ ) ; 
    pidController_ . resize ( jointNum_ ) ; 
    
    jointLowerLimit_ . resize ( jointNum_ ) ; 
    jointUpperLimit_ . resize ( jointNum_ ) ; 
    jointEffortLimit_ . resize ( jointNum_ ) ; 
    
    jointEffort_ . resize ( jointNum_ ) ; 
    jointPosition_ . resize ( jointNum_ ) ; 
    jointVelocity_ . resize ( jointNum_ ) ; 
    
    jointCommand_ . resize ( jointNum_ ) ; 
    
    wheelVelocityMultiplier_ = 1.0 ; //FIXME
    wheelRadius_ = 0.09794 ; //FIXME
    wheelSeparation_ = 0.344 ; //FIXME
    
    // ------------------------------------------------------------------------
    
    // Initialize link data
    if ( ! initWheels ( ) ) { 
    
      ROS_FATAL ( "Could not initialize wheels." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initSides ( ) ) { 
    
      ROS_FATAL ( "Could not initialize sides." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initLinear ( ) ) { 
    
      ROS_FATAL ( "Could not initialize linear." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initLaser ( ) ) { 
    
      ROS_FATAL ( "Could not initialize laser." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initKinect ( ) ) { 
    
      ROS_FATAL ( "Could not initialize kinect." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------

    // Load gazebo joints
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) 
    
      gazeboJoint_ [ i ] = parentModel_ ->GetJoint ( jointName_ [ i ] ) ; 
    
    // ------------------------------------------------------------------------
      
    // Load PID controllers and initialize / set the limits.
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      if ( jointControlMethod_ [ i ] == POSITION_PID ) { 
      
        /*
      
        const ros ::NodeHandle nh ( modelNh_ , robotnamespace_ + 
                                               "/gazebo_ros_control/pid_gains/" + 
                                               jointName_ [ i ] ) ; 
        
        pidController_ [ i ] .init ( nh ) ; 
        
        */
      
      }
    
      if ( jointControlMethod_ [ i ] == VELOCITY ) 

        gazeboJoint_ [ i ] ->SetMaxForce ( 0 , jointEffortLimit_ [ i ] ) ; 
        
    }
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initWheels ( void ) { 
    
    // ------------------------------------------------------------------------
    
    jointName_ [ 0 ] = "left_front_wheel_joint" ; //FIXME
    jointName_ [ 1 ] = "left_rear_wheel_joint" ; //FIXME
    jointName_ [ 2 ] = "right_front_wheel_joint" ; //FIXME
    jointName_ [ 3 ] = "right_rear_wheel_joint" ; //FIXME
    
    // ------------------------------------------------------------------------
    
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
    
      jointType_ [ i ] = urdf ::Joint ::CONTINUOUS ; //FIXME
      
      jointEffort_ [ i ] = 0.0 ; 
      jointPosition_ [ i ] = 0.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      
      jointCommand_ [ i ] = 0.0 ; 
      
      jointEffortLimit_ [ i ] = 100.0 ; //FIXME
    
      jointControlMethod_ [ i ] = VELOCITY ; //FIXME
    
    }
    
    // ------------------------------------------------------------------------
    
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initSides ( void ) { 
    
    // ------------------------------------------------------------------------
    
    jointName_ [ 4 ] = "left_side_joint" ; //FIXME
    jointName_ [ 5 ] = "right_side_joint" ; //FIXME
    
    // ------------------------------------------------------------------------
    
    for ( unsigned int i = 4 ; i < 6 ; i ++ ) { 
    
      jointType_ [ i ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
      jointEffort_ [ i ] = 0.0 ; 
      jointPosition_ [ i ] = 0.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      
      jointCommand_ [ i ] = 0.0 ; 
      
      jointLowerLimit_ [ i ] = - 0.785 ; //FIXME
      jointUpperLimit_ [ i ] = 0.785 ; //FIXME
      
      jointEffortLimit_ [ i ] = 150.0 ; //FIXME
      //jointEffortLimit_ [ i ] = 0.0 ; 
      
      jointControlMethod_ [ i ] = NONE ; //FIXME
    
    }
    
    // ------------------------------------------------------------------------
    
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initLinear ( void ) { 
    
    // ------------------------------------------------------------------------
        
    // Elevator
    
    jointName_ [ 6 ] = "linear_elevator_joint" ; //FIXME
    
    jointType_ [ 6 ] = urdf ::Joint ::PRISMATIC ;//FIXME
     
    jointEffort_ [ 6 ] = 0.0 ; 
    jointPosition_ [ 6 ] = 0.0 ; 
    jointVelocity_ [ 6 ] = 0.0 ; 
    
    jointCommand_ [ 6 ] = 0.0 ; 
    
    jointLowerLimit_ [ 6 ] = 0.0 ; //FIXME
    jointUpperLimit_ [ 6 ] = 0.18 ; //FIXME
    
    jointEffortLimit_ [ 6 ] = 100.0 ; //FIXME
    //jointEffortLimit_ [ 6 ] = 15.0 ; 
    
    jointControlMethod_ [ 6 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 6 ] .initPid ( 15000.0 , 0.0 , 0.0 , 0.0 , 0.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
        
    // Head Pitch
    
    jointName_ [ 7 ] = "linear_head_pitch_joint" ; //FIXME
    
    jointType_ [ 7 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 7 ] = 0.0 ; 
    jointPosition_ [ 7 ] = 0.0 ; 
    jointVelocity_ [ 7 ] = 0.0 ; 
    
    jointCommand_ [ 7 ] = 0.0 ; 
    
    jointLowerLimit_ [ 7 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 7 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 7 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 7 ] = 8.0 ; 
    
    jointControlMethod_ [ 7 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 7 ] .initPid ( 11.0 , 2.0 , 0.25 , 15.0 , - 15.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
        
    // Head Yaw
    
    jointName_ [ 8 ] = "linear_head_yaw_joint" ; //FIXME
    
    jointType_ [ 8 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 8 ] = 0.0 ; 
    jointPosition_ [ 8 ] = 0.0 ; 
    jointVelocity_ [ 8 ] = 0.0 ; 
    
    jointCommand_ [ 8 ] = 0.0 ; 
    
    jointLowerLimit_ [ 8 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 8 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 8 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 8 ] = 8.0 ; 
    
    jointControlMethod_ [ 8 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 8 ] .initPid ( 12.0 , 1.0 , 0.45 , 10.0 , - 10.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
  
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initLaser ( void ) { 
    
    // ------------------------------------------------------------------------
    
    // Roll
    
    jointName_ [ 9 ] = "laser_roll_joint" ; //FIXME
    
    jointType_ [ 9 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 9 ] = 0.0 ; 
    jointPosition_ [ 9 ] = 0.0 ; 
    jointVelocity_ [ 9 ] = 0.0 ; 
    
    jointCommand_ [ 9 ] = 0.0 ; 
    
    jointLowerLimit_ [ 9 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 9 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 9 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 9 ] = 0.1 ; 
    
    jointControlMethod_ [ 9 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 9 ] .initPid ( 1.8 , 0.0 , 0.3 , 0.0 , 0.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
        
    // Pitch
    
    jointName_ [ 10 ] = "laser_pitch_joint" ; 
    
    jointType_ [ 10 ] = urdf ::Joint ::REVOLUTE ; 
    
    jointEffort_ [ 10 ] = 0.0 ; 
    jointPosition_ [ 10 ] = 0.0 ; 
    jointVelocity_ [ 10 ] = 0.0 ; 
    
    jointCommand_ [ 10 ] = 0.0 ; 
    
    jointLowerLimit_ [ 10 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 10 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 10 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 10 ] = 0.1 ; 
    
    jointControlMethod_ [ 10 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 10 ] .initPid ( 2.5 , 0.0 , 0.3 , 0.0 , 0.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
  
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initKinect ( void ) { 
    
    // ------------------------------------------------------------------------
    
    // Pitch
    
    jointName_ [ 11 ] = "kinect_pitch_joint" ; //FIXME
    
    jointType_ [ 11 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 11 ] = 0.0 ; 
    jointPosition_ [ 11 ] = 0.0 ; 
    jointVelocity_ [ 11 ] = 0.0 ; 
    
    jointCommand_ [ 11 ] = 0.0 ; 
    
    jointLowerLimit_ [ 11 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 11 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 11 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 11 ] = 3.0 ; 
    
    jointControlMethod_ [ 11 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 11 ] .initPid ( 8.5 , 1.0 , 0.2 , 10.0 , - 10.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
      
    // Yaw
    
    jointName_ [ 12 ] = "kinect_yaw_joint" ; //FIXME
    
    jointType_ [ 12 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 12 ] = 0.0 ; 
    jointPosition_ [ 12 ] = 0.0 ; 
    jointVelocity_ [ 12 ] = 0.0 ; 
    
    jointCommand_ [ 12 ] = 0.0 ; 
    
    jointLowerLimit_ [ 12 ] = - 1.57079632679 ; //FIXME
    jointUpperLimit_ [ 12 ] = 1.57079632679 ; //FIXME
    
    jointEffortLimit_ [ 12 ] = 50.0 ; //FIXME
    //jointEffortLimit_ [ 12 ] = 5.0 ; 
    
    jointControlMethod_ [ 12 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 12 ] .initPid ( 8.0 , 1.5 , 0.4 , 10.0 , - 10.0 ) ; //FIXME
    
    // ------------------------------------------------------------------------
    
    return true ; 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initXMEGA ( void ) { 
    
    // ------------------------------------------------------------------------
  
    // Number of batteries and range sensors
    batteryNum_ = 2 ; //FIXME
    
    rangeSensorNum_ = 3 ; //FIXME
    
    // ------------------------------------------------------------------------
  
    // Update rates of read methods
      
    batteryUpdateRate_ = gazebo ::common ::Time ( 1 / 1.0 ) ; //FIXME
    batteryLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    rangeSensorUpdateRate_ = gazebo ::common ::Time ( 1 / 2.0 ) ; //FIXME
    rangeSensorLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    // ------------------------------------------------------------------------
    
    // Resize vectors
    batteryData_ .resize ( batteryNum_ ) ; 
    batteryName_ .resize ( batteryNum_ ) ; 
    batteryVoltage_ .resize ( batteryNum_ ) ; 
    batteryVoltageMax_ .resize ( batteryNum_ ) ; 
    batteryVoltageMin_ .resize ( batteryNum_ ) ; 
    batteryDuration_ .resize ( batteryNum_ ) ; 
    
    rangeSensorData_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorName_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorFrameID_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorRadiationType_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorFOV_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorMinRange_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorMaxRange_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorRange_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorBufferCounter_ .resize ( rangeSensorNum_ ) ; 
    rangeSensorRay_ .resize ( rangeSensorNum_ ) ; 
    
    // ------------------------------------------------------------------------
  
    // Initialize XMEGA data
    if ( ! initBatteries ( ) ) { 
    
      ROS_FATAL ( "Could not initialize batteries." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
  
    if ( ! initRangeSensors ( ) ) { 
    
      ROS_FATAL ( "Could not initialize range sensors." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initBatteries ( void ) { 
    
    // ------------------------------------------------------------------------
  
    batteryName_ [ 0 ] = "/PSU_battery" ; //FIXME
    batteryData_ [ 0 ] .name = batteryName_ [ 0 ] ; 
    
    batteryVoltageMax_ [ 0 ] = 24.0 ; //FIXME
    batteryVoltageMin_ [ 0 ] = 18.0 ; //FIXME
    batteryDuration_ [ 0 ] = 60.0 ; //FIXME
    
    batteryVoltage_ [ 0 ] = batteryVoltageMax_ [ 0 ] ; 
    batteryData_ [ 0 ] .voltage = & batteryVoltage_ [ 0 ] ; 
    
    // ------------------------------------------------------------------------
    
    batteryName_ [ 1 ] = "/motors_battery" ; //FIXME
    batteryData_ [ 1 ] .name = batteryName_ [ 1 ] ; 
    
    batteryVoltageMax_ [ 1 ] = 24.0 ; //FIXME
    batteryVoltageMin_ [ 1 ] = 18.0 ; //FIXME
    batteryDuration_ [ 1 ] = 45.0 ; //FIXME
    
    batteryVoltage_ [ 1 ] = batteryVoltageMax_ [ 1 ] ; 
    batteryData_ [ 1 ] .voltage = & batteryVoltage_ [ 1 ] ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initRangeSensors ( void ) { 
    
    
    // ------------------------------------------------------------------------
    
    rangeSensorName_ [ 0 ] = "/sensors/linear_sonar" ; //FIXME
    rangeSensorData_ [ 0 ] .name = rangeSensorName_ [ 0 ] ; 
    
    rangeSensorName_ [ 1 ] = "/sensors/left_sonar" ; //FIXME
    rangeSensorData_ [ 1 ] .name = rangeSensorName_ [ 1 ] ; 
    
    rangeSensorName_ [ 2 ] = "/sensors/right_sonar" ; //FIXME
    rangeSensorData_ [ 2 ] .name = rangeSensorName_ [ 2 ] ; 
    
    rangeSensorFrameID_ [ 0 ] = "linear_sonar_frame" ; //FIXME
    rangeSensorData_ [ 0 ] .frameId = rangeSensorFrameID_ [ 0 ] ; 
    
    rangeSensorFrameID_ [ 1 ] = "left_sonar_frame" ; //FIXME
    rangeSensorData_ [ 1 ] .frameId = rangeSensorFrameID_ [ 1 ] ; 
    
    rangeSensorFrameID_ [ 2 ] = "right_sonar_frame" ; //FIXME
    rangeSensorData_ [ 2 ] .frameId = rangeSensorFrameID_ [ 2 ] ; 
    
    for ( unsigned int i = 0 ; i < rangeSensorNum_ ; i ++ ) { 
    
      rangeSensorRadiationType_ [ i ] = 0 ; //FIXME
      rangeSensorData_ [ i ] .radiationType = 
      & rangeSensorRadiationType_ [ i ] ; 
      
      rangeSensorFOV_ [ i ] = 60.0 ; //FIXME
      rangeSensorData_ [ i ] .fieldOfView = & rangeSensorFOV_ [ i ] ; 
      
      rangeSensorMinRange_ [ i ] = 0.2 ; //FIXME
      rangeSensorData_ [ i ] .minRange = & rangeSensorMinRange_ [ i ] ; 
      
      rangeSensorMaxRange_ [ i ] = 4.0 ; //FIXME
      rangeSensorData_ [ i ] .maxRange = & rangeSensorMaxRange_ [ i ] ; 
      
      rangeSensorRange_ [ i ] .resize ( 5 , rangeSensorMaxRange_ [ i ] ) ; 
      rangeSensorData_ [ i ] .range = & rangeSensorRange_ [ i ] [ 0 ] ; 
      
      rangeSensorBufferCounter_ [ i ] = 0 ; 
    
    }
    
    rangeSensorRay_ [ 0 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::RaySensor > 
     ( gazebo ::sensors ::get_sensor ( "linear_sonar" ) ) ; 
    
    rangeSensorRay_ [ 1 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::RaySensor > 
     ( gazebo ::sensors ::get_sensor ( "left_sonar" ) ) ; 
    
    rangeSensorRay_ [ 2 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::RaySensor > 
     ( gazebo ::sensors ::get_sensor ( "right_sonar" ) ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initARM ( void ) { 
    
    // ------------------------------------------------------------------------
  
    // Number of co2 , thermal and microphone sensors
    co2SensorNum_ = 1 ; //FIXME
    
    thermalSensorNum_ = 3 ; //FIXME
    
    microphoneSensorNum_ = 1 ; //FIXME
    
    // ------------------------------------------------------------------------
  
    // Update rates of read methods
      
    co2SensorUpdateRate_ = gazebo ::common ::Time ( 1 / 2.0 ) ; //FIXME
    co2SensorLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    thermalSensorUpdateRate_ = gazebo ::common ::Time ( 1 / 1.0 ) ; //FIXME
    thermalSensorLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    microphoneSensorUpdateRate_ = gazebo ::common ::Time ( 1 / 2.0 ) ; //FIXME
    microphoneSensorLastUpdateTime_ = world_ ->GetSimTime ( ) ; 
    
    // ------------------------------------------------------------------------
    
    // Resize vectors
    co2SensorData_ .resize ( co2SensorNum_ ) ; 
    co2SensorName_ .resize ( co2SensorNum_ ) ; 
    co2SensorFrameID_ .resize ( co2SensorNum_ ) ; 
    co2SensorCo2Percentage_ .resize ( co2SensorNum_ ) ; 
    co2SensorCamera_ .resize ( co2SensorNum_ ) ; 
    
    thermalSensorData_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorName_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorFrameID_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorHeight_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorWidth_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorStep_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorVector_ .resize ( thermalSensorNum_ ) ; 
    thermalSensorCamera_ .resize ( thermalSensorNum_ ) ; 
    
    microphoneSensorName_ .resize ( microphoneSensorNum_ ) ; 
    microphoneSensorFrameID_ .resize ( microphoneSensorNum_ ) ; 
    microphoneSensorSoundCertainty_ .resize ( microphoneSensorNum_ ) ; 
    microphoneSensorCamera_ .resize ( microphoneSensorNum_ ) ; 
    
    // ------------------------------------------------------------------------
  
    // Initialize ARM data
  
    if ( ! initCO2Sensors ( ) ) { 
    
      ROS_FATAL ( "Could not initialize CO2 sensors." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    if ( ! initThermalSensors ( ) ) { 
    
      ROS_FATAL ( "Could not initialize thermal sensors." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
  
    if ( ! initMicrophoneSensors ( ) ) { 
    
      ROS_FATAL ( "Could not initialize microphone sensors." ) ; 
    
      return false ; 
      
    }
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initCO2Sensors ( void ) { 
    
    // ------------------------------------------------------------------------
  
    co2SensorName_ [ 0 ] = "/sensors/co2" ; //FIXME
    co2SensorData_ [ 0 ] .name = co2SensorName_ [ 0 ] ; 
    
    co2SensorFrameID_ [ 0 ] = "co2_frame" ; //FIXME
    co2SensorData_ [ 0 ] .frameId = co2SensorFrameID_ [ 0 ] ; 
    
    co2SensorCo2Percentage_ [ 0 ] = 0.0 ; 
    co2SensorData_ [ 0 ] .co2Percentage = & co2SensorCo2Percentage_ [ 0 ] ; 
    
    co2SensorCamera_ [ 0 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::CameraSensor > 
     ( gazebo ::sensors ::get_sensor ( "co2" ) ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initThermalSensors ( void ) { 
    
    // ------------------------------------------------------------------------
  
    thermalSensorName_ [ 0 ] = "/sensors/left_thermal" ; //FIXME
    thermalSensorData_ [ 0 ] .name = thermalSensorName_ [ 0 ] ; 
    
    thermalSensorName_ [ 1 ] = "/sensors/middle_thermal" ; //FIXME XXX
    thermalSensorData_ [ 1 ] .name = thermalSensorName_ [ 1 ] ; 
    
    thermalSensorName_ [ 2 ] = "/sensors/right_thermal" ; //FIXME
    thermalSensorData_ [ 2 ] .name = thermalSensorName_ [ 2 ] ; 
    
    thermalSensorFrameID_ [ 0 ] = "left_thermal_optical_frame" ; //FIXME
    thermalSensorData_ [ 0 ] .frameId = thermalSensorFrameID_ [ 0 ] ; 
    
    thermalSensorFrameID_ [ 1 ] = "middle_thermal_optical_frame" ; //FIXME
    thermalSensorData_ [ 1 ] .frameId = thermalSensorFrameID_ [ 1 ] ; 
    
    thermalSensorFrameID_ [ 2 ] = "right_thermal_optical_frame" ; //FIXME
    thermalSensorData_ [ 2 ] .frameId = thermalSensorFrameID_ [ 2 ] ; 
    
    for ( unsigned int i = 0 ; i < thermalSensorNum_ ; i ++ ) { 
    
      thermalSensorHeight_ [ i ] = 8 ; //FIXME
      thermalSensorData_ [ i ] .height = & thermalSensorHeight_ [ i ] ; 
      
      thermalSensorWidth_ [ i ] = 8 ; //FIXME
      thermalSensorData_ [ i ] .width = & thermalSensorWidth_ [ i ] ; 
      
      thermalSensorStep_ [ i ] = thermalSensorWidth_ [ i ] ; //FIXME
      thermalSensorData_ [ i ] .step = & thermalSensorStep_ [ i ] ; 
      
      int resolution = thermalSensorHeight_ [ i ] * thermalSensorWidth_ [ i ] ; 
      thermalSensorVector_ [ i ] .resize ( resolution , 0 ) ; 
      thermalSensorData_ [ i ] .data = & thermalSensorVector_ [ i ] [ 0 ] ; 
    
    }
    
    thermalSensorCamera_ [ 0 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::CameraSensor > 
     ( gazebo ::sensors ::get_sensor ( "left_thermal" ) ) ; 
     
    thermalSensorCamera_ [ 1 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::CameraSensor > 
     ( gazebo ::sensors ::get_sensor ( "middle_thermal" ) ) ; //FIXME: Adjust name
     
    thermalSensorCamera_ [ 2 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::CameraSensor > 
     ( gazebo ::sensors ::get_sensor ( "right_thermal" ) ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  bool GazeboInterface ::initMicrophoneSensors ( void ) { 
    
    // ------------------------------------------------------------------------
  
    microphoneSensorName_ [ 0 ] = "/sensors/microphone" ; //FIXME
    
    microphoneSensorFrameID_ [ 0 ] = "microphone_frame" ; 
    
    microphoneSensorSoundCertainty_ [ 0 ] = 0.0 ; 
    
    microphoneSensorCamera_ [ 0 ] = 
    boost ::dynamic_pointer_cast < gazebo ::sensors ::CameraSensor > 
     ( gazebo ::sensors ::get_sensor ( "microphone" ) ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  bool GazeboInterface ::registerInterfaces ( void ) { 
    
    // ------------------------------------------------------------------------
    
    // Connect and register the joint state handle
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface ::JointStateHandle 
       jointStateHandle ( jointName_ [ i ] , 
                          & jointPosition_ [ i ] , 
                          & jointVelocity_ [ i ] , 
                          & jointEffort_ [ i ] ) ; 
                                             
      jointStateInterface_ .registerHandle ( jointStateHandle ) ; 
      
    }
    
    // ------------------------------------------------------------------------

    // Connect and register the joint velocity handle
      
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
      
      hardware_interface ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointName_ [ i ] ) , 
                     & jointCommand_ [ i ] ) ; 
                             
      velocityJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------

    // Connect and register the joint position handle
      
    for ( unsigned int i = 4 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointName_ [ i ] ) , 
                     & jointCommand_ [ i ] ) ; 
                             
      positionJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------

    // Connect and register the imu sensor handle
    
    hardware_interface ::ImuSensorHandle 
     imuSensorHandle ( imuSensorData_ ) ; 
    
    imuSensorInterface_ .registerHandle ( imuSensorHandle ) ; 
    
    // ------------------------------------------------------------------------
    
    // Connect and register the battery handles
      
    for ( unsigned int i = 0 ; i < batteryNum_ ; i ++ ) { 
    
      pandora_hardware_interface ::xmega ::BatteryHandle 
       batteryHandle ( batteryData_ [ i ] ) ; 
    
      batteryInterface_ .registerHandle ( batteryHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    // Connect and register the range sensor handles
      
    for ( unsigned int i = 0 ; i < rangeSensorNum_ ; i ++ ) { 
    
      pandora_hardware_interface ::xmega ::RangeSensorHandle 
       rangeSensorHandle ( rangeSensorData_ [ i ] ) ; 
    
      rangeSensorInterface_ .registerHandle ( rangeSensorHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    // Connect and register the co2 sensor handles
      
    for ( unsigned int i = 0 ; i < co2SensorNum_ ; i ++ ) { 
    
      pandora_hardware_interface ::arm ::Co2SensorHandle 
       co2SensorHandle ( co2SensorData_ [ i ] ) ; 
    
      co2SensorInterface_ .registerHandle ( co2SensorHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    // Connect and register the thermal sensor handles
      
    for ( unsigned int i = 0 ; i < thermalSensorNum_ ; i ++ ) { 
    
      pandora_hardware_interface ::arm ::ThermalSensorHandle
       thermalSensorHandle ( thermalSensorData_ [ i ] ) ; 
    
      thermalSensorInterface_ .registerHandle ( thermalSensorHandle ) ; 
    
    }
    
    // ------------------------------------------------------------------------
    
    // Register interfaces
    
    registerInterface ( & jointStateInterface_ ) ; 
    registerInterface ( & positionJointInterface_ ) ; 
    registerInterface ( & velocityJointInterface_ ) ; 
    
    registerInterface ( & imuSensorInterface_ ) ; 
    
    registerInterface ( & batteryInterface_ ) ; 
    registerInterface ( & rangeSensorInterface_ ) ; 
    
    registerInterface ( & co2SensorInterface_ ) ; 
    registerInterface ( & thermalSensorInterface_ ) ; 
    
    // ------------------------------------------------------------------------
    
    return true ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readLinks ( void ) { 
    
    // ------------------------------------------------------------------------
  
    // Read robot orientation for IMU
    
    gazebo ::math ::Pose pose = gazeboLink_ [ 0 ] ->GetWorldPose ( ) ; 
    
    imuOrientation_ [ 0 ] = pose .rot .x ; 
    imuOrientation_ [ 1 ] = pose .rot .y ; 
    imuOrientation_ [ 2 ] = pose .rot .z ; 
    imuOrientation_ [ 3 ] = pose .rot .w ; 
    
    // ------------------------------------------------------------------------
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readJoints ( void ) { 
    
    // ------------------------------------------------------------------------
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      // Read joint position
    
      if ( jointType_ [ i ] == urdf ::Joint ::PRISMATIC ) 
      
        jointPosition_ [ i ] = gazeboJoint_ [ i ] 
                                ->GetAngle ( 0 ) .Radian ( ) ; 
      
      else
      
        jointPosition_ [ i ] += 
        angles ::shortest_angular_distance ( jointPosition_ [ i ] , 
                                             gazeboJoint_ [ i ] 
                                              ->GetAngle ( 0 ) .Radian ( ) ) ; 
               
      // Read joint velocity
                              
      jointVelocity_ [ i ] = gazeboJoint_ [ i ] 
                              ->GetVelocity ( 0 ) ; 
    
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readXMEGA ( void ) { 
    
    // ------------------------------------------------------------------------
  
    if ( ( batteryLastUpdateTime_ + batteryUpdateRate_ ) < 
         readTime_ ) { 
    
      readBatteries ( ) ; 
      
      batteryLastUpdateTime_ = readTime_ ; 
      
    }
    
    // ------------------------------------------------------------------------
  
    if ( ( rangeSensorLastUpdateTime_ + rangeSensorUpdateRate_ ) < 
         readTime_ ) { 
    
      readRangeSensors ( ) ; 
      
      rangeSensorLastUpdateTime_ = readTime_ ; 
      
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readBatteries ( void ) { 
    
    // ------------------------------------------------------------------------
    
    for ( unsigned int n = 0 ; n < batteryNum_ ; n ++ ) {
    
      double reduction = ( batteryVoltageMax_ [ n ] - batteryVoltageMin_ [ n ] ) 
                         / ( batteryDuration_ [ n ] * 60.0 ) ; 

      reduction /= batteryUpdateRate_ .sec + 
                   batteryUpdateRate_ .nsec / pow ( 10 , 9 ) ; 
      
      batteryVoltage_ [ n ] -= reduction ; 
      
      if ( batteryVoltage_ [ n ] < batteryVoltageMin_ [ n ] )
      
        ROS_WARN_ONCE ( "WARNING: The battery \"%s\" has died!" , 
                         batteryName_ [ n ] .c_str ( ) ) ; 
        
        //immobilizeRobot ( batteryName_ [ n ] , batteryVoltage_ [ n ] ) ; // TODO
     
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readRangeSensors ( void ) { 
    
    // ------------------------------------------------------------------------
    
    for ( unsigned int n = 0 ; n < rangeSensorNum_ ; n ++ ) { 
    
      int i , hja , hjb ; 
      int j , vja , vjb ; 
      
      double vb , hb ; 
      
      int    j1 , j2 , j3 , j4 ; 
      double r1 , r2 , r3 , r4 , r ; 
    
      gazebo ::math ::Angle maxAngle = rangeSensorRay_ [ n ] ->GetAngleMax ( ) ; 
      gazebo ::math ::Angle minAngle = rangeSensorRay_ [ n ] ->GetAngleMin ( ) ; 

      double maxRange = rangeSensorRay_ [ n ] ->GetRangeMax ( ) ; 
      double minRange = rangeSensorRay_ [ n ] ->GetRangeMin ( ) ; 
      int rayCount = rangeSensorRay_ [ n ] ->GetRayCount ( ) ; 
      int rangeCount = rangeSensorRay_ [ n ] ->GetRangeCount ( ) ; 

      int verticalRayCount = rangeSensorRay_ [ n ] 
                              ->GetVerticalRayCount ( ) ; 
      int verticalRangeCount = rangeSensorRay_ [ n ] 
                                ->GetVerticalRangeCount ( ) ; 
                                
      gazebo ::math ::Angle verticalMaxAngle = rangeSensorRay_ [ n ] 
                                                ->GetVerticalAngleMax ( ) ; 
      gazebo ::math ::Angle verticalMinAngle = rangeSensorRay_ [ n ] 
                                                ->GetVerticalAngleMin ( ) ; 

      double yDiff = maxAngle .Radian ( ) - minAngle .Radian ( ) ; 
      double pDiff = verticalMaxAngle .Radian ( ) - 
                     verticalMinAngle .Radian ( ) ; 
    
      double currentRange = maxRange ; 

      for ( j = 0 ; j < verticalRangeCount ; j++ ) { 
      
        vb = ( verticalRangeCount == 1 ) ? 0 : ( double ) j * 
                                               ( verticalRayCount - 1 ) / 
                                               ( verticalRangeCount -  1 ) ; 
        vja = ( int ) floor ( vb ) ; 
        vjb = std ::min ( vja + 1 , verticalRayCount - 1 ) ; 
        vb = vb - floor ( vb ) ; 

        assert ( vja >= 0 && vja < verticalRayCount ) ; 
        assert ( vjb >= 0 && vjb < verticalRayCount ) ; 

        for ( i = 0 ; i < rangeCount ; i++ ) { 
        
          hb = ( rangeCount == 1 ) ? 0 : ( double ) i * ( rayCount - 1 ) / 
                                         ( rangeCount - 1 ) ; 
          hja = ( int ) floor ( hb ) ; 
          hjb = std ::min ( hja + 1 , rayCount - 1 ) ; 
          hb = hb - floor ( hb ) ; 

          assert ( hja >= 0 && hja < rayCount ) ; 
          assert ( hjb >= 0 && hjb < rayCount ) ; 

          j1 = hja + vja * rayCount ; 
          j2 = hjb + vja * rayCount ; 
          j3 = hja + vjb * rayCount ; 
          j4 = hjb + vjb * rayCount ; 
          
          r1 = std ::min ( rangeSensorRay_ [ n ] 
                            ->GetLaserShape ( ) 
                             ->GetRange ( j1 ) , 
                           maxRange - minRange ) ; 
          r2 = std ::min ( rangeSensorRay_ [ n ] 
                            ->GetLaserShape ( ) 
                             ->GetRange ( j2 ) , 
                           maxRange - minRange ) ; 
          r3 = std ::min ( rangeSensorRay_ [ n ] 
                            ->GetLaserShape ( ) 
                             ->GetRange ( j3 ) , 
                           maxRange - minRange ) ; 
          r4 = std ::min ( rangeSensorRay_ [ n ] 
                            ->GetLaserShape ( ) 
                             ->GetRange ( j4 ) , 
                           maxRange - minRange ) ; 

          r = ( 1 - vb ) * ( ( 1 - hb ) * r1 + hb * r2 ) + 
              vb * ( ( 1 - hb ) * r3 + hb * r4 ) ; 
          
          double yAngle = 0.5 * ( hja + hjb ) * yDiff /  ( rayCount - 1 ) 
                          + minAngle .Radian ( ) ; 
          double pAngle = 0.5 * ( vja + vjb ) * pDiff / ( verticalRayCount - 1 ) 
                          + verticalMinAngle .Radian ( ) ; 

          if ( r != maxRange - minRange ) { 
          
            double point = ( r + minRange ) * cos ( pAngle ) * cos ( yAngle ) ; 
            
            if ( point < currentRange ) 
            
			        currentRange = point ; 
		
          } 
          
        }
        
      }
      
      rangeSensorRange_ [ n ] [ rangeSensorBufferCounter_ [ n ] ] = 
      currentRange ; 
      
      rangeSensorBufferCounter_ [ n ] = 
      fmod ( rangeSensorBufferCounter_ [ n ] + 1 , 5 ) ; 
    
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readARM ( void ) { 
    
    // ------------------------------------------------------------------------
  
    if ( ( co2SensorLastUpdateTime_ + co2SensorUpdateRate_ ) < 
         readTime_ ) { 
    
      readCO2Sensors ( ) ; 
      
      co2SensorLastUpdateTime_ = readTime_ ; 
      
    }
    
    // ------------------------------------------------------------------------
  
    if ( ( thermalSensorLastUpdateTime_ + thermalSensorUpdateRate_ ) < 
         readTime_ ) { 
    
      readThermalSensors ( ) ; 
      
      thermalSensorLastUpdateTime_ = readTime_ ; 
      
    }
    
    // ------------------------------------------------------------------------
  
    if ( ( microphoneSensorLastUpdateTime_ + microphoneSensorUpdateRate_ ) < 
         readTime_ ) { 
    
      readMicrophoneSensors ( ) ; 
      
      microphoneSensorLastUpdateTime_ = readTime_ ; 
      
    }
    
    // ------------------------------------------------------------------------
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readCO2Sensors ( void ) { 
    
    // ------------------------------------------------------------------------
      
    for ( unsigned int n = 0 ; n < co2SensorNum_ ; n ++ ) { 

      unsigned int width = co2SensorCamera_ [ n ] ->GetImageWidth ( ) ; 

      unsigned int height = co2SensorCamera_ [ n ] ->GetImageHeight ( ) ; 

      const unsigned char * data = co2SensorCamera_ [ n ] ->GetImageData ( ) ; 
      
      if ( data == NULL ) 
      
        return ; 
        
      double ambientPpm = 400.0 ; 
      
      double maxPpm = 40000.0 ; 

      double totalPpm = 0.0 ; 
      
      for ( unsigned int i = 0 ; i < width ; i++ ) { 

        for ( unsigned int j = 0 ; j < height ; j++ ) { 
          
          double currentPpm = 0 ; 

          double R = data [ ( ( i * height ) + j ) * 3 + 0 ] ; 
          double G = data [ ( ( i * height ) + j ) * 3 + 1 ] ; 
          double B = data [ ( ( i * height ) + j ) * 3 + 2 ] ; 

          // co2 is represented by green
          double G1 = ( G - R ) ; 
          double G2 = ( G - B ) ; 

          double positiveDiff = 0 ; 

          if ( G1 > 0 ) { 

            currentPpm += pow ( G1 , 2 ) ; 

            ++ positiveDiff ; 
          
          }

          if ( G2 > 0 ) { 

            currentPpm += pow ( G2 , 2 ) ; 

            ++ positiveDiff ; 

          }
          
          currentPpm = sqrt ( currentPpm ) ; 

          if ( positiveDiff == 1 ) 

            currentPpm /= 255.0 ; 

          else if ( positiveDiff == 2 )     

            currentPpm /= sqrt ( pow ( 255.0 , 2 ) 
                                 + pow ( 255.0 , 2 ) ) ;

          totalPpm += currentPpm ; 
          
        }

      }
      
      totalPpm /= ( width * height ) ; 
      
      double percentage = ( ambientPpm + totalPpm * maxPpm ) / 10000.0 ; 
      
      co2SensorCo2Percentage_ [ n ] = percentage ; 
    
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readThermalSensors ( void ) { 
    
    // ------------------------------------------------------------------------
      
    for ( unsigned int n = 0 ; n < thermalSensorNum_ ; n ++ ) { 

      unsigned int cameraWidth = thermalSensorCamera_ [ n ] 
                                  ->GetImageWidth ( ) ; 
                                  
      unsigned int sensorWidth = thermalSensorWidth_ [ n ] ; 
      
      unsigned int divWidth = ( cameraWidth / sensorWidth ) ; 

      unsigned int cameraHeight = thermalSensorCamera_ [ n ] 
                                   ->GetImageHeight ( ) ; 
                                   
      unsigned int sensorHeight = thermalSensorHeight_ [ n ] ; 
      
      unsigned int divHeight = ( cameraHeight / sensorHeight ) ; 
      
      const unsigned char * data = thermalSensorCamera_ [ n ] 
                                    ->GetImageData ( ) ; 
      
      if ( data == NULL ) 
      
        return ; 

      double ambientTemp = 25.0 ; 
      
      double maxTemp = 17.0 ; 
    
      for ( unsigned int i = 0 ; i < sensorWidth ; i++ ) { 

        for ( unsigned int j = 0 ; j < sensorHeight ; j++ ) { 
        
          double meanTemp = 0 ; 
        
          for ( unsigned int k = 0 ; k < divWidth ; k++ ) { 
        
            for ( unsigned int l = 0 ; l < divHeight ; l++ ) { 
          
              double currentTemp = 0 ; 

              double R = data [ ( ( k + i * divWidth ) * cameraHeight + 
                                 ( l + j * divHeight ) ) * 3 + 0 ] ; 
              double G = data [ ( ( k + i * divWidth ) * cameraHeight + 
                                 ( l + j * divHeight ) ) * 3 + 1 ] ; 
              double B = data [ ( ( k + i * divWidth ) * cameraHeight + 
                                 ( l + j * divHeight ) ) * 3 + 2 ] ; 

              // temperature is represented by red
              double R1 = ( R - G ) ; 
              double R2 = ( R - B ) ; 

              double positiveDiff = 0 ; 

              if ( R1 > 0 ) { 

                currentTemp += pow ( R1 , 2 ) ; 

                ++ positiveDiff ; 
              
              }

              if ( R2 > 0 ) { 

                currentTemp += pow ( R2 , 2 ) ; 

                ++ positiveDiff ; 

              }
              
              currentTemp = sqrt ( currentTemp ) ; 

              if ( positiveDiff == 1 ) 

                currentTemp /= 255.0 ; 

              else if ( positiveDiff == 2 ) 

                currentTemp /= sqrt ( pow ( 255.0 , 2 ) 
                                      + pow ( 255.0 , 2 ) ) ; 
            
              meanTemp += currentTemp ; 
            
            }
          
          }
          
          meanTemp /= ( divWidth * divHeight ) ; 
          
          double temp = ambientTemp + meanTemp * maxTemp ; 
          
          unsigned int mirrored_pixel 
          = ( sensorHeight - 1 - i ) * sensorWidth + j ; 

          thermalSensorVector_ [ n ] [ mirrored_pixel ] = ( char ) temp ; 
          
        }

      }
    
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::readMicrophoneSensors ( void ) { 
    
    // ------------------------------------------------------------------------
      
    for ( unsigned int n = 0 ; n < microphoneSensorNum_ ; n ++ ) { 

      unsigned int width = microphoneSensorCamera_ [ n ] ->GetImageWidth ( ) ; 

      unsigned int height = microphoneSensorCamera_ [ n ] ->GetImageHeight ( ) ; 

      const unsigned char * data = microphoneSensorCamera_ [ n ] 
                                    ->GetImageData ( ) ; 
      
      if ( data == NULL ) 
      
        return ; 

      double totalCert = 0.0 ; 
      
      for ( unsigned int i = 0 ; i < width ; i++ ) { 

        for ( unsigned int j = 0 ; j < height ; j++ ) { 
          
          double currentCert = 0 ; 

          double R = data [ ( ( i * height ) + j ) * 3 + 0 ] ; 
          double G = data [ ( ( i * height ) + j ) * 3 + 1 ] ; 
          double B = data [ ( ( i * height ) + j ) * 3 + 2 ] ; 

          // sound is represented by blue
          double B1 = ( B - R ) ; 
          double B2 = ( B - G ) ; 

          double positiveDiff = 0 ; 

          if ( B1 > 0 ) { 

            currentCert += pow ( B1 , 2 ) ; 

            ++ positiveDiff ; 
          
          }

          if ( B2 > 0 ) { 

            currentCert += pow ( B2 , 2 ) ; 

            ++ positiveDiff ; 

          }
          
          currentCert = sqrt ( currentCert ) ; 

          if ( positiveDiff == 1 ) 

            currentCert /= 255.0 ; 

          else if ( positiveDiff == 2 )     

            currentCert /= sqrt ( pow ( 255.0 , 2 ) 
                                  + pow ( 255.0 , 2 ) ) ; 

          totalCert += currentCert ; 
          
        }

      }
      
      double certainty = totalCert / ( width * height ) ; 
    
      microphoneSensorSoundCertainty_ [ n ] = certainty ; 
    
    }
    
    // ------------------------------------------------------------------------
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::writeLinks ( void ) { 
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::writeJoints ( void ) { 
  
    adjustWheelVelocityCommands ( ) ; 
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      // ----------------------------------------------------------------------
    
      if ( jointControlMethod_ [ i ] == POSITION_PID ) { 
        
        double error ; 
        
        double jointCommand = clamp ( jointCommand_ [ i ] , 
                                      jointLowerLimit_ [ i ] , 
                                      jointUpperLimit_ [ i ] ) ; 
        
        if ( jointType_ [ i ] == urdf ::Joint ::REVOLUTE ) 
          
          angles ::shortest_angular_distance_with_limits 
                    ( jointPosition_ [ i ] , 
                      jointCommand , 
                      jointLowerLimit_ [ i ] , 
                      jointUpperLimit_ [ i ] , 
                      error ) ; 
        
        else
          
          error = jointCommand  - jointPosition_ [ i ] ; 
        
        double pidCommand = pidController_ [ i ] 
                             .computeCommand ( error , writePeriod_ ) ; 

        double effortLimit = jointEffortLimit_ [ i ] ; 
                      
        double effort = clamp ( pidCommand , - effortLimit , effortLimit ) ; 
                           
        gazeboJoint_ [ i ] ->SetForce ( 0 , effort ) ; 
        
      }
    
      // ----------------------------------------------------------------------

      else if ( jointControlMethod_ [ i ] == VELOCITY ) { 
        
        gazeboJoint_ [ i ] 
         ->SetVelocity ( 0 , jointCommand_ [ i ] * 
                             wheelVelocityMultiplier_ ) ; 
        
      }
      
    }
  
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////

  void GazeboInterface ::adjustWheelVelocityCommands ( void ) { 
  
    double leftWheelVelocity = jointCommand_ [ 0 ] ; 
    double rightWheelVelocity = jointCommand_ [ 2 ] ; 
    
    double x = ( rightWheelVelocity + leftWheelVelocity ) * 
               wheelRadius_ / 2 ; 
                            
    double z = ( rightWheelVelocity - leftWheelVelocity ) * 
               wheelRadius_ / wheelSeparation_ ; 
               
    /*
                             
    x = ( + 1.001487279 ) * pow ( x , 1 ) + 
        ( + 0.456332984 ) * pow ( x , 3 ) + 
        ( - 4.442671017 ) * pow ( x , 5 ) + 
        ( + 8.289127632 ) * pow ( x , 7 ) ; 
                            
    z = ( + 0.556465955 ) * pow ( z , 1 ) + 
        ( + 2.363328174 ) * pow ( z , 3 ) + 
        ( - 3.710717148 ) * pow ( z , 5 ) + 
        ( + 1.595776157 ) * pow ( z , 7 ) ; 
        
    */
               
    double linearVelocity = ( + 9.446060513 ) * ( 0.1000 ) * pow ( x , 1 ) +
                            ( + 1.315202284 ) * ( 1.0000 ) * pow ( x , 3 ) +
                            ( - 6.404744656 ) * ( 1.0000 ) * pow ( x , 5 ) +
                            ( + 2.449775997 ) * ( 1.0000 ) * pow ( x , 7 ) ;
                            
    double angularVelocity = ( + 4.219745338 ) * ( 0.1000 ) * pow ( z , 1 ) +
                             ( + 3.166949881 ) * ( 0.0100 ) * pow ( z , 3 ) +
                             ( - 1.104404291 ) * ( 0.0100 ) * pow ( z , 5 ) +
                             ( + 8.216225035 ) * ( 0.0001 ) * pow ( z , 7 ) ; 
                             
    double newLeftWheelVelocity = 
    ( linearVelocity - angularVelocity * wheelSeparation_ / 2 ) / wheelRadius_ ; 
    
    double newRightWheelVelocity = 
    ( linearVelocity + angularVelocity * wheelSeparation_ / 2 ) / wheelRadius_ ; 
    
    jointCommand_ [ 0 ] = newLeftWheelVelocity ; 
    jointCommand_ [ 1 ] = newLeftWheelVelocity ; 
    jointCommand_ [ 2 ] = newRightWheelVelocity ; 
    jointCommand_ [ 3 ] = newRightWheelVelocity ; 
    
  }
    
  /////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////
  
}  // namespace pandora_gazebo_interface
