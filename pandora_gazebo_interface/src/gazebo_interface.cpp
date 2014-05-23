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
                                    
    robotnamespace_ = robotnamespace ; 
    modelNh_ = modelNh ; 
    parentModel_ = parentModel ; 
    urdfModel_ = urdfModel ; 
    transmissions_ = transmissions ; 
  
    if ( ! initLinks ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize links . "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    if ( ! initJoints ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize joints. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    if ( ! initXMEGA ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize XMEGA. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    if ( ! initARM ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to initialize ARM. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    if ( ! registerInterfaces ( ) ) {
  
      ROS_FATAL_STREAM (    "Unable to register interfaces. "
                         << "pandora_gazebo_interface initializing failed." ) ; 
    
      return false ; 
    
    }
    
    ROS_INFO ( "pandora_gazebo_interface initialized successfully!" ) ; 
    
    return true ; 
    
  }

  void GazeboInterface ::readSim ( ros ::Time time , ros ::Duration period ) { 
  
    readTime_ = time ; 
    readPeriod_ = period ; 
  
    readLinks ( ) ; 
    
    readJoints ( ) ; 
    
    readXMEGA ( ) ; 
    
    readARM ( ) ; 
    
  }

  void GazeboInterface ::writeSim ( ros ::Time time , ros ::Duration period ) { 
  
    writeTime_ = time ; 
    writePeriod_ = period ; 
  
    writeLinks ( ) ; 
  
    writeJoints ( ) ; 
    
  }

  bool GazeboInterface ::initLinks ( void ) { 
  
    // Number of links
    linkNum_ = 1 ; //FIXME
    
    // Resize vectors
    gazeboLink_ .resize ( linkNum_ ) ; 
    
    linkName_ .resize ( linkNum_ ) ; 
    
    // Initialize link data
    if ( ! initIMU ( ) ) { 
    
      return false ; 
      
    }

    // Load gazebo links
    for ( unsigned int i = 0 ; i < linkNum_ ; i ++ ) 
    
      gazeboLink_ [ i ] = parentModel_ ->GetLink ( linkName_ [ i ] ) ; 
    
    return true ; 
    
  }

  bool GazeboInterface ::initIMU ( void ) { 
    
    linkName_ [ 0 ] = "base_link" ; //FIXME
    
    imuOrientation_ [ 0 ] = 0 ; //FIXME
    imuOrientation_ [ 1 ] = 0 ; //FIXME
    imuOrientation_ [ 2 ] = 0 ; //FIXME
    imuOrientation_ [ 3 ] = 1 ; //FIXME
    
    imuData_. orientation = imuOrientation_ ; 
    
    imuData_ .name= "/sensors/imu" ; //FIXME
    imuData_ .frame_id = linkName_ [ 0 ] ; 
    
    return true ; 
  
  }

  bool GazeboInterface ::initJoints ( void ) { 
  
    // Number of joints
    jointNum_ = 13 ; //FIXME
    //jointNum_ = transmissions .size ( ) ; 
    
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
    
    wheel_velocity_multiplier_ . resize ( jointNum_ ) ; 
    
    // Initialize link data
    if ( ! initWheels ( ) ) { 
    
      return false ; 
      
    }
    
    if ( ! initSides ( ) ) { 
    
      return false ; 
      
    }
    
    if ( ! initLinear ( ) ) { 
    
      return false ; 
      
    }
    
    if ( ! initLaser ( ) ) { 
    
      return false ; 
      
    }
    
    if ( ! initKinect ( ) ) { 
    
      return false ; 
      
    }

    // Load gazebo joints
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) 
    
      gazeboJoint_ [ i ] = parentModel_ ->GetJoint ( jointName_ [ i ] ) ; 
      
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
    
    return true ; 
    
  }

  bool GazeboInterface ::initWheels ( void ) { 
    
    jointName_ [ 0 ] = "left_front_wheel_joint" ; //FIXME
    jointName_ [ 1 ] = "left_rear_wheel_joint" ; //FIXME
    jointName_ [ 2 ] = "right_front_wheel_joint" ; //FIXME
    jointName_ [ 3 ] = "right_rear_wheel_joint" ; //FIXME
    
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
    
      jointType_ [ i ] = urdf ::Joint ::CONTINUOUS ; //FIXME
      
      jointEffort_ [ i ] = 0.0 ; 
      jointPosition_ [ i ] = 0.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      
      jointCommand_ [ i ] = 0.0 ; 
      
      jointEffortLimit_ [ i ] = 100.0 ; //FIXME
    
      jointControlMethod_ [ i ] = VELOCITY ; //FIXME
      
      wheel_velocity_multiplier_ [ i ] = 1.25 * 22.5 / 255.0 ; //FIXME
      //wheel_velocity_multiplier_ [ i ] = 15.8 ; 
    
    }
    
    return true ; 
  
  }

  bool GazeboInterface ::initSides ( void ) { 
    
    jointName_ [ 4 ] = "left_side_joint" ; //FIXME
    jointName_ [ 5 ] = "right_side_joint" ; //FIXME
    
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
    
    return true ; 
  
  }

  bool GazeboInterface ::initLinear ( void ) { 
        
    // Elevator ---------------------------------------------------------------
    
    jointName_ [ 6 ] = "linear_elevator_joint" ; //FIXME
    
    jointType_ [ 6 ] = urdf ::Joint ::PRISMATIC ;//FIXME
     
    jointEffort_ [ 6 ] = 0.0 ; 
    jointPosition_ [ 6 ] = 0.0 ; 
    jointVelocity_ [ 6 ] = 0.0 ; 
    
    jointCommand_ [ 6 ] = 0.0 ; 
    
    jointLowerLimit_ [ 6 ] = 0.0 ; //FIXME
    jointUpperLimit_ [ 6 ] = 0.23 ; //FIXME
    
    jointEffortLimit_ [ 6 ] = 100.0 ; //FIXME
    //jointEffortLimit_ [ 6 ] = 15.0 ; 
    
    jointControlMethod_ [ 6 ] = POSITION_PID ; //FIXME
    
    pidController_ [ 6 ] .initPid ( 90.0 , 5.0 , 20.0 , 100.0 , - 100.0 ) ; //FIXME
        
    // Head Pitch -------------------------------------------------------------
    
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
        
    // Head Yaw ---------------------------------------------------------------
    
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
  
    return true ; 
  
  }

  bool GazeboInterface ::initLaser ( void ) { 
    
    // Roll -------------------------------------------------------------------
    
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
        
    // Pitch ------------------------------------------------------------------
    
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
  
    return true ; 
  
  }

  bool GazeboInterface ::initKinect ( void ) { 
    
    // Pitch ------------------------------------------------------------------
    
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
      
    // Yaw --------------------------------------------------------------------
    
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
    
    return true ; 
  
  }

  bool GazeboInterface ::initXMEGA ( void ) { 
  
    if ( ! initSonars ( ) ) { 
    
      return false ; 
      
    }
    
    return true ; 
    
  }

  bool GazeboInterface ::initSonars ( void ) { 
  
    // TODO
    
    return true ; 
    
  }

  bool GazeboInterface ::initARM ( void ) { 
  
    if ( ! initThermals ( ) ) { 
    
      return false ; 
      
    }
  
    if ( ! initCO2 ( ) ) { 
    
      return false ; 
      
    }
  
    if ( ! initMicrophone ( ) ) { 
    
      return false ; 
      
    }
    
    return true ; 
    
  }

  bool GazeboInterface ::initThermals ( void ) { 
  
    // TODO
    
    return true ; 
    
  }

  bool GazeboInterface ::initCO2 ( void ) { 
  
    // TODO
    
    return true ; 
    
  }

  bool GazeboInterface ::initMicrophone ( void ) { 
  
    // TODO
    
    return true ; 
    
  }
  
  bool GazeboInterface ::registerInterfaces ( void ) { 

    // Connect and register the imu sensor handle
    
    hardware_interface ::ImuSensorHandle imuSensorHandle ( imuData_ ) ; 
    
    imuSensorInterface_ .registerHandle ( imuSensorHandle ) ; 
    
    // Connect and register the joint state handle
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface 
       ::JointStateHandle jointStateHandle ( jointName_ [ i ] , 
                                             & jointPosition_ [ i ] , 
                                             & jointVelocity_ [ i ] , 
                                             & jointEffort_ [ i ] ) ; 
                                             
      jointStateInterface_ .registerHandle ( jointStateHandle ) ; 
      
    }

    // Connect and register the joint velocity handle
      
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointName_ [ i ] ) , 
                     & jointCommand_ [ i ] ) ; 
                             
      velocityJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }

    // Connect and register the joint position handle
      
    for ( unsigned int i = 4 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointName_ [ i ] ) , 
                     & jointCommand_ [ i ] ) ; 
                             
      positionJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }
    
    // Register interfaces
    
    registerInterface ( & imuSensorInterface_ ) ; 
    registerInterface ( & jointStateInterface_ ) ; 
    registerInterface ( & positionJointInterface_ ) ; 
    registerInterface ( & velocityJointInterface_ ) ; 
    
    return true ; 
    
  }

  void GazeboInterface ::readLinks ( void ) { 
  
    // Read robot orientation for IMU
    
    gazebo ::math ::Pose pose = gazeboLink_ [ 0 ] ->GetWorldPose ( ) ; 
    
    imuOrientation_ [ 0 ] = pose .rot .x ; 
    imuOrientation_ [ 1 ] = pose .rot .y ; 
    imuOrientation_ [ 2 ] = pose .rot .z ; 
    imuOrientation_ [ 3 ] = pose .rot .w ; 
    
  }

  void GazeboInterface ::readJoints ( void ) { 
    
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
  
  }

  void GazeboInterface ::readXMEGA ( void ) { 
  
    // TODO
  
  }

  void GazeboInterface ::readARM ( void ) { 
  
    // TODO
    
  }

  void GazeboInterface ::writeLinks ( void ) { 
  
  }

  void GazeboInterface ::writeJoints ( void ) { 
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
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

      else if ( jointControlMethod_ [ i ] == VELOCITY )  
        
        gazeboJoint_ [ i ] 
         ->SetVelocity ( 0 , jointCommand_ [ i ] * 
                             wheel_velocity_multiplier_ [ i ] ) ; 
      
    }
  
  }
  
}  // namespace pandora_gazebo_interface
