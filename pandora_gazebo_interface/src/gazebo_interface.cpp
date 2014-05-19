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
    
    writeXMEGA ( ) ; 
    
    writeARM ( ) ; 
    
  }

  bool GazeboInterface ::initLinks ( void ) { 
  
    // Number of links
    linkNum_ = 1 ; //FIXME
    
    // Resize vectors
    gazeboLinks_ .resize ( linkNum_ ) ; 
    
    linkNames_ .resize ( linkNum_ ) ; 
    
    // Initialize link data
    if ( ! initIMU ( ) ) { 
    
      return false ; 
      
    }

    // Load gazebo links
    for ( unsigned int i = 0 ; i < linkNum_ ; i ++ ) 
    
      gazeboLinks_ [ i ] = parentModel_ ->GetLink ( linkNames_ [ i ] ) ; 
    
    return true ; 
    
  }

  bool GazeboInterface ::initIMU ( void ) { 
    
    linkNames_ [ 0 ] = "base_link" ; //FIXME
    
    imuOrientation_ [ 0 ] = 0 ; //FIXME
    imuOrientation_ [ 1 ] = 0 ; //FIXME
    imuOrientation_ [ 2 ] = 0 ; //FIXME
    imuOrientation_ [ 3 ] = 1 ; //FIXME
    
    imuData_. orientation = imuOrientation_ ; 
    
    imuData_ .name= "/sensors/imu" ; //FIXME
    imuData_ .frame_id = linkNames_ [ 0 ] ; 
    
    return true ; 
  
  }

  bool GazeboInterface ::initJoints ( void ) { 
  
    // Number of joints
    jointNum_ = 13 ; //FIXME
    //jointNum_ = transmissions .size ( ) ; 
    
    // Resize vectors
    gazeboJoints_ .resize ( jointNum_ ) ; 
    
    jointNames_ . resize ( jointNum_ ) ; 
    jointTypes_ . resize ( jointNum_ ) ; 
    
    jointControlMethods_ . resize ( jointNum_ ) ; 
    pidControllers_ . resize ( jointNum_ ) ; 
    
    jointLowerLimits_ . resize ( jointNum_ ) ; 
    jointUpperLimits_ . resize ( jointNum_ ) ; 
    jointEffortLimits_ . resize ( jointNum_ ) ; 
    
    jointEffort_ . resize ( jointNum_ ) ; 
    jointPosition_ . resize ( jointNum_ ) ; 
    jointVelocity_ . resize ( jointNum_ ) ; 
    
    jointPositionCommand_ . resize ( jointNum_ ) ; 
    jointVelocityCommand_ . resize ( jointNum_ ) ; 
    
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
    
      gazeboJoints_ [ i ] = parentModel_ ->GetJoint ( jointNames_ [ i ] ) ; 
      
    // Load PID controllers and initialize / set the limits.
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      if ( jointControlMethods_ [ i ] == POSITION_PID ) { 
      
        /*
      
        const ros ::NodeHandle nh ( modelNh_ , robotnamespace_ + 
                                               "/gazebo_ros_control/pid_gains/" + 
                                               jointNames_ [ i ] ) ; 
        
        pidControllers_ [ i ] .init ( nh ) ; 
        
        */
      
      }
    
      if ( jointControlMethods_ [ i ] == VELOCITY ) 

        gazeboJoints_ [ i ] ->SetMaxForce ( 0 , jointEffortLimits_ [ i ] ) ; 
        
    }
    
    return true ; 
    
  }

  bool GazeboInterface ::initWheels ( void ) { 
    
    jointNames_ [ 0 ] = "left_front_wheel_joint" ; //FIXME
    jointNames_ [ 1 ] = "left_rear_wheel_joint" ; //FIXME
    jointNames_ [ 2 ] = "right_front_wheel_joint" ; //FIXME
    jointNames_ [ 3 ] = "right_rear_wheel_joint" ; //FIXME
    
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
    
      jointTypes_ [ i ] = urdf ::Joint ::CONTINUOUS ; //FIXME
      
      jointEffort_ [ i ] = 1.0 ; 
      jointPosition_ [ i ] = 1.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      jointVelocityCommand_ [ i ] = 0.0 ; 
      jointEffortLimits_ [ i ] = 100.0 ; //FIXME
    
      jointControlMethods_ [ i ] = VELOCITY ; //FIXME
      
      wheel_velocity_multiplier_ [ i ] = 1.25 * 22.5 / 255.0 ; //FIXME
      //wheel_velocity_multiplier_ [ i ] = 15.8 ; 
    
    }
    
    return true ; 
  
  }

  bool GazeboInterface ::initSides ( void ) { 
    
    jointNames_ [ 4 ] = "left_side_joint" ; //FIXME
    jointNames_ [ 5 ] = "right_side_joint" ; //FIXME
    
    for ( unsigned int i = 0 ; i < 2 ; i ++ ) { 
    
      jointTypes_ [ i ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
      jointEffort_ [ i ] = 0.0 ; 
      jointPosition_ [ i ] = 0.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      jointPositionCommand_ [ i ] = 0.0 ; 
      jointLowerLimits_ [ i ] = - 0.785 ; //FIXME
      jointUpperLimits_ [ i ] = 0.785 ; //FIXME
      jointEffortLimits_ [ i ] = 150.0 ; //FIXME
      //jointEffortLimits_ [ i ] = 0.0 ; 
      
      jointControlMethods_ [ i ] = NONE ; //FIXME
    
    }
    
    return true ; 
  
  }

  bool GazeboInterface ::initLinear ( void ) { 
        
    // Elevator ---------------------------------------------------------------
    
    jointNames_ [ 6 ] = "linear_elevator_joint" ; //FIXME
    
    jointTypes_ [ 6 ] = urdf ::Joint ::PRISMATIC ;//FIXME
     
    jointEffort_ [ 6 ] = 0.0 ; 
    jointPosition_ [ 6 ] = 0.0 ; 
    jointVelocity_ [ 6 ] = 0.0 ; 
    jointPositionCommand_ [ 6 ] = 0.0 ; 
    jointLowerLimits_ [ 6 ] = 0.0 ; //FIXME
    jointUpperLimits_ [ 6 ] = 0.23 ; //FIXME
    jointEffortLimits_ [ 6 ] = 100.0 ; //FIXME
    //jointEffortLimits_ [ 6 ] = 15.0 ; 
    
    jointControlMethods_ [ 6 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 6 ] .initPid ( 90.0 , 5.0 , 20.0 , 100.0 , - 100.0 ) ; //FIXME
        
    // Head Pitch -------------------------------------------------------------
    
    jointNames_ [ 7 ] = "linear_head_pitch_joint" ; //FIXME
    
    jointTypes_ [ 7 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 7 ] = 0.0 ; 
    jointPosition_ [ 7 ] = 0.0 ; 
    jointVelocity_ [ 7 ] = 0.0 ; 
    jointPositionCommand_ [ 7 ] = 0.0 ; 
    jointLowerLimits_ [ 7 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 7 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 7 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 7 ] = 8.0 ; 
    
    jointControlMethods_ [ 7 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 7 ] .initPid ( 11.0 , 2.0 , 0.25 , 15.0 , - 15.0 ) ; //FIXME
        
    // Head Yaw ---------------------------------------------------------------
    
    jointNames_ [ 8 ] = "linear_head_yaw_joint" ; //FIXME
    
    jointTypes_ [ 8 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 8 ] = 0.0 ; 
    jointPosition_ [ 8 ] = 0.0 ; 
    jointVelocity_ [ 8 ] = 0.0 ; 
    jointPositionCommand_ [ 8 ] = 0.0 ; 
    jointLowerLimits_ [ 8 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 8 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 8 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 8 ] = 8.0 ; 
    
    jointControlMethods_ [ 8 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 8 ] .initPid ( 12.0 , 1.0 , 0.45 , 10.0 , - 10.0 ) ; //FIXME
  
    return true ; 
  
  }

  bool GazeboInterface ::initLaser ( void ) { 
    
    // Roll -------------------------------------------------------------------
    
    jointNames_ [ 9 ] = "laser_roll_joint" ; //FIXME
    
    jointTypes_ [ 9 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 9 ] = 0.0 ; 
    jointPosition_ [ 9 ] = 0.0 ; 
    jointVelocity_ [ 9 ] = 0.0 ; 
    jointPositionCommand_ [ 9 ] = 0.0 ; 
    jointLowerLimits_ [ 9 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 9 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 9 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 9 ] = 0.1 ; 
    
    jointControlMethods_ [ 9 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 9 ] .initPid ( 1.8 , 0.0 , 0.3 , 0.0 , 0.0 ) ; //FIXME
        
    // Pitch ------------------------------------------------------------------
    
    jointNames_ [ 10 ] = "laser_pitch_joint" ; 
    
    jointTypes_ [ 10 ] = urdf ::Joint ::REVOLUTE ; 
    
    jointEffort_ [ 10 ] = 0.0 ; 
    jointPosition_ [ 10 ] = 0.0 ; 
    jointVelocity_ [ 10 ] = 0.0 ; 
    jointPositionCommand_ [ 10 ] = 0.0 ; 
    jointLowerLimits_ [ 10 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 10 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 10 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 10 ] = 0.1 ; 
    
    jointControlMethods_ [ 10 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 10 ] .initPid ( 2.5 , 0.0 , 0.3 , 0.0 , 0.0 ) ; //FIXME
  
    return true ; 
  
  }

  bool GazeboInterface ::initKinect ( void ) { 
    
    // Pitch ------------------------------------------------------------------
    
    jointNames_ [ 11 ] = "kinect_pitch_joint" ; //FIXME
    
    jointTypes_ [ 11 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 11 ] = 0.0 ; 
    jointPosition_ [ 11 ] = 0.0 ; 
    jointVelocity_ [ 11 ] = 0.0 ; 
    jointPositionCommand_ [ 11 ] = 0.0 ; 
    jointLowerLimits_ [ 11 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 11 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 11 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 11 ] = 3.0 ; 
    
    jointControlMethods_ [ 11 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 11 ] .initPid ( 8.5 , 1.0 , 0.2 , 10.0 , - 10.0 ) ; //FIXME
      
    // Yaw --------------------------------------------------------------------
    
    jointNames_ [ 12 ] = "kinect_yaw_joint" ; //FIXME
    
    jointTypes_ [ 12 ] = urdf ::Joint ::REVOLUTE ; //FIXME
    
    jointEffort_ [ 12 ] = 0.0 ; 
    jointPosition_ [ 12 ] = 0.0 ; 
    jointVelocity_ [ 12 ] = 0.0 ; 
    jointPositionCommand_ [ 12 ] = 0.0 ; 
    jointLowerLimits_ [ 12 ] = - 1.57079632679 ; //FIXME
    jointUpperLimits_ [ 12 ] = 1.57079632679 ; //FIXME
    jointEffortLimits_ [ 12 ] = 50.0 ; //FIXME
    //jointEffortLimits_ [ 12 ] = 5.0 ; 
    
    jointControlMethods_ [ 12 ] = POSITION_PID ; //FIXME
    
    pidControllers_ [ 12 ] .initPid ( 8.0 , 1.5 , 0.4 , 10.0 , - 10.0 ) ; //FIXME
    
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
       ::JointStateHandle jointStateHandle ( jointNames_ [ i ] , 
                                             & jointEffort_ [ i ] , 
                                             & jointPosition_ [ i ] , 
                                             & jointVelocity_ [ i ] ) ; 
                                             
      jointStateInterface_ .registerHandle ( jointStateHandle ) ; 
      
    }

    // Connect and register the joint velocity handle
      
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointNames_ [ i ] ) , 
                     & jointVelocityCommand_ [ i ] ) ; 
                             
      velocityJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }

    // Connect and register the joint position handle
      
    for ( unsigned int i = 4 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointNames_ [ i ] ) , 
                     & jointPositionCommand_ [ i ] ) ; 
                             
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
    
    gazebo ::math ::Pose pose = gazeboLinks_ [ 0 ] ->GetWorldPose ( ) ; 
    
    imuOrientation_ [ 0 ] = pose .rot .x ; 
    imuOrientation_ [ 1 ] = pose .rot .y ; 
    imuOrientation_ [ 2 ] = pose .rot .z ; 
    imuOrientation_ [ 3 ] = pose .rot .w ; 
    
  }

  void GazeboInterface ::readJoints ( void ) { 
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      // Read joint position
    
      if ( jointTypes_ [ i ] == urdf ::Joint ::PRISMATIC ) 
      
        jointPosition_ [ i ] = gazeboJoints_ [ i ] 
                                ->GetAngle ( 0 ) .Radian ( ) ; 
      
      else
      
        jointPosition_ [ i ] += 
        angles ::shortest_angular_distance ( jointPosition_ [ i ] , 
                                             gazeboJoints_ [ i ] 
                                              ->GetAngle ( 0 ) .Radian ( ) ) ; 
               
      // Read joint velocity
                              
      jointVelocity_ [ i ] = gazeboJoints_ [ i ] 
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
    
      switch ( jointControlMethods_ [ i ] ) { 

        case POSITION_PID: {
        
            double error ; 
            
            switch ( jointTypes_ [ i ] ) {
            
              case urdf ::Joint ::REVOLUTE: 
              
                angles ::shortest_angular_distance_with_limits 
                          ( jointPosition_ [ i ] , 
                            jointPositionCommand_ [ i ] , 
                            jointLowerLimits_ [ i ] , 
                            jointUpperLimits_ [ i ] , 
                            error ) ; 
                            
              break ; 
                
              case urdf ::Joint ::CONTINUOUS: 
              
                error = angles ::shortest_angular_distance 
                                  ( jointPosition_ [ i ] , 
                                    jointPositionCommand_ [ i] ) ; 
                                    
              break ; 
                
              default: 
              
                error = jointPositionCommand_ [ i ] - jointPosition_ [ i ] ; 
                
            }
            
            double command = pidControllers_ [ i ] 
                              .computeCommand ( error , writePeriod_ ) ; 

            double effortLimit = jointEffortLimits_ [ i ] ; 
                                        
            double effort = 
            std ::min ( std ::max ( command , - effortLimit ) , effortLimit ) ; 
                                        
            gazeboJoints_ [ i ] ->SetForce ( 0 , effort ) ; 
            
        }
        
        break ; 

        case VELOCITY: 
        
          gazeboJoints_ [ i ] 
           ->SetVelocity ( 0 , jointVelocityCommand_ [ i ] * 
                               wheel_velocity_multiplier_ [ i ] ) ; 
          
        break ; 
          
      }
      
    }
  
  }

  void GazeboInterface ::writeXMEGA ( void ) { 
  
    // TODO
    
  }

  void GazeboInterface ::writeARM ( void ) { 
  
    // TODO
    
  }
  
}  // namespace pandora_gazebo_interface
