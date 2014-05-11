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

  bool GazeboInterface ::initSim ( const std ::string & robotnamespace , 
                                   ros ::NodeHandle modelNh , 
                                   gazebo ::physics ::ModelPtr parentModel , 
                                   const urdf ::Model * const urdfModel , 
                                   std 
                                    ::vector 
                                    < transmission_interface ::TransmissionInfo > 
                                    transmissions ) 
  
  {
  
    // Number of transmissions / joints
    //jointNum_ = transmissions .size ( ) ; 
    jointNum_ = 8 ; 
    //jointNum_ = 9 ; 
    
    // Resize vectors
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
    
    jointEffortCommand_ . resize ( jointNum_ ) ; 
    jointPositionCommand_ . resize ( jointNum_ ) ; 
    jointVelocityCommand_ . resize ( jointNum_ ) ; 
    
    wheel_velocity_multiplier_ . resize ( jointNum_ ) ; 
    
    gazeboJoints_ .resize ( jointNum_ ) ; 
  
    // Variable initialization
    registerInterfaces ( ) ; 

    // Load gazebo link
    gazeboLink_ = parentModel ->GetLink ( imuData_ .frame_id ) ; 

    // Load gazebo joints
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) 
    
      gazeboJoints_ [ i ] = parentModel ->GetJoint ( jointNames_ [ i ] ) ; 
      
    // Load PID controllers and initialize / set the limits.
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
    
      if ( jointControlMethods_ [ i ] == POSITION_PID ) { 
      
        /*
      
        const ros ::NodeHandle nh ( modelNh , robotnamespace + 
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

  GazeboInterface ::~GazeboInterface ( ) { 
  
  
  
  }

  void GazeboInterface ::readSim ( ros ::Time time , ros ::Duration period ) { 
  
    // Read robot orientation for IMU
    
    gazebo ::math ::Pose pose = gazeboLink_ ->GetWorldPose ( ) ; 
    
    imuOrientation_ [ 0 ] = pose .rot .x ; 
    imuOrientation_ [ 1 ] = pose .rot .y ; 
    imuOrientation_ [ 2 ] = pose .rot .z ; 
    imuOrientation_ [ 3 ] = pose .rot .w ; 
    
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

  void GazeboInterface ::writeSim ( ros ::Time time , ros ::Duration period ) { 
    
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
                              .computeCommand ( error , period ) ; 

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
  
  void GazeboInterface ::registerInterfaces ( ) { 

    // Connect and register imu sensor interface
    
    imuOrientation_ [ 0 ] = 0 ; 
    imuOrientation_ [ 1 ] = 0 ; 
    imuOrientation_ [ 2 ] = 0 ; 
    imuOrientation_ [ 3 ] = 1 ; 
    
    imuData_. orientation = imuOrientation_ ; 
    imuData_ .name= "/sensors/imu" ; 
    imuData_ .frame_id = "base_link" ; 
    
    hardware_interface ::ImuSensorHandle imuSensorHandle ( imuData_ ) ; 
    imuSensorInterface_ .registerHandle ( imuSensorHandle ) ; 
    
    registerInterface ( & imuSensorInterface_ ) ; 

    // Connect and register the joint state interface
    
    // All joints are currently hardcoded
    
    jointNames_ [ 0 ] = "left_front_wheel_joint" ; 
    jointNames_ [ 1 ] = "left_rear_wheel_joint" ; 
    jointNames_ [ 2 ] = "right_front_wheel_joint" ; 
    jointNames_ [ 3 ] = "right_rear_wheel_joint" ; 
    
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
    
      jointTypes_ [ i ] = urdf ::Joint ::CONTINUOUS ; 
      jointEffort_ [ i ] = 1.0 ; 
      jointPosition_ [ i ] = 1.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      jointEffortCommand_ [ i ] = 0.0 ; 
      jointPositionCommand_ [ i ] = 0.0 ; 
      jointVelocityCommand_ [ i ] = 0.0 ; 
      jointEffortLimits_ [ i ] = 50.0 ; 
      wheel_velocity_multiplier_ [ i ] = 15.8 ; 
    
    }
    
    jointNames_ [ 4 ] = "kinect_pitch_joint" ; 
    jointNames_ [ 5 ] = "kinect_yaw_joint" ; 
    jointNames_ [ 6 ] = "laser_roll_joint" ; 
    jointNames_ [ 7 ] = "laser_pitch_joint" ; 
    
    for ( unsigned int i = 4 ; i < 8 ; i ++ ) { 
    
      jointTypes_ [ i ] = urdf ::Joint ::REVOLUTE ; 
      jointEffort_ [ i ] = 0.0 ; 
      jointPosition_ [ i ] = 0.0 ; 
      jointVelocity_ [ i ] = 0.0 ; 
      jointEffortCommand_ [ i ] = 0.0 ; 
      jointPositionCommand_ [ i ] = 0.0 ; 
      jointVelocityCommand_ [ i ] = 0.0 ; 
      jointLowerLimits_ [ i ] = - 1.57079632679 ; 
      jointUpperLimits_ [ i ] = 1.57079632679 ; 
      jointEffortLimits_ [ i ] = 300.0 ; 
    
    }
    
    pidControllers_ [ 4 ] .initPid ( 1.2 , 0.0 , 0.5 , 0.0 , 0. ) ; 
    pidControllers_ [ 5 ] .initPid ( 0.8 , 0.0 , 0.45 , 0.0 , 0.0 ) ; 
    pidControllers_ [ 6 ] .initPid ( 1.8 , 0.0 , 0.45 , 0.0 , 0.0 ) ; 
    pidControllers_ [ 7 ] .initPid ( 2.5 , 0.0 , 0.3 , 0.0 , 0.0 ) ; 
    
    /*
    
    jointNames_ [ 8 ] = "linear_joint" ; 
    
    jointTypes_ [ i ] = urdf ::Joint ::PRISMATIC ; 
    jointEffort_ [ i ] = 0.0 ; 
    jointPosition_ [ i ] = 0.0 ; 
    jointVelocity_ [ i ] = 0.0 ; 
    jointEffortCommand_ [ i ] = 0.0 ; 
    jointPositionCommand_ [ i ] = 0.0 ; 
    jointVelocityCommand_ [ i ] = 0.0 ; 
    jointLowerLimits_ [ i ] = - TODO ; 
    jointUpperLimits_ [ i ] = TODO ; 
    jointEffortLimits_ [ i ] = TODO ; 
    
    pidController_ [ 8 ] .initPid ( TODO ) ; 
    
    */
    
    for ( unsigned int i = 0 ; i < jointNum_ ; i ++ ) { 
      
      hardware_interface 
       ::JointStateHandle jointStateHandle ( jointNames_ [ i ] , 
                                             & jointEffort_ [ i ] , 
                                             & jointPosition_ [ i ] , 
                                             & jointVelocity_ [ i ] ) ; 
                                             
      jointStateInterface_ .registerHandle ( jointStateHandle ) ; 
      
    }

    // Connect and register the joint velocity interface
      
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
    
      jointControlMethods_ [ i ] = VELOCITY ; 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointNames_ [ i ] ) , 
                     & jointVelocityCommand_ [ i ] ) ; 
                             
      velocityJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }

    // Connect and register the joint position interface
      
    for ( unsigned int i = 4 ; i < jointNum_ ; i ++ ) { 
    
      jointControlMethods_ [ i ] = POSITION_PID ; 
      
      hardware_interface 
       ::JointHandle 
       jointHandle ( jointStateInterface_ .getHandle ( jointNames_ [ i ] ) , 
                     & jointPositionCommand_ [ i ] ) ; 
                             
      positionJointInterface_ .registerHandle ( jointHandle ) ; 
    
    }
      
    registerInterface ( & jointStateInterface_ ) ; 
    registerInterface ( & positionJointInterface_ ) ; 
    registerInterface ( & velocityJointInterface_ ) ; 
    
  }
  
}  // namespace pandora_gazebo_interface
