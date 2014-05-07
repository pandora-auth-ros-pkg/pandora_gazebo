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
                                   physics ::ModelPtr parentModel , 
                                   const urdf ::Model * const urdfModel , 
                                   std 
                                    ::vector 
                                    < transmission_interface ::TransmissionInfo > 
                                    transmissions ) 
  
  {
  
    // Variable initialization
    registerInterfaces ( ) ; 

    // Get gazebo entities
    gazeboLink_ = parentModel ->GetLink ( imuData_ .frame_id ) ; 

    for ( unsigned int i = 0 ; i < this ->jointNames_ .size ( ) ; i ++ ) { 
    
      physics ::JointPtr joint = parentModel 
                                  ->GetJoint ( jointNames_ [ i ] ) ; 
                                   
      this ->gazeboJoints_ .push_back ( joint ) ; 
       
    }
    
  }

  GazeboInterface ::~GazeboInterface ( ) { 
  
  
  
  }

  void GazeboInterface ::readSim ( ros ::Time time , ros ::Duration period ) { 
  
    // Read robot orientation for IMU
    
    math ::Pose pose = gazeboLink_ ->GetWorldPose ( ) ; 
    
    imuOrientation [ 0 ] = pose .rot .x ; 
    imuOrientation [ 1 ] = pose .rot .y ; 
    imuOrientation [ 2 ] = pose .rot .z ; 
    imuOrientation [ 3 ] = pose .rot .w ; 
    
    for ( unsigned int i = 0 ; i < this ->jointNames_ .size ( ) ; i ++ ) { 
    
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
  
    this ->positionJointSaturationInterface_ .enforceLimits ( period ) ; 
    this ->positionJointLimitsInterface_ .enforceLimits ( period ) ; 
    this ->velocityJointSaturationInterface_ .enforceLimits ( period ) ; 
    this ->velocityJointLimitsInterface_ .enforceLimits ( period ) ; 
    
    for ( unsigned int i = 0 ; i < this ->jointNames_ .size ( ) ; i ++ ) { 
    
      switch ( jointControlMethods_ [ i ] ) { 

        case POSITION_PID: {
        
            double error ; 
            
            switch ( this ->jointTypes [ i ] ) {
            
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
                                        
            this ->gazeboJoints_ [ i ] ->SetForce ( 0 , effort ) ; 
            
        }
        
        break ; 

        case VELOCITY: 
        
          this ->gazeboJoints_ [ i ] ->SetVelocity ( 0 , jointCommand_ [ i ] ) ; 
          
        break ; 
          
      }
      
    }
    
  }
  
  void GazeboInterface ::registerInterfaces ( ) { 

    // Connect and register imu sensor interface
    
    imuOrientation [ 0 ] = 0 ; 
    imuOrientation [ 1 ] = 0 ; 
    imuOrientation [ 2 ] = 0 ; 
    imuOrientation [ 3 ] = 1 ; 
    
    imuData_. orientation = imuOrientation ; 
    imuData_ .name= "/sensors/imu" ; 
    imuData_ .frame_id = "base_link" ; 
    
    hardware_interface ::ImuSensorHandle imuSensorHandle ( imuData_ ) ; 
    imuSensorInterface_ .registerHandle ( imuSensorHandle ) ; 
    
    registerInterface ( & imuSensorInterface_ ) ; 

    // Connect and register the joint state interface
    
    this ->jointNames_ .push_back ( "/left_front_wheel_joint" ) ; 
    this ->jointTypes_ .push_back ( urdf::Joint::REVOLUTE ) ; 
    
    this ->jointNames_ .push_back ( "/left_rear_wheel_joint" ) ; 
    this ->jointTypes_ .push_back ( urdf::Joint::REVOLUTE ) ; 
    
    this ->jointNames_ .push_back ( "/right_front_wheel_joint" ) ; 
    this ->jointTypes_ .push_back ( urdf::Joint::REVOLUTE ) ; 
    
    this ->jointNames_ .push_back ( "/right_rear_wheel_joint" ) ; 
    this ->jointTypes_ .push_back ( urdf::Joint::REVOLUTE ) ; 
    
    for ( unsigned int i = 0 ; i < this ->jointNames_ .size ( ) ; i ++ ) { 
      
      hardware_interface 
       ::JointStateHandle jointStateHandle ( this ->jointNames_ [ i ] , 
                                             & this ->jointPosition_ [ i ] , 
                                             & this ->jointVelocity_ [ i ] , 
                                             & this ->jointEffort_ [ i ] ) ; 
                                             
      this ->jointStateInterface_ .registerHandle ( jointStateHandle ) ; 
      
    }
      
    registerInterface ( & this ->jointStateInterface_ ) ; 

    // Connect and register the joint velocity interface
      
    for ( unsigned int i = 0 ; i < 4 ; i ++ ) { 
      
      hardware_interface 
       ::JointStateHandle 
       jointVelocityHandle ( this ->jointStateInterface_ 
                                     .getHandle ( this ->jointNames_ [ i ] ) , 
                             & this ->jointCommand_ [ i ] ) ; 
                             
      this ->velocityJointInterface_ .registerHandle ( jointVelocityHandle ) ; 
    
    }
      
    registerInterface ( & this ->velocityJointInterface_ ) ; 

    /*

    // Connect and register the joint position interface
      
    for ( unsigned int i = 4 ; i < this ->jointNames_ .size ( ) ; i ++ ) { 
      
      hardware_interface 
       ::JointStateHandle 
       jointPositionHandle ( this ->jointStateInterface_ 
                                     .getHandle ( this ->jointNames_ [ i ] ) , 
                             & this ->jointCommand_ [ i ] ) ; 
                             
      this ->positionJointInterface_ .registerHandle ( jointPositionHandle ) ; 
    
    }
      
    registerInterface ( & this ->positionJointInterface_ ) ; 
    
    */
    
  }
  
}  // namespace pandora_gazebo_interface
