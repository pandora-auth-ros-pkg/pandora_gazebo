/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#include "pandora_gazebo_plugins/pandora_differential_plugin.h"

namespace gazebo { 

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN ( GazeboRosDifferential ) 

  /////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosDifferential ::GazeboRosDifferential ( void ) { 
  
    this ->seed = 0 ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosDifferential ::~GazeboRosDifferential ( void ) { 
  
    event ::Events ::DisconnectWorldUpdateBegin ( this ->update_connection_ ) ; 
  
    // Finalize the controller
    this ->rosnode_ ->shutdown ( ) ; 
    delete this ->rosnode_ ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosDifferential ::Load ( physics ::ModelPtr _parent , 
                                      sdf ::ElementPtr _sdf ) { 
  
    // Save pointers
    this ->world_ = _parent ->GetWorld ( ) ; 
    this ->sdf = _sdf ; 

    // ROS callback queue for processing subscription
    this ->deferred_load_thread_ = 
    boost ::thread ( boost ::bind ( & GazeboRosDifferential ::LoadThread , 
                                    this ) ) ; 
    
    this ->model_ = _parent ; 
    
  }

  /////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosDifferential ::LoadThread ( void ) { 
  
    // Load parameters
    this ->robot_namespace_ = "" ; 
  
    if ( this ->sdf ->HasElement ( "robotNamespace" ) ) 
  
      this ->robot_namespace_ = this ->sdf ->Get < std ::string > 
                                           ( "robotNamespace" ) 
                                + "/" ; 
  
    if ( ! this ->sdf ->HasElement ( "baseLink" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <baseLink>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
    
      this ->base_link_ = this ->model_ 
                                ->GetLink ( this ->sdf 
                                                  ->Get < std ::string > 
                                                    ( "baseLink" ) ) ; 
    
    }
  
    if ( ! this ->sdf ->HasElement ( "leftFrontWheelLink" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <leftFrontWheelLink>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
    
      this ->left_front_wheel_link_ = 
      this ->model_ ->GetLink ( this ->sdf ->Get < std ::string > 
                                             ( "leftFrontWheelLink" ) ) ; 
    
    }
  
    if ( ! this ->sdf ->HasElement ( "leftRearWheelLink" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <leftRearWheelLink>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
    
      this ->left_rear_wheel_link_ = 
      this ->model_ ->GetLink ( this ->sdf ->Get < std ::string > 
                                             ( "leftRearWheelLink" ) ) ; 
    
    }
  
    if ( ! this ->sdf ->HasElement ( "rightFrontWheelLink" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <rightFrontWheelLink>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
    
      this ->right_front_wheel_link_ = 
      this ->model_ ->GetLink ( this ->sdf ->Get < std ::string > 
                                             ( "rightFrontWheelLink" ) ) ; 
    
    }
  
    if ( ! this ->sdf ->HasElement ( "rightRearWheelLink" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <rightRearWheelLink>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
    
      this ->right_rear_wheel_link_ = 
      this ->model_ ->GetLink ( this ->sdf ->Get < std ::string > 
                                             ( "rightRearWheelLink" ) ) ; 
    
    }

    if ( ! this ->sdf ->HasElement ( "leftSideJoint" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <leftSideJoint>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
  
      this ->left_side_joint_ = 
      this ->model_ 
            ->GetJoint ( this ->sdf 
                               -> Get < std ::string > ( "leftSideJoint" ) ) ; 
    
      joint_state_msg_ 
       .name.push_back ( this ->sdf 
                               -> Get < std ::string > ( "leftSideJoint" ) ) ; 
    
    }

    if ( ! this ->sdf ->HasElement ( "rightSideJoint" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <rightSideJoint>, exiting" ) ; 
    
      return ; 
    
    }
  
    else { 
  
      this ->right_side_joint_ = 
      this ->model_ 
            ->GetJoint ( this ->sdf 
                               -> Get < std ::string > ( "rightSideJoint" ) ) ; 
    
      joint_state_msg_ 
       .name.push_back ( this ->sdf 
                               -> Get < std ::string > ( "rightSideJoint" ) ) ; 
    
    }
    
    if ( ! this ->sdf ->HasElement ( "maxAngle" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <maxAngle>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
  
      this ->max_angle_ = this ->sdf ->Get < double > ( "maxAngle" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "maxDownforce" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <maxDownforce>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->max_downforce_ = this ->sdf ->Get < double > ( "maxDownforce" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "maxDifferentialForceZ" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <maxDifferentialForceZ>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->max_differential_force_z_ = 
      this ->sdf ->Get < double > ( "maxDifferentialForceZ" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "maxDifferentialForceY" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <maxDifferentialForceY>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->max_differential_force_y_ = 
      this ->sdf ->Get < double > ( "maxDifferentialForceY" ) ; 
      
    }
    
    // TODO: Get the value directly from the joints.
    if ( ! this ->sdf ->HasElement ( "sideJointDamping" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <sideJointDamping>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->side_joint_damping_ = 
      this ->sdf ->Get < double > ( "sideJointDamping" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "correctionForceModifier" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <correctionForceModifier>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->correction_force_modifier_ = 
      this ->sdf ->Get < double > ( "correctionForceModifier" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "P" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <P>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->k_p_ = this ->sdf ->Get < double > ( "P" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "I" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <I>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->k_i_ = this ->sdf ->Get < double > ( "I" ) ; 
      
    }
    
    if ( ! this ->sdf ->HasElement ( "D" ) ) { 
  
      ROS_FATAL ( "Differential plugin missing <D>, exiting" ) ; 
    
      return ; 
    
    }
    
    else { 
                                 
      this ->k_d_ = this ->sdf ->Get < double > ( "D" ) ; 
      
    }
    
    // Initialize the correction force
    this ->correction_force_ = gazebo ::math ::Vector3 ( 0 , 0 , 0 ) ; 
  
    // Initialize the variables used in the PID algorithm
    this ->previous_error_ = 0.0 ; 
    this ->integral_ = 0.0 ; 
    
    // Make sure the ROS node for Gazebo has already been initialized
    if ( ! ros ::isInitialized ( ) ) { 
  
      ROS_FATAL_STREAM ( "A ROS node for Gazebo has not been initialized, unable to load plugin. " 
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)" ) ; 
    
      return ; 
    
    }

    this ->rosnode_ = new ros ::NodeHandle ( this ->robot_namespace_ ) ; 

    // Publish multi queue
    this ->pmq .startServiceThread ( ) ; 

    this ->joint_state_pub_Queue = 
    this ->pmq .addPub < sensor_msgs ::JointState > ( ) ; 
                     
    this ->joint_state_pub_ = this ->rosnode_ 
                                    ->advertise < sensor_msgs ::JointState > 
                                      ( "differential_side_joint_states" , 1 ) ; 

    // Advertise services on the custom queue
    ros ::AdvertiseServiceOptions aso =
    ros ::AdvertiseServiceOptions ::create < std_srvs ::Empty > 
                                  ( "/differantial_side" , 
                                    boost ::bind ( & GazeboRosDifferential 
                                                      ::ServiceCallback , 
                                                   this , 
                                                   _1 , 
                                                   _2 ) , 
                                    ros ::VoidPtr ( ) , 
                                    & this ->callback_queue_ ) ; 
                    
    this ->srv_ = this ->rosnode_ ->advertiseService ( aso ) ; 
    
    dynamic_reconfigure 
     ::Server < pandora_gazebo_plugins ::DifferentialConfig > config_callback ; 
     
    config_callback = boost ::bind ( & GazeboRosDifferential ::ConfigCallback , 
                                     _1 ,
                                     _2 ) ; 
                                   
    this ->config_server_ .setCallback ( config_callback ) ;  
  
    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this ->update_connection_ = event 
                                 ::Events 
                                  ::ConnectWorldUpdateBegin 
                                    ( boost ::bind ( & GazeboRosDifferential 
                                                        ::UpdateChild , 
                                                     this ) ) ; 
                                                     
  }

  /////////////////////////////////////////////////////////////////////////////
  // Returns true always
  bool GazeboRosDifferential ::ServiceCallback 
                               ( std_srvs ::Empty ::Request & req , 
                                 std_srvs ::Empty ::Response & res ) { 
                                 
    return true ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Reconfigures the variables when the dynamic_reconfigure 
  // sents a new configuration
  void GazeboRosDifferential ::ConfigCallback 
                               ( pandora_gazebo_plugins 
                                  ::DifferentialConfig & config , 
                                 uint32_t level ) { 
                                 
    this ->max_angle_                 = config .maxAngle ; 
    this ->side_joint_damping_        = config .sideJointDamping ; 
    this ->max_downforce_             = config .maxDownforce ; 
    this ->max_differential_force_z_  = config .maxDifferentialForceZ ; 
    this ->max_differential_force_y_  = config .maxDifferentialForceY ; 
    this ->k_p_                       = config .P ; 
    this ->k_i_                       = config .I ; 
    this ->k_d_                       = config .D ; 
    this ->correction_force_modifier_ = config .correctionForceModifier ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Returns the real time update rate of the physics engine
  double GazeboRosDifferential ::GetUpdateRate ( void ) { 
                                 
    return ( this ->world_ ->GetPhysicsEngine ( ) ->GetRealTimeUpdateRate ( ) ) ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Updates the angles of the side joints
  void GazeboRosDifferential ::UpdateAngles ( void ) { 
  
    this ->left_angle_ = this ->left_side_joint_ ->GetAngle ( 0 ) .Radian ( ) ; 
    this ->right_angle_ = this ->right_side_joint_ ->GetAngle ( 0 ) .Radian ( ) ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Publishes joint states
  void GazeboRosDifferential ::PublishJointStates ( void ) { 
  
    this ->joint_state_msg_ .header .stamp = ros ::Time ::now ( ) ; 
    this ->joint_state_msg_ .position .clear ( ) ; 
    this ->joint_state_msg_ .position .push_back ( this ->left_angle_ ) ; 
    this ->joint_state_msg_ .position .push_back ( this ->right_angle_ ) ; 
  
    {
  
      boost ::mutex ::scoped_lock lock ( this ->lock_ ) ; 
    
      // Publish to ROS
      this ->joint_state_pub_Queue
            ->push ( this ->joint_state_msg_ , this ->joint_state_pub_ ) ; 
    
    }
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Adds forces at the rear wheels in y and z axis due to differential activity
  void GazeboRosDifferential ::AddDifferentialForces ( void ) { 

    // Initialize the forces to be set
    math ::Vector3 left_rear_force ( 0 , 0 , 0 ) ; 
    math ::Vector3 right_rear_force ( 0 , 0 , 0 ) ; 
  
    // Calculate and normalize the positive angle difference
    double angle_difference = fabs ( fabs ( this ->left_angle_ ) - 
                                     fabs ( this ->right_angle_ ) ) ; 
                                     
    angle_difference /= this ->max_angle_ ; 
  
    // Calculate the forces for each link
    // TODO: Test if the force are applied with the correct sign.
    if ( this ->left_angle_ > this ->right_angle_ ) { 
  
      left_rear_force .z = this ->max_differential_force_z_ * 
                           angle_difference * 
                           ( - 1 ) ; 
  
      left_rear_force .y = this ->max_differential_force_y_ * 
                           angle_difference ; 
                                
    }
    
    if ( this ->left_angle_ < this ->right_angle_ ) { 
  
      right_rear_force .z = this ->max_differential_force_z_ * 
                            angle_difference * 
                            ( - 1 ) ; 
  
      right_rear_force .y = this ->max_differential_force_y_ * 
                            angle_difference * 
                            ( - 1 ) ; 
    
    }
    
    // Set the forces to the wheel links
    this ->left_rear_wheel_link_ ->AddForce ( left_rear_force ) ; 
    this ->right_rear_wheel_link_ ->AddForce ( right_rear_force ) ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Adds downforces at the wheels due to the side joint damping
  // TODO: Implement maximum side joint damping and normalize side joint damping.
  void GazeboRosDifferential ::AddDownforces ( void ) { 
  
    // Get the linear velocity of each link in the z axis in ( mm / sec )
    double left_front_z_vel = 1000.0 * left_front_wheel_link_ 
                                        ->GetWorldLinearVel ( ) 
                                         .z ; 
    double left_rear_z_vel = 1000.0 * left_rear_wheel_link_ 
                                       ->GetWorldLinearVel ( ) 
                                        .z ; 
    double right_front_z_vel = 1000.0 * right_front_wheel_link_ 
                                         ->GetWorldLinearVel ( ) 
                                          .z ; 
    double right_rear_z_vel = 1000.0 * right_rear_wheel_link_ 
                                        ->GetWorldLinearVel ( ) 
                                         .z ; 

    // Initialize the downforces to be set
    math ::Vector3 left_front_downforce ( 0 , 0 , 0 ) ; 
    math ::Vector3 left_rear_downforce ( 0 , 0 , 0 ) ; 
    math ::Vector3 right_front_downforce ( 0 , 0 , 0 ) ; 
    math ::Vector3 right_rear_downforce ( 0 , 0 , 0 ) ; 
  
    // Calculate the downforce for each link
    if ( left_front_z_vel < 0 ) 
  
      left_front_downforce .z = this ->max_downforce_ * 
                                this ->side_joint_damping_ * 
                                left_front_z_vel ; 
    
    if ( left_rear_z_vel < 0 ) 
  
      left_rear_downforce .z = this ->max_downforce_ * 
                                this ->side_joint_damping_ * 
                               left_rear_z_vel ; 
    
    if ( right_front_z_vel < 0 ) 
  
      right_front_downforce .z = this ->max_downforce_ * 
                                this ->side_joint_damping_ * 
                                 right_front_z_vel ; 
    
    if ( right_rear_z_vel < 0 ) 
  
      right_rear_downforce .z = this ->max_downforce_ * 
                                this ->side_joint_damping_ * 
                                right_rear_z_vel ; 
  
    // Set the downforces to the wheel links
    this ->left_front_wheel_link_ ->AddForce ( left_front_downforce ) ; 
    this ->left_rear_wheel_link_ ->AddForce ( left_rear_downforce ) ; 
    this ->right_front_wheel_link_ ->AddForce ( right_front_downforce ) ; 
    this ->right_rear_wheel_link_ ->AddForce ( right_rear_downforce ) ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Calculates the output after the PID control
  double GazeboRosDifferential ::PIDAlgorithm ( void ) { 
    
    // Calculate the error
    double error = ( ( this ->left_angle_ + this ->right_angle_ ) / 2 ) ; 
    //double error = ( left_angle_abs - right_angle_abs ) ; 
  
    // Normalize the error
    error /= this ->max_angle_ ; 
  
    double dt = ( 1.0 / GazeboRosDifferential ::GetUpdateRate ( ) ) ; 
  
    this ->integral_ += ( error * dt ) ; 
  
    double derivative = ( ( error - this ->previous_error_ ) / dt ) ; 
  
    // Calculate the output
    double output = ( ( this ->k_p_ * error ) + 
                      ( this ->k_i_ * this ->integral_ ) + 
                      ( this ->k_d_ * derivative ) ) ; 
    /*
    double output = ( error + this ->integral_ / t_i + t_d * derivative ) * 
                    this ->k_p_ ; 
                    
    double output = ( error + this ->integral_ / t_i + t_d * derivative ) / 
                    this ->k_p_ ; 
    */
    
    this ->previous_error_ = error ; 
    
    return output ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Calculates and sets the correction force to the base link
  void GazeboRosDifferential ::AddCorrectionForce ( void ) { 
    
    // Calculate the force to be set
    this ->correction_force_ .x = ( this ->correction_force_modifier_ / 100 ) * 
                                    GazeboRosDifferential ::PIDAlgorithm ( ) ; 
    
    // Set the correction force to the base link
    this ->base_link_ ->AddForce ( this ->correction_force_ ) ; 
  
  }

  /////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosDifferential ::UpdateChild ( void ) { 
  
    GazeboRosDifferential ::UpdateAngles ( ) ; 
    
    GazeboRosDifferential ::AddCorrectionForce ( ) ; 
  
    //GazeboRosDifferential ::AddDownforces ( ) ; 
    
    //GazeboRosDifferential ::AddDifferentialForces ( ) ; 
    
    //ROS_INFO ( "Error = %f" , this ->previous_error_ ) ; 
    //ROS_INFO ( "Correction force = %f" , this ->correction_force_ .x ) ; 
    //ROS_INFO ( "-----------------------" ) ; 
    
    GazeboRosDifferential ::PublishJointStates ( ) ; 
  
  }

}
