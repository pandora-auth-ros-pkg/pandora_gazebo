/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_DIFFERENTIAL_HH
#define GAZEBO_ROS_DIFFERENTIAL_HH

#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/PubQueue.h>

#include <dynamic_reconfigure/server.h>
#include <pandora_gazebo_plugins/DifferentialConfig.h>

namespace gazebo { 

  class GazeboRosDifferential : public ModelPlugin { 
  
    /// \brief Constructor
    public: GazeboRosDifferential ( void ) ; 

    /// \brief Destructor
    public: virtual ~GazeboRosDifferential ( void ) ; 

    /// \brief Load the controller
    /// \param Node XML config node
    public: void Load ( physics ::ModelPtr _parent , sdf ::ElementPtr _sdf ) ; 
    
    /// \brief Load parameters from sdf
    private: bool LoadParameters ( void ) ; 
    
    /// \brief Load the controller
    private: void LoadThread ( void ) ; 
    
    /// \brief Initialize the dynamic reconfigure services
    private: void LoadReconfigureThread ( void ) ; 
    
    /// \brief Callback when using service
    private: bool ServiceCallback ( std_srvs ::Empty ::Request & req , 
                                    std_srvs ::Empty ::Response & res ) ; 
    
    /// \brief Callback which reconfigures the variables when the 
    ///        dynamic_reconfigure sents a new configuration
    private: void ConfigCallback ( pandora_gazebo_plugins 
                                    ::DifferentialConfig & config , 
                                   uint32_t level ) ; 

    /// \brief Update the controller
    protected: virtual void UpdateChild ( void ) ; 
    
    /// \brief Publishes the joint states to ROS
    private: void PublishJointStates ( void ) ; 
    
    /// \brief Get the real time update rate of the physics engine
    private: double GetUpdateRate ( void ) ; 
    
    /// \brief Implements the PID Algorithm
    private: double PIDAlgorithm ( double error , 
                                   double & previous_error , 
                                   double & integral , 
                                   double k_p , 
                                   double k_i , 
                                   double k_d , 
                                   double i_clamp_min , 
                                   double i_clamp_max ) ; 
    private: double PIDAlgorithm ( void ) ; 
    
    /// \brief Update the angles of the side joints
    private: void UpdateAngles ( void ) ; 
    
    /// \brief Add forces at the rear wheels in y and z axis due to 
    ///        differential activity
    private: void AddDifferentialForces ( void ) ; 
    
    /// \brief Add downforces at the wheels due to the side joint damping
    private: void AddDownforces ( void ) ; 
    
    /// \brief Add force at the base link to correct its angle
    private: void AddBaseCorrectionForce ( void ) ; 
    
    /// \brief Add force at the side joints to correct base's angle
    private: void AddSideCorrectionForce ( void ) ; 
    private: void AddSideCorrectionForce2 ( void ) ; 

    /// \brief Pointer to ros node
    private: ros ::NodeHandle * rosnode_ ; 
    private: ros ::Publisher joint_state_pub_ ; 
    private: PubQueue < sensor_msgs ::JointState > ::Ptr joint_state_pub_Queue ; 

    // ROS publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq ; 

    // Pointer to the update event connection
    private: event ::ConnectionPtr update_connection_ ; 
    
    private: ros ::ServiceServer srv_ ; 
    private: ros ::CallbackQueue callback_queue_ ; 

    /// \brief A mutex to lock access to fields
    ///        that are used in message callbacks
    private: boost ::mutex lock_ ; 
    
    /// \brief ROS JointState message
    private: sensor_msgs ::JointState joint_state_msg_ ; 

    /// \brief For setting ROS name space
    private: std ::string robot_namespace_ ; 
    
    private: boost ::thread deferred_load_thread_ ; 
    private: unsigned int seed ; 
    
    // Dynamic reconfigure
    private: boost ::shared_ptr < boost ::thread > reconfigure_thread_ ; 
    
    private: boost ::shared_ptr 
             < dynamic_reconfigure ::Server 
              < pandora_gazebo_plugins ::DifferentialConfig > > 
              reconfigure_srv_ ; 
             
    private: dynamic_reconfigure ::Server 
             < pandora_gazebo_plugins ::DifferentialConfig > 
              ::CallbackType reconfigure_callback_ ; 

    // Deferred load in case ros is blocking
    private: sdf ::ElementPtr sdf ; 

    /// \brief The parent World
    private: physics ::WorldPtr world_ ; 
    
    /// \brief Robot's model
    private: physics ::ModelPtr model_ ; 
    
    // Robot's links
    private: physics ::LinkPtr base_link_ ; 
    private: physics ::LinkPtr left_front_wheel_link_ ; 
    private: physics ::LinkPtr left_rear_wheel_link_ ; 
    private: physics ::LinkPtr right_front_wheel_link_ ; 
    private: physics ::LinkPtr right_rear_wheel_link_ ; 
    
    // Robot's joints
    private: physics ::JointPtr left_side_joint_ ; 
    private: physics ::JointPtr right_side_joint_ ; 
    
    // Joint angles and maximum angle
    private: double left_angle_ ; 
    private: double right_angle_ ; 
    private: double max_angle_ ; 
    
    // Maximum manual forces and side joint damping
    private: double max_downforce_ ; 
    private: double max_differential_force_z_ ; 
    private: double max_differential_force_y_ ; 
    private: double side_joint_damping_ ; 
    
    // PID parameters
    private: double k_p_ ; 
    private: double k_i_ ; 
    private: double k_d_ ; 
    
    // Stored variables of the PID algorithm
    private: double previous_error_ ; 
    private: double integral_ ; 
    
    // Correction force modifier and correction force
    private: double correction_force_modifier_ ; 
    
  } ; 
  
}

#endif


