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

namespace gazebo { 

  class GazeboRosDifferential : public ModelPlugin { 
  
    /// \brief Constructor
    public: GazeboRosDifferential ( void ) ; 

    /// \brief Destructor
    public: virtual ~GazeboRosDifferential ( void ) ; 

    /// \brief Load the controller
    /// \param Node XML config node
    public: void Load ( physics ::ModelPtr _parent , sdf ::ElementPtr _sdf ) ; 

    /// \brief Update the controller
    protected: virtual void UpdateChild ( void ) ; 

    /// \brief The parent World
    private: physics ::WorldPtr world_ ; 

    /// \brief Pointer to ros node
    private: ros ::NodeHandle * rosnode_ ; 
    private: ros ::Publisher joint_state_pub_ ; 
    private: PubQueue < sensor_msgs ::JointState > ::Ptr joint_state_pub_Queue ; 
    
    /// \brief ROS JointState message
    private: sensor_msgs ::JointState joint_state_msg_ ; 
    
    /// \brief Publishes the joint states to ROS
    private: void PublishJointStates ( void ) ; 

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost ::mutex lock_ ; 

    /// \brief For setting ROS name space
    private: std ::string robot_namespace_ ; 
    
    /// \brief Call back when using service
    private: bool ServiceCallback ( std_srvs ::Empty ::Request & req , 
                                    std_srvs ::Empty ::Response & res ) ; 
                                  
    private: ros ::ServiceServer srv_ ; 
    private: ros ::CallbackQueue callback_queue_ ; 

    // Pointer to the update event connection
    private: event ::ConnectionPtr update_connection_ ; 

    // Deferred load in case ros is blocking
    private: sdf ::ElementPtr sdf ; 
    private: void LoadThread ( void ) ; 
    private: boost ::thread deferred_load_thread_ ; 
    private: unsigned int seed ; 

    // ROS publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq ; 
    
    /// \brief Implements the PID Algorithm
    private: double PIDAlgorithm ( void ) ; 
    
    // Get the PID parameters
    private: double GetP ( void ) ; 
    private: double GetI ( void ) ; 
    private: double GetD ( void ) ; 
    
    private: double previous_error_ ; 
    private: double integral_ ; 
    
    // Robot's model
    private: gazebo ::physics ::ModelPtr model_ ; 
    
    // Robot's links
    private: gazebo ::physics ::LinkPtr base_link_ ; 
    private: gazebo ::physics ::LinkPtr left_front_wheel_link_ ; 
    private: gazebo ::physics ::LinkPtr left_rear_wheel_link_ ; 
    private: gazebo ::physics ::LinkPtr right_front_wheel_link_ ; 
    private: gazebo ::physics ::LinkPtr right_rear_wheel_link_ ; 
    
    // Robot's joints
    private: gazebo ::physics ::JointPtr left_side_joint_ ; 
    private: gazebo ::physics ::JointPtr right_side_joint_ ; 
    
    /// \brief Updates the angles of the side joints
    private: void UpdateAngles ( void ) ; 
    
    private: double left_angle_ ; 
    private: double right_angle_ ; 
    private: double max_angle_ ; 
    
    // Add the forces manually
    private: void AddSideForces ( void ) ; 
    private: void AddDownforces ( void ) ; 
    
    // Get the maximum downforce and the side joint damping
    private: double GetMaxDownforce ( void ) ; 
    private: double GetSideJointDamping ( void ) ; 
    
    /// \brief Adds force at the base link to correct its angle
    private: void AddCorrectionForce ( void ) ; 
    
    // Get the force adjustment
    private: double GetForceAdjustment ( void ) ; 
    private: gazebo ::math ::Vector3 correction_force_ ; 
    
    /// \brief Get the real time update rate of the physics engine
    private: double GetUpdateRate ( void ) ; 
    
  } ; 
  
}

#endif


