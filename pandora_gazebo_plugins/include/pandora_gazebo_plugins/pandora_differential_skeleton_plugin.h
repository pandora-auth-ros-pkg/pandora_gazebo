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
#ifndef GAZEBO_ROS_DIFFERENTIAL_SKELETON_HH
#define GAZEBO_ROS_DIFFERENTIAL_SKELETON_HH

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

namespace gazebo
{
  class GazeboRosDifferentialSkeleton : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosDifferentialSkeleton();

    /// \brief Destructor
    public: virtual ~GazeboRosDifferentialSkeleton();

    /// \brief Load the controller
    /// \param node XML config node
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    /// \brief The parent World
    private: physics::WorldPtr world_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher joint_state_pub_;
    private: PubQueue<sensor_msgs::JointState>::Ptr joint_state_pub_Queue;
    
    /// \brief ros JointState message
    private: sensor_msgs::JointState joint_state_msg_;

    /// \brief A mutex to lock access to fields
    /// that are used in message callbacks
    private: boost::mutex lock_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;
    
    /// \brief call back when using service
    private: bool ServiceCallback(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);
                                  
    private: ros::ServiceServer srv_;
    private: ros::CallbackQueue callback_queue_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;
    
    private: gazebo::physics::JointPtr left_skeleton_joint_;
    private: gazebo::physics::JointPtr right_skeleton_joint_;
    private: gazebo::physics::ModelPtr model_;
    
    
  };
}
#endif


