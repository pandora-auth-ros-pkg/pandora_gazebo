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

#include "pandora_differential_skeleton_plugin.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDifferentialSkeleton)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosDifferentialSkeleton::GazeboRosDifferentialSkeleton()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosDifferentialSkeleton::~GazeboRosDifferentialSkeleton()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDifferentialSkeleton::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // save pointers
  this->world_ = _parent->GetWorld();
  this->sdf = _sdf;

  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosDifferentialSkeleton::LoadThread, this));
    
  this->model_ = _parent;
    
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosDifferentialSkeleton::LoadThread()
{
  // load parameters
  this->robot_namespace_ = "";
  if (this->sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf->Get<std::string>("robotNamespace") + "/";

  if (!this->sdf->HasElement("leftSkeletonJoint"))
  {
    ROS_FATAL("differential plugin missing <leftSkeletonJoint>, exiting");
    return;
  }
  else
  {
    this->left_skeleton_joint_ = this->model_->GetJoint(this->sdf->Get<std::string>("leftSkeletonJoint"));
    joint_state_msg_.name.push_back(this->sdf->Get<std::string>("leftSkeletonJoint"));
  }
    
  if (!this->sdf->HasElement("rightSkeletonJoint"))
  {
    ROS_FATAL("differential plugin missing <rightSkeletonJoint>, exiting");
    return;
  }
  else
  {
    this->right_skeleton_joint_  = this->model_->GetJoint(this->sdf->Get<std::string>("rightSkeletonJoint"));
    joint_state_msg_.name.push_back(this->sdf->Get<std::string>("rightSkeletonJoint"));
  }
    

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  this->joint_state_pub_Queue = this->pmq.addPub<sensor_msgs::JointState>();
  this->joint_state_pub_ = this->rosnode_->advertise<sensor_msgs::JointState>(
    "differential_skeleton_joint_states", 1);

  // advertise services on the custom queue
  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "/differantial_skeleton", boost::bind(&GazeboRosDifferentialSkeleton::ServiceCallback,
    this, _1, _2), ros::VoidPtr(), &this->callback_queue_);
  this->srv_ = this->rosnode_->advertiseService(aso);
  

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosDifferentialSkeleton::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// returns true always
bool GazeboRosDifferentialSkeleton::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosDifferentialSkeleton::UpdateChild()
{    
  double left_angle = left_skeleton_joint_->GetAngle(0).Radian();
  double right_angle = right_skeleton_joint_->GetAngle(0).Radian();
  double angle_diff = fabs(left_angle) - fabs(right_angle);
  
  //publish joint states
  joint_state_msg_.header.stamp = ros::Time::now();
  joint_state_msg_.position.clear();
  joint_state_msg_.position.push_back(left_angle);
  joint_state_msg_.position.push_back(right_angle);
  
  {
    boost::mutex::scoped_lock lock(this->lock_);
    // publish to ros
    this->joint_state_pub_Queue->push(this->joint_state_msg_, this->joint_state_pub_);
  }
  
  if( (left_angle * right_angle ) > 0 )
  {
    right_skeleton_joint_->SetForce(0,-copysign(20,right_angle));
    left_skeleton_joint_->SetForce(0,-copysign(20,left_angle));
  }
  else
  {
    if(angle_diff > 0)
    {
      right_skeleton_joint_->SetForce(0,-copysign(5,left_angle));
      left_skeleton_joint_->SetForce(0,-copysign(1,right_angle));
    }
    else
    {
      left_skeleton_joint_->SetForce(0,-copysign(5,right_angle));
      right_skeleton_joint_->SetForce(0,-copysign(1,left_angle));
    }
  }
}

}
