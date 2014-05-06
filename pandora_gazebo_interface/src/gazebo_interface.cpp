/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  Evangelos Apostolidis
*********************************************************************/
#include "pandora_gazebo_interface/gazebo_interface.h"

namespace pandora_gazebo_interface
{
  bool GazeboInterface::initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // Variable initialization ?
    registerInterfaces();

    // Get gazebo entities
    gazeboLink_ = parent_model->GetLink(imuData_.frame_id);

    //~ for (int ii = 0; ii < jointNames_.size(); ii++)
    //~ {
      //~ gazebo::physics::JointPtr joint = parent_model->GetJoint(jointNames_[ii]);
      //~ gazeboJoints_.push_back(joint);
    //~ }
  }

  GazeboInterface::~GazeboInterface()
  {
  }

  void GazeboInterface::readSim(ros::Time time, ros::Duration period)
  {
    // Read robot orientation for IMU
    gazebo::math::Pose pose;
    pose = gazeboLink_->GetWorldPose();
    imuOrientation[0] = pose.rot.x;
    imuOrientation[1] = pose.rot.y;
    imuOrientation[2] = pose.rot.z;
    imuOrientation[3] = pose.rot.w;

    //~ for (int ii = 0; ii < 8; ii++)
    //~ {
      //~ jointPosition_[ii] +=
        //~ angles::shortest_angular_distance(
          //~ jointPosition_[ii],
          //~ gazeboJoints_[ii]->GetAngle(0).Radian());
      //~ jointVelocity_[ii] = gazeboJoints_[ii]->GetVelocity(0);
      //~ jointEffort_[ii] = gazeboJoints_[ii]->GetForce((unsigned int)(0));
    //~ }
  }

  void GazeboInterface::writeSim(ros::Time time, ros::Duration period)
  {
    //~ for (int ii = 0; ii < 3; ii++)
    //~ {
      //~ gazeboJoints_[ii]->SetVelocity(0, jointCommand_[ii]);
    //~ }
    //~ for (int ii = 4; ii < 8; ii++)
    //~ {
      //~ gazeboJoints_[ii]->SetAngle(0, jointCommand_[ii]);
    //~ }
  }

  std::vector<std::string> GazeboInterface::getJointNameFromParamServer()
  {
    std::vector<std::string> jointNames;
    std::string name;
    nodeHandle_.getParam(
      "/motor_joints/robot_movement_joints/left_front_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/robot_movement_joints/right_front_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/robot_movement_joints/left_rear_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/robot_movement_joints/right_rear_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/stabilizer_joints/pitch_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/stabilizer_joints/roll_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/kinect_orientation_joints/pitch_joint",
      name);
    jointNames.push_back(name);
    nodeHandle_.getParam(
      "/motor_joints/kinect_orientation_joints/yaw_joint",
      name);
    jointNames.push_back(name);

    return jointNames;
  }

  void GazeboInterface::registerInterfaces()
  {
    // connect and register imu sensor interface
    imuOrientation[0] = 0;
    imuOrientation[1] = 0;
    imuOrientation[2] = 0;
    imuOrientation[3] = 1;
    imuData_.orientation = imuOrientation;
    imuData_.name="/sensors/imu";
    imuData_.frame_id="base_link";
    hardware_interface::ImuSensorHandle imuSensorHandle(imuData_);
    imuSensorInterface_.registerHandle(imuSensorHandle);
    registerInterface(&imuSensorInterface_);

    // connect and register the joint state interface
    jointNames_ = getJointNameFromParamServer();
    //~ for (int ii = 0; ii < jointNames_.size(); ii++)
    //~ {
      //~ hardware_interface::JointStateHandle jointStateHandle(
        //~ jointNames_[ii],
        //~ &jointPosition_[ii],
        //~ &jointVelocity_[ii],
        //~ &jointEffort_[ii]);
      //~ jointStateInterface_.registerHandle(jointStateHandle);
    //~ }
    //~ registerInterface(&jointStateInterface_);

    // connect and register the joint velocity interface
    //~ for (int ii = 0; ii < 4; ii++)
    //~ {
      //~ hardware_interface::JointHandle jointVelocityHandle(
        //~ jointStateInterface_.getHandle(jointNames_[ii]),
        //~ &jointCommand_[ii]);
      //~ velocityJointInterface_.registerHandle(jointVelocityHandle);
    //~ }
    //~ registerInterface(&velocityJointInterface_);

    // connect and register the joint position interface
    //~ for (int ii = 4; ii < jointNames_.size(); ii++)
    //~ {
      //~ hardware_interface::JointHandle jointPositionHandle(
        //~ jointStateInterface_.getHandle(jointNames_[ii]),
        //~ &jointCommand_[ii]);
      //~ positionJointInterface_.registerHandle(jointPositionHandle);
    //~ }
    //~ registerInterface(&positionJointInterface_);
  }
}  // namespace pandora_gazebo_interface
