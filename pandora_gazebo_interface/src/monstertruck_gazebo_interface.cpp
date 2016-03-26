/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, P.A.N.D.O.R.A. Team.
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
 * Author: Geromichalos Dimitrios Panagiotis <geromidg@gmail.com>
 * Author: George Kouros <gkoursog@yahoo.gr>
 *********************************************************************/

#include "pandora_gazebo_interface/monstertruck_gazebo_interface.h"

namespace
{

  void clamp(
      double& val,
      const double min_val,
      const double max_val)
  {
    val = std::min(std::max(val, min_val), max_val);
  }

}  // namespace

namespace pandora_gazebo_interface
{

  MonstertruckGazeboInterface::~MonstertruckGazeboInterface()
  {
  }

  bool MonstertruckGazeboInterface::initSim(
    const std::string& robotnamespace,
    ros::NodeHandle modelNh,
    gazebo::physics::ModelPtr parentModel,
    const urdf::Model* const urdfModel,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    robotnamespace_ = robotnamespace;
    modelNh_ = modelNh;
    parentModel_ = parentModel;
    urdfModel_ = urdfModel;
    transmissions_ = transmissions;
    world_ = parentModel_->GetWorld();

    // Physical properties
    wheelRadius_ = 0.075;
    wheelbase_ = 0.32;
    wheelTrack_ = 0.26;

    // The rate that each element gets updated
    jointReadRate_ = gazebo::common::Time(1 / 100.0);
    jointWriteRate_ = gazebo::common::Time(1 / 100.0);
    imuReadRate_ = gazebo::common::Time(1 / 50.0);

    gazebo::common::Time simTime = world_->GetSimTime();
    jointLastReadTime_ = simTime;
    jointLastWriteTime_ = simTime;
    imuLastReadTime_ = simTime;


    // Register each hardware_interface
    registerJointInterface();
    registerImuInterface();

    // initialize joint position, velocity, effort and command variables
    for (int i = 0; i < jointNum_; i++)
    {
      jointPosition_[i] = 0;
      jointVelocity_[i] = 0;
      jointEffort_[i] = 0;
      jointCommand_[i] = 0;
    }

    // Load gazebo joints
    for (unsigned int i = 0; i < jointNum_; i++)
    {
      gazeboJoint_[i] = parentModel_->GetJoint(jointName_[i]);
    }

    // Load PID controllers and initialize/set the limits
    for (unsigned int i = 0; i < jointNum_; i++)
    {
      switch (jointControlMethod_[i])
      {
        case POSITION_PID:
        {
          // const ros::NodeHandle nh (modelNh_, robotnamespace_ + "/gazebo_ros_control/pid_gains/" + jointName_[i]);
          // pidController_[i].init (nh);
          break;
        }
        case VELOCITY:
        {
#if GAZEBO_MAJOR_VERSION > 2
          gazeboJoint_[i]->SetParam("fmax", 0, jointEffortLimit_[i]);
#else
          gazeboJoint_[i]->SetMaxForce(0, jointEffortLimit_[i]);
#endif
          break;
        }
        default:
        {
          break;
        }
      }
    }

    // Load gazebo imu link
    gazeboImuLink_ = parentModel_->GetLink(imuLinkName_);

    ROS_INFO("monstertruck_gazebo_interface initialized successfully!");
    return true;
  }

  void MonstertruckGazeboInterface::registerJointInterface()
  {
    XmlRpc::XmlRpcValue jointList;
    modelNh_.getParam("joint_list", jointList);
    ROS_ASSERT(jointList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    // get number of joints
    jointNum_ = jointList.size();

    // Resizing vector according to num of elements
    gazeboJoint_.resize(jointNum_);
    jointName_.resize(jointNum_);
    jointType_.resize(jointNum_);
    jointControlMethod_.resize(jointNum_);
    pidController_.resize(jointNum_);
    jointLowerLimit_.resize(jointNum_);
    jointUpperLimit_.resize(jointNum_);
    jointEffortLimit_.resize(jointNum_);
    jointEffort_.resize(jointNum_);
    jointPosition_.resize(jointNum_);
    jointVelocity_.resize(jointNum_);
    jointCommand_.resize(jointNum_);

    // load joints from param server
    for (int i = 0; i < jointNum_; i++)
    {
      // load joint name
      ROS_ASSERT(jointList[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      jointName_[i] = static_cast<std::string>(jointList[i]["name"]);

      // load joint effort limit
      ROS_ASSERT(jointList[i]["effort_limit"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      jointEffortLimit_[i] = static_cast<double>(jointList[i]["effort_limit"]);

      // load joint control method
      ROS_ASSERT(jointList[i]["control_method"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      jointControlMethod_[i] =
        static_cast<ControlMethod>(
          static_cast<int>((jointList[i]["control_method"])));

      // load joint type
      ROS_ASSERT(jointList[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
      std::string type = static_cast<std::string>(jointList[i]["type"]);
      if (type == "continuous")
        jointType_[i] = urdf::Joint::CONTINUOUS;
      else if (type == "revolute")
        jointType_[i] = urdf::Joint::REVOLUTE;
      else if (type == "prismatic")
        jointType_[i] = urdf::Joint::PRISMATIC;
      else
      {
        ROS_ERROR("Unidentified joint type: %s[%s]", jointName_[i].c_str(),
          type.c_str());
        exit(-1);
      }

      if ( jointType_[i] == urdf::Joint::REVOLUTE
        || jointType_[i] == urdf::Joint::PRISMATIC )
      {
        // load joint upper limit
        ROS_ASSERT(jointList[i]["upper_limit"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        jointUpperLimit_[i] = static_cast<double>(jointList[i]["upper_limit"]);

        // load joint lower limit
        ROS_ASSERT(jointList[i]["lower_limit"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        jointLowerLimit_[i] = static_cast<double>(jointList[i]["lower_limit"]);

        // load joint pid gains
        ROS_ASSERT(
          jointList[i]["pid"]["p"].getType() == XmlRpc::XmlRpcValue::TypeDouble
          && jointList[i]["pid"]["i"].getType() == XmlRpc::XmlRpcValue::TypeDouble
          && jointList[i]["pid"]["d"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        pidController_[i].initPid(
          static_cast<double>(jointList[i]["pid"]["p"]),
          static_cast<double>(jointList[i]["pid"]["i"]),
          static_cast<double>(jointList[i]["pid"]["d"]),
          0.0,
          0.0);
      }
    }

    // Connect and register the joint state interface
    for (unsigned int i = 0; i < jointNum_; i++)
    {
      hardware_interface::JointStateHandle jointStateHandle(
          jointName_[i],
          &jointPosition_[i],
          &jointVelocity_[i],
          &jointEffort_[i]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }
    registerInterface(&jointStateInterface_);

    // Connect and register the joint interfaces
    for (unsigned int i = 0; i < jointNum_; i++)
    {
      hardware_interface::JointHandle jointHandle(
          jointStateInterface_.getHandle(jointName_[i]),
          &jointCommand_[i]);
      velocityJointInterface_.registerHandle(jointHandle);
      effortJointInterface_.registerHandle(jointHandle);
      positionJointInterface_.registerHandle(jointHandle);
    }
    registerInterface(&velocityJointInterface_);
    registerInterface(&effortJointInterface_);
    registerInterface(&positionJointInterface_);
  }

  void MonstertruckGazeboInterface::registerImuInterface()
  {
    // Imu sensor
    imuLinkName_ = "base_link";

    imuOrientation_[0] = 0;
    imuOrientation_[1] = 0;
    imuOrientation_[2] = 0;
    imuOrientation_[3] = 1;

    imuSensorData_.orientation = imuOrientation_;
    imuSensorData_.name = "/sensors/imu";
    imuSensorData_.frame_id = imuLinkName_;

    // Connect and register the imu sensor interface
    hardware_interface::ImuSensorHandle imuSensorHandle(imuSensorData_);
    imuSensorInterface_.registerHandle(imuSensorHandle);
    registerInterface(&imuSensorInterface_);

    // Imu RPY
    imuRoll_ = new double;
    imuPitch_ = new double;
    imuYaw_ = new double;

    *imuRoll_ = 0;
    *imuPitch_ = 0;
    *imuYaw_ = 0;

    imuRPYData_.roll = imuRoll_;
    imuRPYData_.pitch = imuPitch_;
    imuRPYData_.yaw = imuYaw_;
    imuRPYData_.name = "/sensors/imu_rpy";
    imuRPYData_.frame_id = imuLinkName_;

    // Connect and register the imu rpy interface
    pandora_hardware_interface::imu::ImuRPYHandle imuRPYHandle(imuRPYData_);
    imuRPYInterface_.registerHandle(imuRPYHandle);
    registerInterface(&imuRPYInterface_);
  }

  void MonstertruckGazeboInterface::readSim(
      ros::Time time,
      ros::Duration period)
  {
    readTime_ = gazebo::common::Time(time.sec, time.nsec);
    readPeriod_ = period;

    readJoints();
    readImu();
  }

  void MonstertruckGazeboInterface::readJoints()
  {
    if ((jointLastReadTime_ + jointReadRate_) < readTime_)
    {
      for (unsigned int i = 0; i < jointNum_; i++)
      {
        // Read joint position
        switch (jointType_[i])
        {
          case urdf::Joint::PRISMATIC:
          {
            jointPosition_[i] = gazeboJoint_[i]->GetAngle(0).Radian();
            break;
          }
          default:
          {
            jointPosition_[i] +=
                angles::shortest_angular_distance(
                  jointPosition_[i],
                  gazeboJoint_[i]->GetAngle(0).Radian());
            break;
          }
        }
        // Read joint velocity
        jointVelocity_[i] = gazeboJoint_[i]->GetVelocity(0);
      }
    }
  }

  void MonstertruckGazeboInterface::readImu()
  {
    if ((imuLastReadTime_ + imuReadRate_) < readTime_)
    {
      // Read robot orientation for IMU
      gazebo::math::Quaternion quaternion = gazeboImuLink_->GetWorldPose().rot;
      imuOrientation_[0] = quaternion.x;
      imuOrientation_[1] = quaternion.y;
      imuOrientation_[2] = quaternion.z;
      imuOrientation_[3] = quaternion.w;

      // Read robot rpy for IMU
      *imuRoll_ = angles::from_degrees(quaternion.GetRoll());
      *imuPitch_ = angles::from_degrees(quaternion.GetPitch());
      *imuYaw_ = angles::from_degrees(quaternion.GetYaw());
    }
  }

  void MonstertruckGazeboInterface::writeSim(ros::Time time, ros::Duration period)
  {
    writeTime_ = gazebo::common::Time(time.sec, time.nsec);
    writePeriod_ = period;
    writeJoints();
  }

  void MonstertruckGazeboInterface::writeJoints()
  {
    if ((jointLastWriteTime_ + jointWriteRate_) < writeTime_)
    {
      for (unsigned int i = 0; i < jointNum_; i++)
      {
        switch (jointControlMethod_[i])
        {
          case POSITION_PID:
          {
            double error;
            double jointCommand = jointCommand_[i];
            clamp(jointCommand, jointLowerLimit_[i], jointUpperLimit_[i]);

            switch (jointType_[i])
            {
              case urdf::Joint::REVOLUTE:
              {
                angles::shortest_angular_distance_with_limits(
                    jointPosition_[i],
                    jointCommand,
                    jointLowerLimit_[i],
                    jointUpperLimit_[i],
                    error);
                break;
              }
              default:
              {
                error = jointCommand - jointPosition_[i];
                break;
              }
            }

            double pidCommand = pidController_[i].computeCommand(error, writePeriod_);
            double effortLimit = jointEffortLimit_[i];
            double effort = pidCommand;
            clamp(effort, -effortLimit, effortLimit);
            gazeboJoint_[i]->SetForce(0, effort);

            break;
          }
          case VELOCITY:
          {
#if GAZEBO_MAJOR_VERSION > 2
            gazeboJoint_[i]->SetParam("vel", 0, jointCommand_[i]);
#else
            gazeboJoint_[i]->SetVelocity(0, jointCommand_[i]);
#endif
            break;
          }
          case EFFORT:
          {
            gazeboJoint_[i]->SetForce(0, jointCommand_[i]);
            break;
          }
          default:
          {
            ROS_ERROR("Unidentified joint control method");
            break;
          }
        }
      }
    }
  }
}  // namespace pandora_gazebo_interface
