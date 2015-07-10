/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 *********************************************************************/

#include "pandora_gazebo_interface/gio_gazebo_interface.h"

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

  GioGazeboInterface::~GioGazeboInterface()
  {
  }

  bool GioGazeboInterface::initSim(
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
    wheelRadius_ = 0.09794;
    wheelSeparation_ = 0.344;

    // Number of elements
    jointNum_ = 13;
    batteryNum_ = 2;
    rangeSensorNum_ = 2;
    co2SensorNum_ = 1;

    // The rate that each element gets updated
    jointReadRate_ = gazebo::common::Time(1 / 100.0);
    jointWriteRate_ = gazebo::common::Time(1 / 100.0);
    imuReadRate_ = gazebo::common::Time(1 / 50.0);
    co2SensorReadRate_ = gazebo::common::Time(1 / 2.0);
    batteryReadRate_ = gazebo::common::Time(1 / 1.0);
    rangeSensorReadRate_ = gazebo::common::Time(1 / 2.0);

    gazebo::common::Time simTime = world_->GetSimTime();
    jointLastReadTime_ = simTime;
    jointLastWriteTime_ = simTime;
    imuLastReadTime_ = simTime;
    co2SensorLastReadTime_ = simTime;
    batteryLastReadTime_ = simTime;
    rangeSensorLastReadTime_ = simTime;

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

    co2SensorData_.resize(co2SensorNum_);
    co2SensorName_.resize(co2SensorNum_);
    co2SensorFrameID_.resize(co2SensorNum_);
    co2SensorCo2PercentageStored_.resize(co2SensorNum_);
    co2SensorCo2Percentage_.resize(co2SensorNum_);

    batteryData_.resize(batteryNum_);
    batteryName_.resize(batteryNum_);
    batteryVoltage_.resize(batteryNum_);
    batteryVoltageMax_.resize(batteryNum_);
    batteryVoltageMin_.resize(batteryNum_);
    batteryDuration_.resize(batteryNum_);

    rangeSensorData_.resize(rangeSensorNum_);
    rangeSensorName_.resize(rangeSensorNum_);
    rangeSensorFrameID_.resize(rangeSensorNum_);
    rangeSensorRadiationType_.resize(rangeSensorNum_);
    rangeSensorFOV_.resize(rangeSensorNum_);
    rangeSensorMinRange_.resize(rangeSensorNum_);
    rangeSensorMaxRange_.resize(rangeSensorNum_);
    rangeSensorRangeStored_.resize(rangeSensorNum_);
    rangeSensorRange_.resize(rangeSensorNum_);
    rangeSensorBufferCounter_.resize(rangeSensorNum_);

    // Register each hardware_interface
    registerJointInterface();
    registerImuInterface();
    registerArmInterface();

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
          gazeboJoint_[i]->SetMaxForce(0, jointEffortLimit_[i]);

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

    // CO2 sensors subscriber
    co2SensorSubscriber_ = modelNh_.subscribe(
        "gazebo_sensors/co2",
        1,
        &GioGazeboInterface::co2SensorCallback,
        this);

    // Range sensors subscriber
    rangeSensorSubscriber_ = modelNh_.subscribe(
        "gazebo_sensors/range",
        1,
        &GioGazeboInterface::rangeSensorCallback,
        this);

    ROS_INFO("gio_gazebo_interface initialized successfully!");
    return true;
  }

  void GioGazeboInterface::registerJointInterface()
  {
    // Wheel joints
    jointName_[0] = "left_front_wheel_joint";
    jointName_[1] = "left_rear_wheel_joint";
    jointName_[2] = "right_front_wheel_joint";
    jointName_[3] = "right_rear_wheel_joint";
    for (unsigned int i = 0; i <= 3; i++)
    {
      jointType_[i] = urdf::Joint::CONTINUOUS;
      jointEffort_[i] = 0.0;
      jointPosition_[i] = 0.0;
      jointVelocity_[i] = 0.0;
      jointCommand_[i] = 0.0;
      jointEffortLimit_[i] = 100.0;
      jointControlMethod_[i] = VELOCITY;
    }

    // Side joints
    jointName_[4] = "left_side_joint";
    jointName_[5] = "right_side_joint";
    for (unsigned int i = 4; i <= 5; i++)
    {
      jointType_[i] = urdf::Joint::REVOLUTE;
      jointEffort_[i] = 0.0;
      jointPosition_[i] = 0.0;
      jointVelocity_[i] = 0.0;
      jointCommand_[i] = 0.0;
      jointLowerLimit_[i] = -0.785;
      jointUpperLimit_[i] = 0.785;
      jointEffortLimit_[i] = 150.0;
      jointControlMethod_[i] = NONE;
    }

    // Laser roll joint
    jointName_[6] = "laser_roll_joint";
    jointType_[6] = urdf::Joint::REVOLUTE;
    jointEffort_[6] = 0.0;
    jointPosition_[6] = 0.0;
    jointVelocity_[6] = 0.0;
    jointCommand_[6] = 0.0;
    jointLowerLimit_[6] = -1.57079632679;
    jointUpperLimit_[6] = 1.57079632679;
    jointEffortLimit_[6] = 50.0;
    jointControlMethod_[6] = POSITION_PID;
    pidController_[6].initPid(1.8, 0.0, 0.3, 0.0, 0.0);

    // Laser pitch joint
    jointName_[7] = "laser_pitch_joint";
    jointType_[7] = urdf::Joint::REVOLUTE;
    jointEffort_[7] = 0.0;
    jointPosition_[7] = 0.0;
    jointVelocity_[7] = 0.0;
    jointCommand_[7] = 0.0;
    jointLowerLimit_[7] = -1.57079632679;
    jointUpperLimit_[7] = 1.57079632679;
    jointEffortLimit_[7] = 50.0;
    jointControlMethod_[7] = POSITION_PID;
    pidController_[7].initPid(2.5, 0.0, 0.3, 0.0, 0.0);

    // Kinect pitch joint
    jointName_[8] = "kinect_pitch_joint";
    jointType_[8] = urdf::Joint::REVOLUTE;
    jointEffort_[8] = 0.0;
    jointPosition_[8] = 0.0;
    jointVelocity_[8] = 0.0;
    jointCommand_[8] = 0.0;
    jointLowerLimit_[8] = -1.57079632679;
    jointUpperLimit_[8] = 1.57079632679;
    jointEffortLimit_[8] = 50.0;
    jointControlMethod_[8] = POSITION_PID;
    pidController_[8].initPid(8.5, 1.0, 0.2, 10.0, -10.0);

    // Kinect yaw joint
    jointName_[9] = "kinect_yaw_joint";
    jointType_[9] = urdf::Joint::REVOLUTE;
    jointEffort_[9] = 0.0;
    jointPosition_[9] = 0.0;
    jointVelocity_[9] = 0.0;
    jointCommand_[9] = 0.0;
    jointLowerLimit_[9] = -1.57079632679;
    jointUpperLimit_[9] = 1.57079632679;
    jointEffortLimit_[9] = 50.0;
    jointControlMethod_[9] = POSITION_PID;
    pidController_[9].initPid(8.0, 1.5, 0.4, 10.0, -10.0);

    // Linear actuaator joint
    jointName_[10] = "linear_actuator_joint";
    jointType_[10] = urdf::Joint::PRISMATIC;
    jointEffort_[10] = 0.0;
    jointPosition_[10] = 0.0;
    jointVelocity_[10] = 0.0;
    jointCommand_[10] = 0.0;
    jointLowerLimit_[10] = 0.0;
    jointUpperLimit_[10] = 0.14;
    jointEffortLimit_[10] = 100.0;
    jointControlMethod_[10] = POSITION_PID;
    pidController_[10].initPid(15000.0, 0.0, 0.0, 0.0, 0.0);

    // Camera effector pan joint
    jointName_[11] = "camera_effector_pan_joint";
    jointType_[11] = urdf::Joint::REVOLUTE;
    jointEffort_[11] = 0.0;
    jointPosition_[11] = 0.0;
    jointVelocity_[11] = 0.0;
    jointCommand_[11] = 0.0;
    jointLowerLimit_[11] = -1.3962;
    jointUpperLimit_[11] = 1.3962;
    jointEffortLimit_[11] = 50.0;
    jointControlMethod_[11] = POSITION_PID;
    pidController_[11].initPid(11.0, 2.0, 0.25, 15.0, -15.0);

    // Camera effector tilt joint
    jointName_[12] = "camera_effector_tilt_joint";
    jointType_[12] = urdf::Joint::REVOLUTE;
    jointEffort_[12] = 0.0;
    jointPosition_[12] = 0.0;
    jointVelocity_[12] = 0.0;
    jointCommand_[12] = 0.0;
    jointLowerLimit_[12] = -0.6;
    jointUpperLimit_[12] = 0.6;
    jointEffortLimit_[12] = 50.0;
    jointControlMethod_[12] = POSITION_PID;
    pidController_[12].initPid(12.0, 1.0, 0.45, 10.0, -10.0);

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

  void GioGazeboInterface::registerImuInterface()
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

  void GioGazeboInterface::registerArmInterface()
  {
    // CO2 sensors
    co2SensorName_[0] = "/sensors/co2";
    co2SensorData_[0].name = co2SensorName_[0];
    co2SensorFrameID_[0] = "co2_frame";
    co2SensorData_[0].frameId = co2SensorFrameID_[0];
    co2SensorCo2PercentageStored_[0] = 0.0;
    co2SensorCo2Percentage_[0] = co2SensorCo2PercentageStored_[0];
    co2SensorData_[0].co2Percentage = &co2SensorCo2Percentage_[0];

    // Connect and register the co2 sensor interface
    for (unsigned int i = 0; i < co2SensorNum_; i++)
    {
      pandora_hardware_interface::arm::Co2SensorHandle co2SensorHandle(co2SensorData_[i]);
      co2SensorInterface_.registerHandle(co2SensorHandle);
    }
    registerInterface(&co2SensorInterface_);

    // Electronics battery sensor
    batteryName_[0] = "/PSU_battery";
    batteryData_[0].name = batteryName_[0];
    batteryVoltageMax_[0] = 25.2;
    batteryVoltageMin_[0] = 22.2;
    batteryDuration_[0] = 30.0;
    batteryVoltage_[0] = batteryVoltageMax_[0];
    batteryData_[0].voltage = &batteryVoltage_[0];

    // Motors battery sensor
    batteryName_[1] = "/motors_battery";
    batteryData_[1].name = batteryName_[1];
    batteryVoltageMax_[1] = 25.2;
    batteryVoltageMin_[1] = 22.2;
    batteryDuration_[1] = 45.0;
    batteryVoltage_[1] = batteryVoltageMax_[1];
    batteryData_[1].voltage = &batteryVoltage_[1];

    // Connect and register the battery interface
    for (unsigned int i = 0; i < batteryNum_; i++)
    {
      pandora_hardware_interface::arm::BatteryHandle batteryHandle(batteryData_[i]);
      batteryInterface_.registerHandle(batteryHandle);
    }
    registerInterface(&batteryInterface_);

    // Range sensors
    rangeSensorName_[0] = "/sensors/left_sonar";
    rangeSensorFrameID_[0] = "left_sonar_frame";
    rangeSensorData_[0].name = rangeSensorName_[0];
    rangeSensorData_[0].frameId = rangeSensorFrameID_[0];

    rangeSensorName_[1] = "/sensors/right_sonar";
    rangeSensorFrameID_[1] = "right_sonar_frame";
    rangeSensorData_[1].name = rangeSensorName_[1];
    rangeSensorData_[1].frameId = rangeSensorFrameID_[1];

    for (unsigned int i = 0; i < rangeSensorNum_; i++)
    {
      rangeSensorRadiationType_[i] = 0;
      rangeSensorFOV_[i] = 60.0;
      rangeSensorMinRange_[i] = 0.2;
      rangeSensorMaxRange_[i] = 4.0;
      rangeSensorRangeStored_[i] = 0.0;
      rangeSensorRange_[i].resize(5, rangeSensorMaxRange_[i]);
      rangeSensorData_[i].radiationType = &rangeSensorRadiationType_[i];
      rangeSensorData_[i].fieldOfView = &rangeSensorFOV_[i];
      rangeSensorData_[i].minRange = &rangeSensorMinRange_[i];
      rangeSensorData_[i].maxRange = &rangeSensorMaxRange_[i];
      rangeSensorData_[i].range = &rangeSensorRange_[i][0];
      rangeSensorBufferCounter_[i] = 0;
    }

    // Connect and register the range sensor interface
    for (unsigned int i = 0; i < rangeSensorNum_; i++)
    {
      pandora_hardware_interface::arm::RangeSensorHandle rangeSensorHandle(rangeSensorData_[i]);
      rangeSensorInterface_.registerHandle(rangeSensorHandle);
    }
    registerInterface(&rangeSensorInterface_);
  }

  void GioGazeboInterface::readSim(
      ros::Time time,
      ros::Duration period)
  {
    readTime_ = gazebo::common::Time(time.sec, time.nsec);
    readPeriod_ = period;

    readJoints();
    readImu();
    readArm();
  }

  void GioGazeboInterface::readJoints()
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
                angles::shortest_angular_distance(jointPosition_[i], gazeboJoint_[i]->GetAngle(0).Radian());

            break;
          }
        }

        // Read joint velocity
        jointVelocity_[i] = gazeboJoint_[i]->GetVelocity(0);
      }
    }
  }

  void GioGazeboInterface::readImu()
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
      *imuRoll_ = quaternion.GetRoll() * 360 / (gazebo::math::Angle::TwoPi.Radian());
      *imuPitch_ = quaternion.GetPitch() * 360 / (gazebo::math::Angle::TwoPi.Radian());
      *imuYaw_ = quaternion.GetYaw() * 360 / (gazebo::math::Angle::TwoPi.Radian());
    }
  }

  void GioGazeboInterface::readArm()
  {
    // Read co2 sensors
    if ((co2SensorLastReadTime_ + co2SensorReadRate_) < readTime_)
    {
      for (unsigned int n = 0; n < co2SensorNum_; n++)
      {
        co2SensorCo2Percentage_[n] = co2SensorCo2PercentageStored_[n];
      }

      co2SensorLastReadTime_ = readTime_;
    }

    // Read battery sensors
    if ((batteryLastReadTime_ + batteryReadRate_) < readTime_)
    {
      for (unsigned int n = 0; n < batteryNum_; n++)
      {
        double reduction = (batteryVoltageMax_[n] - batteryVoltageMin_[n]) / (batteryDuration_[n] * 60.0);
        reduction /= batteryReadRate_.sec + batteryReadRate_.nsec / pow(10, 9);
        batteryVoltage_[n] -= reduction;

        if (batteryVoltage_[n] < batteryVoltageMin_[n])
        {
          ROS_WARN_ONCE("WARNING: The battery \"%s\" has died!", batteryName_[n].c_str());
          // immobilizeRobot (batteryName_[n], batteryVoltage_[n]);
        }
      }

      batteryLastReadTime_ = readTime_;
    }

    // Read range sensors
    if ((rangeSensorLastReadTime_ + rangeSensorReadRate_) < readTime_)
    {
      for (unsigned int n = 0; n < rangeSensorNum_; n++)
      {
        rangeSensorRange_[n][rangeSensorBufferCounter_[n]] = rangeSensorRangeStored_[n];
        rangeSensorBufferCounter_[n] = fmod(rangeSensorBufferCounter_[n] + 1, 5);
      }

      rangeSensorLastReadTime_ = readTime_;
    }
  }

  void GioGazeboInterface::writeSim(
      ros::Time time,
      ros::Duration period)
  {
    writeTime_ = gazebo::common::Time(time.sec, time.nsec);
    writePeriod_ = period;

    writeJoints();
  }

  void GioGazeboInterface::writeJoints()
  {
    if ((jointLastWriteTime_ + jointWriteRate_) < writeTime_)
    {
      adjustWheelVelocityCommands();

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
            gazeboJoint_[i]->SetVelocity(0, jointCommand_[i]);

            break;
          }
          case EFFORT:
          {
            gazeboJoint_[i]->SetForce(0, jointCommand_[i]);

            break;
          }
          default:
          {
            break;
          }
        }
      }
    }
  }

  void GioGazeboInterface::co2SensorCallback(
      const pandora_sensor_msgs::Co2MsgConstPtr& msg)
  {
    for (unsigned int n = 0; n < co2SensorNum_; n++)
    {
      if (msg->header.frame_id == co2SensorFrameID_[n])
      {
        co2SensorCo2PercentageStored_[n] = msg->co2_percentage;
      }
    }
  }

  void GioGazeboInterface::rangeSensorCallback(
      const sensor_msgs::RangeConstPtr& msg)
  {
    for (unsigned int n = 0; n < rangeSensorNum_; n++)
    {
      if (msg->header.frame_id == rangeSensorFrameID_[n])
      {
        rangeSensorRangeStored_[n] = msg->range;
      }
    }
  }

  void GioGazeboInterface::adjustWheelVelocityCommands()
  {
    double leftWheelVelocity = jointCommand_[0];
    double rightWheelVelocity = jointCommand_[2];

    double x = (rightWheelVelocity + leftWheelVelocity) * wheelRadius_ / 2;
    double z = (rightWheelVelocity - leftWheelVelocity) * wheelRadius_ / wheelSeparation_;

    double linearVelocity = x;
    /*
        (+9.446060513) * (0.1000) * pow(x, 1) +
        (+1.315202284) * (1.0000) * pow(x, 3) +
        (-6.404744656) * (1.0000) * pow(x, 5) +
        (+2.449775997) * (1.0000) * pow(x, 7);
    */
    double angularVelocity = z;
    /*
        (+4.219745338) * (0.1000) * pow(z, 1) +
        (+3.166949881) * (0.0100) * pow(z, 3) +
        (-1.104404291) * (0.0100) * pow(z, 5) +
        (+8.216225035) * (0.0001) * pow(z, 7);
    */

    double newLeftWheelVelocity = (linearVelocity - angularVelocity * wheelSeparation_ / 2) / wheelRadius_;
    double newRightWheelVelocity = (linearVelocity + angularVelocity * wheelSeparation_ / 2) / wheelRadius_;
    jointCommand_[0] = newLeftWheelVelocity;
    jointCommand_[1] = newLeftWheelVelocity;
    jointCommand_[2] = newRightWheelVelocity;
    jointCommand_[3] = newRightWheelVelocity;
  }
}  // namespace pandora_gazebo_interface
