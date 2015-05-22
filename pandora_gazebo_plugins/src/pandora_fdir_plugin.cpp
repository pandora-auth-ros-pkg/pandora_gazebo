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
* Author:  Zisis Konstantinos <zisikons@gmail.com>
*********************************************************************/
#include "pandora_gazebo_plugins/pandora_fdir_plugin.h"


namespace gazebo
{
  /*
   * Load() function that all gazebo plugins must have.It has similar functionality 
   * with a constructor
   */
  void AdaptiveFdir::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Load Parameters
    this->model_ = _model;
    this->sdf = _sdf;

    // Connect with world update Event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AdaptiveFdir::OnUpdate, this, _1));

    // Load SDF Parameters
    if (!LoadParameters())
      ROS_FATAL("Loading Parameters failed :( ");

    yaw = 0;
  }

  /*
   * Function to load parameters from URDF.Called in Load().
   */
  bool AdaptiveFdir::LoadParameters()
  {
    uint32_t zero = 0;

    /* ############### LOAD NAMESPACE (from sdf) ############### */
    this ->robot_namespace_ = "";

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
     return false;
    }

    // Load Namespace from sdf
    if ( this ->sdf ->HasElement ( "robotNamespace" ) )
      this ->robot_namespace_ = this ->sdf ->Get < std ::string > ("robotNamespace") + "/";


    /* ############### LEFT REAR WHEEL ############### */
    if ( !this ->sdf ->HasElement("leftRearWheelLink" ) )
    {
      ROS_FATAL("AdaptiveFdir Plugin missing <leftRearWheelLink>.");
      return false;
    }

    else
    {
        this->left_rear_fdir_ = &(this ->model_ ->
                        GetLink(this ->sdf ->Get < std ::string >("leftRearWheelLink"))
                            -> GetCollision(zero)
                                         ->GetSurface()
                                                     ->fdir1);
    }

    /* ############### LEFT FRONT WHEEL ############### */
    if ( !this ->sdf ->HasElement("leftFrontWheelLink" ) )
    {
      ROS_FATAL("AdaptiveFdir Plugin missing <leftFrontWheelLink>.");
      return false;
    }

    else
    {
        this->left_front_fdir_ = &(this ->model_ ->
                        GetLink(this ->sdf ->Get < std ::string >("leftFrontWheelLink"))
                            -> GetCollision(zero)
                                         ->GetSurface()
                                                     ->fdir1);
    }

    /* ############### RIGHT REAR WHEEL ############### */
    if ( !this ->sdf ->HasElement("rightRearWheelLink" ) )
    {
      ROS_FATAL("AdaptiveFdir Plugin missing <rightRearWheelLink>.");
      return false;
    }

    else
    {
        this->right_rear_fdir_ = &(this ->model_ ->
                        GetLink(this ->sdf ->Get < std ::string >("rightRearWheelLink"))
                            -> GetCollision(zero)
                                         ->GetSurface()
                                                     ->fdir1);
    }

    /* ############### RIGHT FRONT WHEEL ###############*/
    if ( !this ->sdf ->HasElement("rightFrontWheelLink" ) )
    {
      ROS_FATAL("AdaptiveFdir Plugin missing <rightFrontWheelLink>.");
      return false;
    }

    else
    {
        this->right_front_fdir_ = &(this ->model_ ->
                        GetLink(this ->sdf ->Get < std ::string >("rightFrontWheelLink"))
                            -> GetCollision(zero)
                                         ->GetSurface()
                                                     ->fdir1);
    }

    return true;
  }

  /*
   * Update Function , called every simulator iteration.In this case , the function
   * reads the current yaw and update friction parameter fdir1 in every wheel , so 
   * friction is irrelevant to mu. (not yet sure if this works)
   */
  void AdaptiveFdir::OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    // Read Yaw
    robot_pose_ = model_->GetWorldPose();
    yaw = robot_pose_.rot.GetYaw();

    // Change fdir vector to ALL wheels:
    left_rear_fdir_->x   = std::sin(yaw);
    left_front_fdir_->x  = std::sin(yaw);
    right_rear_fdir_->x  = std::sin(yaw);
    right_front_fdir_->x = std::sin(yaw);


    left_rear_fdir_->y   = std::cos(yaw);
    left_front_fdir_->y  = std::cos(yaw);
    right_rear_fdir_->y  = std::cos(yaw);
    right_front_fdir_->y = std::cos(yaw);
  }

}  // namespace gazebo
