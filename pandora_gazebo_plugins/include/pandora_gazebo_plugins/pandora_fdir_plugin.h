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
#ifndef PANDORA_GAZEBO_PLUGINS_PANDORA_FDIR_PLUGIN_H
#define PANDORA_GAZEBO_PLUGINS_PANDORA_FDIR_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/bind.hpp>
#include <string>

namespace gazebo
{

class AdaptiveFdir : public ModelPlugin
{
  /* #################    Class Variables    #################*/
   private:
    double yaw;
    // Pointer to the model
    physics::ModelPtr model_;
    sdf::ElementPtr sdf;
    event::ConnectionPtr updateConnection;

    math::Vector3 *left_front_fdir_;
    math::Vector3 *left_rear_fdir_;
    math::Vector3 *right_front_fdir_;
    math::Vector3 *right_rear_fdir_;

    gazebo::math::Pose robot_pose_;

    // Robot Namespace : Used for topics
    std::string robot_namespace_;

  /* #################    Class Methods    #################*/
  public:
    AdaptiveFdir() {}
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    bool LoadParameters();
    void OnUpdate(const common::UpdateInfo & /*_info*/);
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AdaptiveFdir)

}  // namespace gazebo
#endif  // PANDORA_GAZEBO_PLUGINS_PANDORA_FDIR_PLUGIN_H
