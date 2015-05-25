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
* Author:  Chris Zalidis
*********************************************************************/
#ifndef PANDORA_LINEAR_MOVEMENT_CONTROLLER_LINEAR_MOVEMENT_CONTROLLER_H
#define PANDORA_LINEAR_MOVEMENT_CONTROLLER_LINEAR_MOVEMENT_CONTROLLER_H

#include <string>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>
#include <urdf_parser/urdf_parser.h>

#include <pandora_linear_movement_controller/MoveLinearAction.h>

namespace pandora_control
{
  class LinearMovementActionServer
  {
    private:
      ros::NodeHandle nodeHandle_;
      std::string actionName_;
      actionlib::SimpleActionServer<
        pandora_linear_movement_controller::MoveLinearAction> actionServer_;

      ros::Publisher linearCommandPublisher_;

      int command_;
      double minElevation_;
      double minCommand_;
      double maxCommand_;
      double movementThreshold_;
      double laxMovementThreshold_;
      double commandTimeout_;
      double previousTarget_;
      std::string linearCommandTopic_;
      std::string linearMotorFrame_;

      tf::TransformListener tfListener_;

      void callback(const pandora_linear_movement_controller::MoveLinearGoalConstPtr& goal);

      bool getcontrollerParams();
      void testLinear();
      void lowerLinear();
      void moveLinear(std::string pointOfInterest, std::string centerPoint,
        double movementThreshold);

    public:
      LinearMovementActionServer(
        std::string name,
        ros::NodeHandle nodeHandle_);

      ~LinearMovementActionServer(void);
  };
}  // namespace pandora_control
#endif  // PANDORA_LINEAR_MOVEMENT_CONTROLLER_LINEAR_MOVEMENT_CONTROLLER_H
