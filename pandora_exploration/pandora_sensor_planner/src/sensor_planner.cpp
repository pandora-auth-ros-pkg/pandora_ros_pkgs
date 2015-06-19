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
 * Authors: 
 *   Lykartsis Ioannis <lykartsis.giannis@gmail.com>
 *********************************************************************/

#include <string>
#include <vector>

#include "sensor_coverage/sensor_planner.h"

 namespace pandora_exploration
{
  namespace pandora_sensor_coverage
  {

    SensorPlanner::SensorPlanner(const std::string& ns)
    {
      //  initialize NodeHandle and Map.
      nh_.reset( new ros::NodeHandle(ns) );

      //  Subscribe to Octomap topic.
      if (nh_->getParam("subscribed_topic_names/kinect_surface", param))
        octomapSubscriber_ = nh_->subscribe(param, 1, &SensorCoverage::octomapUpdate, this);
      else
      {
        ROS_FATAL("Octomap topic name param not found");
        ROS_BREAK();
      }

      //  Subscribe to Occupancy Grid topic.
      if (nh_->getParam("subscribed_topic_names/kinect_space", param))
        occupancyGridSubscriber_ = nh_->subscribe(param, 1, &SensorCoverage::occupancyGridUpdate, this);
      else
      {
        ROS_FATAL("Occupancy topic name param not found");
        ROS_BREAK();
      }

      currentState_ = 0;

    }

    void SensorPlanner::startTransition(int newState)
    {
      currentState_ = newState;
      transitionComplete(newState);
    }

    void SensorPlanner::completeTrasition()
    {
      switch (currentState_)
      {
        case state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE:
          sensorPlanning_ = EXPLORATION_STATE;
          break;
        case state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION:
          sensorPlanning_ = IDENTIFICATION_STATE;
          break;
        case state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD:
          sensorPlanning_ = HOLD_STATE;
          break;
        case state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_MAPPING:
          sensorPlanning_ = true;
          break;
        default:
          sensorPlanning_ = false;
          break;
      }
    }

    void SensorPlanner::octomapUpdate(const octomap_msgs::Octomap& msg)
    {
      octomap::AbstractOcTree* map = octomap_msgs::fullMsgToMap(msg);
      if (map)
      {
        coveredSurface_.reset(dynamic_cast<octomap::OcTree*>(map));
        if (coveredSurface_.get())
        {
          setMap3d(coveredSurface_);
        }
        else
        {
          ROS_WARN_NAMED("SENSOR_COVERAGE",
              "[SENSOR_COVERAGE_MAP3D_UPDATE %d] AbstractOcTree sent is not OcTree",
              __LINE__);
        }
      }
      else
      {
        ROS_WARN_NAMED("SENSOR_COVERAGE",
            "[SENSOR_COVERAGE_MAP3D_UPDATE %d] Could not deserialize message to OcTree",
            __LINE__);
      }
    }

    void SensorPlanner::occupancyGridUpdate(const nav_msgs::OccupancyGridConstPtr& msg)
    {
      *coveredSpace_ = *msg;
      coveredSpace_->info.origin.orientation.w = 1;
    }



