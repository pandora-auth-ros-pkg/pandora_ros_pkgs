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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>
#include <vector>

#include "pandora_sensor_coverage/sensor_coverage.h"

namespace pandora_exploration
{
  namespace pandora_sensor_coverage
  {

    SensorCoverage::SensorCoverage(const std::string& ns)
    {
      int ii;
      //  initialize NodeHandle and Map.
      nh_.reset( new ros::NodeHandle(ns) );

      globalMap3dPtrPtr_.reset( new octomap::OcTree* );
      *globalMap3dPtrPtr_ = NULL;
      Sensor::setMap3d(globalMap3dPtrPtr_);

      std::string param;
      double paramD = 0;

      //  Subscribe to 3d slam topic.
      if (nh_->getParam("subscribed_topic_names/map3d", param))
        map3dSubscriber_ = nh_->subscribe(param, 1, &SensorCoverage::map3dUpdate, this);
      else
      {
        ROS_FATAL("map3d topic name param not found");
        ROS_BREAK();
      }

      //  Subscribe to 2d slam topic.
      if (nh_->getParam("subscribed_topic_names/map2d", param))
        map2dSubscriber_ = nh_->subscribe(param, 1, &SensorCoverage::map2dUpdate, this);
      else
      {
        ROS_FATAL("map2d topic name param not found");
        ROS_BREAK();
      }

      // Server of flushing service
      if (nh_->getParam("service_server_names/flush_coverage", param))
      {
        flushService_ = nh_->advertiseService(param,
            &SensorCoverage::flushCoverage, this);
      }
      else
      {
        ROS_FATAL("flush_coverage service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("published_topic_names/fused_coverage", param))
      {
        fusedCoveragePublisher_ = nh_->advertise<nav_msgs::OccupancyGrid>(param, 1);
      }
      else
      {
        ROS_FATAL("fused_coverage publish topic name param not found");
        ROS_BREAK();
      }

      fusingCoverage_ = false;

      resetFusedCoverage();

      //  Set up occupancy grid 2d map occupancy threshold.
      nh_->param<double>("occupied_cell_thres", paramD, static_cast<double>(0.5));
      CoverageChecker::setOccupiedCellThres(paramD);

      //  Set up maximum height of interest.
      if (nh_->getParam("max_height", paramD))
        CoverageChecker::setMaxHeight(paramD);
      else
      {
        ROS_FATAL("max height param not found");
        ROS_BREAK();
      }

      //  Set up footprint's width.
      if (nh_->getParam("footprint/width", paramD))
        CoverageChecker::setFootprintWidth(paramD);
      else
      {
        ROS_FATAL("footprint's width param not found");
        ROS_BREAK();
      }

      //  Set up footprint's height.
      if (nh_->getParam("footprint/height", paramD))
        CoverageChecker::setFootprintHeight(paramD);
      else
      {
        ROS_FATAL("footprint's height param not found");
        ROS_BREAK();
      }

      //  Set up orientation circle.
      if (nh_->getParam("orientation_circle", paramD))
        SurfaceChecker::setOrientationCircle(paramD);
      else
      {
        ROS_FATAL("orientation circle param not found");
        ROS_BREAK();
      }

      //  Set up maps' global static frame.
      if (nh_->getParam("global_frame", param))
        Sensor::setGlobalFrame(param);
      else
      {
        ROS_FATAL("global frame name param not found");
        ROS_BREAK();
      }

      //  Set up robot's base frame name.
      if (nh_->getParam("robot_base_frame", param))
        Sensor::setRobotBaseFrame(param);
      else
      {
        ROS_FATAL("robot base frame name param not found");
        ROS_BREAK();
      }

      currentState_ = 0;

      //  Get map's origin (can be either SLAM or TEST).
      if (!nh_->getParam("map_origin", param))
      {
        ROS_FATAL("map origin param not found");
        ROS_BREAK();
      }
      //  Get frames that will be tracked to produce their coverage patch maps.
      XmlRpc::XmlRpcValue framesToTrack;
      if (!nh_->getParam("frames_to_track", framesToTrack))
      {
        ROS_FATAL("frames to track param not found");
        ROS_BREAK();
      }
      ROS_ASSERT(framesToTrack.getType() == XmlRpc::XmlRpcValue::TypeArray);
      //  For each frame make a Sensor object.
      for (ii = 0; ii < framesToTrack.size(); ++ii)
      {
        ROS_ASSERT(framesToTrack[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
        registeredSensors_.push_back(
            SensorPtr( new Sensor(
                nh_,
                static_cast<std::string>(framesToTrack[ii]),
                param)));
      }

      for (ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->shareFusedCoverage(fusedCoveragePtr_);
      }

      coverageFuser_ = nh_->createTimer(ros::Duration(0.1),
          &SensorCoverage::fuseCoverage, this);

      clientInitialize();
    }

    void
    SensorCoverage::
    fuseCoverage(const ros::TimerEvent& event)
    {
      if (!fusingCoverage_)
        return;

      fusedCoveragePtr_->header.stamp = ros::Time::now();
      fusedCoveragePublisher_.publish(fusedCoveragePtr_);
    }

    void SensorCoverage::startTransition(int newState)
    {
      currentState_ = newState;
      transitionComplete(newState);
    }

    void SensorCoverage::completeTransition()
    {
      for (int ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->notifyStateChange(currentState_);
      }
      if (currentState_ == state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE ||
          currentState_ == state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION ||
          currentState_ == state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD ||
          currentState_ == state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_MAPPING)
        fusingCoverage_ = true;
      else
        fusingCoverage_ = false;
    }

    void SensorCoverage::map3dUpdate(const octomap_msgs::Octomap& msg)
    {
      octomap::AbstractOcTree* map = octomap_msgs::fullMsgToMap(msg);
      if (map)
      {
        if (*globalMap3dPtrPtr_)
          delete *globalMap3dPtrPtr_;
        *globalMap3dPtrPtr_ = dynamic_cast<octomap::OcTree*>(map);

        if (*globalMap3dPtrPtr_ == NULL)
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

    void SensorCoverage::map2dUpdate(const nav_msgs::OccupancyGridConstPtr& msg)
    {
      globalMap2dPtr_ = msg;

      Utils::alignWithNewMap(msg, fusedCoveragePtr_);
      for (int ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->setMap2d(msg);
      }
    }

    bool SensorCoverage::flushCoverage(
        std_srvs::Empty::Request& rq,
        std_srvs::Empty::Response& rs)
    {
      resetFusedCoverage();

      for (int ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->flushCoverage();
      }

      return true;
    }

    void
    SensorCoverage::
    resetFusedCoverage()
    {
      fusedCoveragePtr_.reset( new nav_msgs::OccupancyGrid );
      for (int ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->shareFusedCoverage(fusedCoveragePtr_);
      }
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_exploration
