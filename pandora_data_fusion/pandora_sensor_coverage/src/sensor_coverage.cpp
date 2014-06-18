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

#include "sensor_coverage/sensor_coverage.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    SensorCoverage::SensorCoverage(const std::string& ns)
    {
      //  initialize NodeHandle and Map.
      nh_.reset( new ros::NodeHandle(ns) );
      globalMap_.reset( new octomap_msgs::Octomap );

      //  Subscribe to octomap topic.
      std::string param;
      if (nh_->getParam("subscribed_topic_names/map", param))
      {
        mapSubscriber_ = nh_->subscribe(param, 1, &SensorCoverage::mapUpdate, this);
      }
      else
      {
        ROS_FATAL("map topic name param not found");
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
      for (int32_t ii = 0; ii < framesToTrack.size(); ++ii)
      {
        ROS_ASSERT(framesToTrack[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
        registeredSensors_.push_back(
            SensorPtr( new Sensor(
                nh_,
                globalMap_,
                static_cast<std::string>(framesToTrack[ii]),
                param)));
      }

      clientInitialize();
    }

    void SensorCoverage::startTransition(int newState)
    {
      currentState_ = newState;
    }

    void SensorCoverage::completeTransition()
    {
      for (int ii = 0; ii < registeredSensors_.size(); ++ii)
      {
        registeredSensors_[ii]->notifyStateChange(currentState_);
      }
    }

    void SensorCoverage::mapUpdate(const octomap_msgs::Octomap& map)
    {
      *globalMap_ = map;
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion
