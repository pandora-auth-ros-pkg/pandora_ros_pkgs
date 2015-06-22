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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>

#include "pandora_vision_msgs/ObstacleAlert.h"

#include "alert_handler/barrel.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{
  Barrel::Barrel()
  {
    obstacleType_ = pandora_vision_msgs::ObstacleAlert::BARREL;
  }
  Barrel::~Barrel() {}

  void Barrel::getVisualization(visualization_msgs::MarkerArray* markers) const
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = Barrel::getGlobalFrame();
    marker.header.stamp = ros::Time::now();
    marker.ns = type_;
    marker.id = id_;
    marker.pose = pose_;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.3;
    marker.color.r = 0;
    marker.color.g = 0.2;
    marker.color.b = 1;
    marker.color.a = 1.0;
    markers->markers.push_back(marker);

    visualization_msgs::Marker description;
    description.header.frame_id = Barrel::getGlobalFrame();
    description.header.stamp = ros::Time::now();
    description.ns = type_ + "_BRIEF";
    description.id = id_;
    description.pose = pose_;
    description.pose.position.z = pose_.position.z + 0.1;
    description.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    description.text = getFrameId();
    description.scale.z = 0.1;
    description.color.r = 0;
    description.color.g = 0;
    description.color.b = 1;
    description.color.a = 0.7;
    markers->markers.push_back(description);
  }

  std::string Barrel::setFrameId(int id)
  {
    return "barrel_" + boost::to_string(id);
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion
