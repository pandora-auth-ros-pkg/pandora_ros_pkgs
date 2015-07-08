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
#include <cmath>

#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>

#include "pandora_data_fusion_utils/utils.h"
#include "pandora_data_fusion_utils/exceptions.h"

#include "frame_matcher/view_pose_finder.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{

  ViewPoseFinder::
  ViewPoseFinder(const std::string& mapType) : pose_finder::PoseFinder(mapType) {}

  ViewPoseFinder::
  ~ViewPoseFinder() {}

  void
  ViewPoseFinder::
  findViewOrientation(
      const geometry_msgs::Point& point,
      const tf::Transform& tfTransform,
      double* yaw,
      double* pitch)
  {
    tf::Vector3 poi_position = pandora_data_fusion_utils::Utils::pointToVector3(point);
    tf::Vector3 sensor_position = tfTransform.getOrigin();
    tf::Vector3 viewFromOrigin = poi_position - sensor_position;
    tf::Vector3 viewFromSensor = tfTransform.getBasis() * viewFromOrigin;
    if (viewFromSensor[0] <= 0)
      throw pandora_data_fusion_utils::AlertException("viewFromSensor should not have non-positive x vector component");
    *yaw = tan(viewFromSensor[1] / viewFromSensor[0]);
    *pitch = tan(- viewFromSensor[2] / viewFromSensor[0]);
  }

}  // namespace frame_matcher
}  // namespace pandora_data_fusion
