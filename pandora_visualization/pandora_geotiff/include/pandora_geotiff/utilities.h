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
 *   Sideris Konstantinos <siderisk@auth.gr>
 *********************************************************************/

#ifndef PANDORA_GEOTIFF_UTILITIES_H
#define PANDORA_GEOTIFF_UTILITIES_H

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


namespace pandora_geotiff
{
  /**
   * @brief Sorts poses based on time.
   *
   * @param poses
   * @param times
   */

  std::vector<geometry_msgs::PoseStamped>
  sortObjects(std::vector<geometry_msgs::PoseStamped> &poses, std::vector<ros::Time> times)
  {
    if (poses.size() != times.size())
    {
      //ROS_ERROR("sortObjects: The size of the vectors is not the same.");
    }

    std::vector<geometry_msgs::PoseStamped> sortedPoses;

    struct DataFusionObject
    {
      geometry_msgs::PoseStamped pose;
      ros::Time time;
    };

    struct byTimeFound
    {
      bool operator()(DataFusionObject const &a, DataFusionObject const &b)
      {
        return a.time.toSec() < b.time.toSec();
      }
    };

    std::vector<DataFusionObject> objects;

    for (int i = 0; i < poses.size(); i++)
    {
      DataFusionObject temp = {poses[i], times[i]};
      objects.push_back(temp);
    }

    std::sort(objects.begin(), objects.end(), byTimeFound());

    for (int i = 0; i < objects.size(); i++)
    {
      sortedPoses.push_back(objects[i].pose);
    }

    return sortedPoses;

  };
}  // namespace pandora_geotiff

#endif
