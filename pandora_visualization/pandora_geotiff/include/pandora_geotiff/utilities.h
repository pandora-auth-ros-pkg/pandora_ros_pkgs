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
#include <algorithm>
#include <ros/ros.h>


namespace pandora_geotiff
{
  struct timeObject
  {
    ros::Time time_;
    int idx;
  };

  struct byMinTime
  {
    bool operator()(const timeObject &a, const timeObject &b)
    {
      return a.time_.toSec() < b.time_.toSec();
    }
  };

  /**
   * @brief Returns the permutations after sorting a vector.
   *
   * @param times [ros::Time] Vector of times to sort.
   */

  std::vector<int> getPermutationByTime(std::vector<ros::Time> times)
  {
    std::vector<timeObject> timeObjects;
    std::vector<int> index;

    for (size_t i = 0; i < times.size(); i++)
    {
      timeObject temp = {times[i], i};
      timeObjects.push_back(temp);
    }

    std::sort(timeObjects.begin(), timeObjects.end(), byMinTime());

    for (size_t i = 0; i < timeObjects.size(); i++)
    {
      index.push_back(timeObjects[i].idx);
    }

    return index;
  }
}  // namespace pandora_geotiff

#endif  // PANDORA_GEOTIFF_UTILITIES_H
