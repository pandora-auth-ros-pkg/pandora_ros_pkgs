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

#include <cstdlib>

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

#include "pandora_sensor_coverage/utils.h"

namespace pandora_exploration
{
namespace pandora_sensor_coverage
{
  void
  Utils::
  alignWithNewMap(const nav_msgs::OccupancyGridConstPtr& in,
      const nav_msgs::OccupancyGridPtr& out)
  {
    int oldSize = out->data.size();
    int newSize = in->data.size();
    int8_t* oldMap = new int8_t[oldSize];
    nav_msgs::MapMetaData oldMetaData;
    if (oldSize != 0 && oldSize != newSize)
    {
      // Copy old coverage map meta data.
      oldMetaData = out->info;
      // Copy old coverage map.
      memcpy(oldMap, &out->data[0], out->data.size());
    }
    // Reset coveredSpace_->and copy map2dPtr_'s metadata.
    out->header = in->header;
    out->info = in->info;
    if (oldSize != newSize)
    {
      ROS_WARN("[SENSOR_COVERAGE_SPACE_CHECKER %d] Resizing space coverage...", __LINE__);
      out->data.resize(newSize, 0);
      // ROS_ASSERT(newSize == out->data.size());

      if (oldSize != 0)
      {
        double yawDiff = tf::getYaw(out->info.origin.orientation) -
          tf::getYaw(oldMetaData.origin.orientation);
        double xDiff = out->info.origin.position.x -
          oldMetaData.origin.position.x;
        double yDiff = out->info.origin.position.y -
          oldMetaData.origin.position.y;

        double x = 0, y = 0, xn = 0, yn = 0;
        for (unsigned int ii = 0; ii < oldMetaData.width; ++ii)
        {
          for (unsigned int jj = 0; jj < oldMetaData.height; ++jj)
          {
            x = ii * oldMetaData.resolution;
            y = jj * oldMetaData.resolution;
            xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
            yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
            int coords = static_cast<int>(round(xn / out->info.resolution) +
                  round(yn * out->info.width / out->info.resolution));
            if ((coords > newSize) || (coords < 0))
            {
              ROS_WARN("Error resizing to: %d\nCoords Xn: %f, Yn: %f\n", newSize, xn, yn);
            }
            else
            {
              uint8_t temp = oldMap[ii + jj * oldMetaData.width];
              out->data[coords] = temp;
              Utils::mapDilation(out, 2, coords);
            }
          }
        }
      }
    }
    delete[] oldMap;
  }

  void
  Utils::
  mapDilation(const nav_msgs::OccupancyGridPtr& in, int steps, int coords)
  {
    if (steps == 0)
      return;

    signed char cell = in->data[coords];

    if (cell != 0)  // That's foreground
    {
      // Check for all adjacent
      if (in->data[coords + in->info.width + 1] == 0)
      {
        in->data[coords + in->info.width + 1] = cell;
        Utils::mapDilation(in, steps - 1, coords + in->info.width + 1);
      }
      if (in->data[coords + in->info.width] == 0)
      {
        in->data[coords + in->info.width] = cell;
      }
      if (in->data[coords + in->info.width - 1] == 0)
      {
        in->data[coords + in->info.width - 1] = cell;
        Utils::mapDilation(in, steps - 1, coords + in->info.width - 1);
      }
      if (in->data[coords + 1] == 0)
      {
        in->data[coords + 1] = cell;
      }
      if (in->data[coords - 1] == 0)
      {
        in->data[coords - 1] = cell;
      }
      if (in->data[coords - in->info.width + 1] == 0)
      {
        in->data[coords - in->info.width + 1] = cell;
        Utils::mapDilation(in, steps - 1, coords - in->info.width + 1);
      }
      if (in->data[coords - in->info.width] == 0)
      {
        in->data[coords - in->info.width] = cell;
      }
      if (in->data[coords - in->info.width - 1] == 0)
      {
        in->data[coords - in->info.width - 1] = cell;
        Utils::mapDilation(in, steps - 1, coords - in->info.width - 1);
      }
    }
  }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_exploration
