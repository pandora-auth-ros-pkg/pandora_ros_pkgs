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

#include "std_msgs/Float32.h"

#include "sensor_coverage/space_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    SpaceChecker::SpaceChecker(const NodeHandlePtr& nh, const std::string& frameName)
      : CoverageChecker(nh, frameName)
    {
      coverageMap3d_ = map3d_;

      std::string topic;

      if (nh_->getParam(frameName_+"/published_topic_names/space_coverage", topic))
      {
        coveragePublisher_ = nh_->advertise<nav_msgs::OccupancyGrid>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s space coverage published topic name param not found",
            frameName_.c_str());
        ROS_BREAK();
      }

      if (nh_->getParam(frameName_+"/published_topic_names/area_coverage", topic))
      {
        areaCoveragePublisher_ = nh_->advertise<std_msgs::Float32>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s area coverage published topic name param not found",
            frameName_.c_str());
        ROS_BREAK();
      }

      if (!nh_->getParam(frameName_+"/binary/space_coverage", binary_))
      {
        ROS_FATAL("%s binary space coverage param not found", frameName_.c_str());
        ROS_BREAK();
      }

      getParameters();
    }

    double SpaceChecker::MAX_HEIGHT = 0;
    double SpaceChecker::FOOTPRINT_WIDTH = 0;
    double SpaceChecker::FOOTPRINT_HEIGHT = 0;

    void SpaceChecker::findCoverage(const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      // Aligning coverage OGD with current map. Resizing, rotating and translating.
      alignCoverageWithMap();

      // Declare helper variables
      CoverageChecker::findCoverage(sensorTransform);
      const float resolution = map2d_->info.resolution;
      float robotX = baseTransform.getOrigin()[0];
      float robotY = baseTransform.getOrigin()[1];
      float minZ = baseTransform.getOrigin()[2];
      float currX = position_.x();
      float currY = position_.y();
      float fov = (SENSOR_HFOV / 180.0) * PI;
      octomap::point3d cell;

      // Robot is standing in fully covered space (assumption).
      for (int ii = -static_cast<int>((ceil(FOOTPRINT_WIDTH/resolution)));
          ii < static_cast<int>((ceil(FOOTPRINT_WIDTH/resolution))) + 1; ii++)
      {
        for (int jj = -static_cast<int>((ceil(FOOTPRINT_HEIGHT/resolution)));
            jj < static_cast<int>((ceil(FOOTPRINT_HEIGHT/resolution))) + 1; jj++)
        {
          coveredSpace_.data[ii + jj * coveredSpace_.info.width] = 100;
        }
      }

      // Raycast on 2d map rays that orient between -fov_x/2 and fov_x/2 and
      // find how to much covered space (line) corresponds to a 2d cell of
      // the map inside the raycasting area.
      for (float angle = -fov/2; angle < fov/2; angle += DEGREE)
      {
        cell.x() = resolution * cos(yaw_ + angle) + currX;
        cell.y() = resolution * sin(yaw_ + angle) + currY;

        while (CELL(cell.x(), cell.y(), map2d_)
            < static_cast<int8_t>(OCCUPIED_CELL_THRES * 100)
            && Utils::distanceBetweenPoints2D(octomap::pointOctomapToMsg(position_),
              octomap::pointOctomapToMsg(cell)) < SENSOR_RANGE)
        {
          signed covered;
          if (binary_)
            covered = 100;
          else
            covered = static_cast<signed char>(floor(cellCoverage(cell, minZ) * 100));
          if (covered > CELL(cell.x(), cell.y(), (&coveredSpace_)))
          {
            CELL(cell.x(), cell.y(), (&coveredSpace_)) = covered;
          }
          coverageDilation(1, COORDS(cell.x(), cell.y(), (&coveredSpace_)));
          cell.x() += resolution * cos(yaw_ + angle);
          cell.y() += resolution * sin(yaw_ + angle);
        }
      }

      // Exclude all cells that are currently truly unknown or occupied. Count all those
      // that are covered indeed.
      unsigned int cellsCovered = 0;
      for (int ii = 0; ii < coveredSpace_.info.width; ++ii)
      {
        for (int jj = 0; jj < coveredSpace_.info.height; ++jj)
        {
          if (map2d_->data[ii + jj * map2d_->info.width] >= static_cast<int8_t>(OCCUPIED_CELL_THRES * 100))
          {
            coveredSpace_.data[ii + jj * coveredSpace_.info.width] = 0;
          }
          if (coveredSpace_.data[ii + jj * coveredSpace_.info.width] != 0)
          {
            cellsCovered++;
          }
        }
      }
      // Calculate total area explored according to this sensor.
      totalAreaCovered_ = cellsCovered * resolution * resolution;
      ROS_INFO_THROTTLE(20,
          "[SENSOR_COVERAGE_SPACE_CHECKER %d] Total area covered is %f m^2.",
          __LINE__, totalAreaCovered_);
    }

    /**
     * @details Calculates space covered above a cell in 2D map as a percentage of
     * covered space over unoccupied space according to current 3D map. Vertical
     * raytracing goes top to bottom and stops either when the node examined is
     * in the same height as base footprint or when an occupied node is found and
     * the space below it is uncovered. This method is assuming that there is
     * no way that the robot's base footprint can have different z for the same (x, y),
     * but it assures that coverage as a percentage is true at all times and it is not
     * dependent of current z.
     */
    float SpaceChecker::cellCoverage(const octomap::point3d& cell, float minHeight)
    {
      octomap::point3d end(cell.x(), cell.y(), minHeight);
      //  begin can be later implemented having z = minHeight + MAX_HEIGHT.
      octomap::point3d begin(cell.x(), cell.y(), MAX_HEIGHT);
      octomap::KeyRay keyRay;
      if (!coverageMap3d_->computeRayKeys(begin, end, keyRay))
      {
        ROS_ERROR("[SENSOR_COVERAGE_SPACE_CHECKER %d] Compute ray went out of range!",
            __LINE__);
      }
      bool coversSpace = false, occupied = false;
      float coveredSpace = 0, startZ = begin.z(), unoccupiedSpace = 0;
      for (octomap::KeyRay::iterator it = keyRay.begin();
          it != keyRay.end(); ++it)
      {
        octomap::OcTreeNode* node = coverageMap3d_->search(*it);
        if (!node)
        {
          continue;
        }
        if (occupied)
        {
          if (node->getOccupancy() == 0)
          {
            //  In 3d navigation planning this must be changed.
            occupied = false;
            coversSpace = false;
            break;
          }
          else if (node->getOccupancy() <= coverageMap3d_->getOccupancyThres())
          {
            occupied = false;
            coversSpace = true;
            startZ = coverageMap3d_->keyToCoord(*it).z();
          }
        }
        else if (coversSpace)
        {
          if (node->getOccupancy() == 0 || it == --keyRay.end())
          {
            coversSpace = false;
            octomap::point3d coord = coverageMap3d_->keyToCoord(*it);
            coveredSpace += startZ - coord.z();
            unoccupiedSpace += startZ - coord.z();
            startZ = coord.z();
          }
          else if (node->getOccupancy() > coverageMap3d_->getOccupancyThres())
          {
            occupied = true;
            coversSpace = false;
            octomap::point3d coord = coverageMap3d_->keyToCoord(*it);
            coveredSpace += startZ - coord.z();
            unoccupiedSpace += startZ - coord.z();
          }
        }
        else
        {
          if (node->getOccupancy() > coverageMap3d_->getOccupancyThres())
          {
            occupied = true;
            unoccupiedSpace += startZ - coverageMap3d_->keyToCoord(*it).z();
          }
          else if (node->getOccupancy() > 0)
          {
            coversSpace = true;
            octomap::point3d coord = coverageMap3d_->keyToCoord(*it);
            unoccupiedSpace += startZ - coord.z();
            startZ = coord.z();
          }
        }
      }
      return coveredSpace / unoccupiedSpace;
    }

    void SpaceChecker::alignCoverageWithMap()
    {
      int oldSize = coveredSpace_.data.size();
      int newSize = map2d_->data.size();
      int8_t* oldCoverage = new int8_t[oldSize];
      nav_msgs::MapMetaData oldMetaData;
      if (oldSize != 0 && oldSize != newSize)
      {
        // Copy old coverage map meta data.
        oldMetaData = coveredSpace_.info;
        // Copy old coverage map.
        for (unsigned int ii = 0; ii < oldSize; ++ii)
        {
          oldCoverage[ii] = coveredSpace_.data[ii];
        }
      }
      // Reset coveredSpace_ and copy map2D_'s metadata.
      coveredSpace_.header = map2d_->header;
      coveredSpace_.info = map2d_->info;
      if (oldSize != newSize)
      {
        ROS_WARN("[SENSOR_COVERAGE_SPACE_CHECKER %d] Resizing space coverage...", __LINE__);
        coveredSpace_.data.resize(newSize, 0);
        ROS_ASSERT(newSize == coveredSpace_.data.size());

        if (oldSize != 0)
        {
          double yawDiff = tf::getYaw(coveredSpace_.info.origin.orientation) -
            tf::getYaw(oldMetaData.origin.orientation);
          double xDiff = coveredSpace_.info.origin.position.x -
            oldMetaData.origin.position.x;
          double yDiff = coveredSpace_.info.origin.position.y -
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
              int coords = static_cast<int>((floor((xn + yn * coveredSpace_.info.width)
                    / coveredSpace_.info.resolution)));
              coveredSpace_.data[coords] = oldCoverage[ii + jj * oldMetaData.width];
              coverageDilation(1, COORDS(xn, yn, (&coveredSpace_)));
            }
          }
        }
      }
      delete[] oldCoverage;
    }

    void SpaceChecker::coverageDilation(int steps, int coords)
    {
      if (steps == 0)
        return;

      signed char cell = coveredSpace_.data[coords];

      if (cell != 0)  // That's foreground
      {
        // Check for all adjacent
        if (coveredSpace_.data[coords + coveredSpace_.info.width + 1] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width + 1] = cell;
          coverageDilation(steps - 1, coords + coveredSpace_.info.width + 1);
        }
        if (coveredSpace_.data[coords + coveredSpace_.info.width] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width] = cell;
        }
        if (coveredSpace_.data[coords + coveredSpace_.info.width - 1] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width - 1] = cell;
          coverageDilation(steps - 1, coords + coveredSpace_.info.width - 1);
        }
        if (coveredSpace_.data[coords + 1] == 0)
        {
          coveredSpace_.data[coords + 1] = cell;
        }
        if (coveredSpace_.data[coords - 1] == 0)
        {
          coveredSpace_.data[coords - 1] = cell;
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width + 1] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width + 1] = cell;
          coverageDilation(steps - 1, coords - coveredSpace_.info.width + 1);
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width] = cell;
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width - 1] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width - 1] = cell;
          coverageDilation(steps - 1, coords - coveredSpace_.info.width - 1);
        }
      }
    }

    void SpaceChecker::publishCoverage()
    {
      coveragePublisher_.publish(coveredSpace_);
      std_msgs::Float32 msg;
      msg.data = totalAreaCovered_;
      areaCoveragePublisher_.publish(msg);
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

