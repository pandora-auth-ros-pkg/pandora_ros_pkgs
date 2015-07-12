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

#include <std_msgs/Float32.h>

#include "pandora_sensor_coverage/utils.h"

namespace pandora_exploration
{
  namespace pandora_sensor_coverage
  {

    template <class TreeType>
    SpaceChecker<TreeType>::SpaceChecker(
        const NodeHandlePtr& nh, const std::string& frameName)
      : CoverageChecker(nh, frameName)
    {
      resetCoverage();

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

    template <class TreeType>
    nav_msgs::OccupancyGridPtr
    SpaceChecker<TreeType>::
    getSpaceCoverage() const
    {
      return coveredSpace_;
    }

    template <class TreeType>
    void SpaceChecker<TreeType>::findCoverage(
        const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      // Declare helper variables
      CoverageChecker::findCoverage(sensorTransform, baseTransform);
      const float resolution = map2dPtr_->info.resolution;
      float currX = sensorPosition_.x();
      float currY = sensorPosition_.y();
      float fov = (SENSOR_HFOV / 180.0) * PI;
      octomap::point3d cell;

      // Raycast on 2d map rays that orient between -fov_x/2 and fov_x/2 and
      // find how to much covered space (line) corresponds to a 2d cell of
      // the map inside the raycasting area.
      for (float angle = -fov/2; angle < fov/2; angle += DEGREE)
      {
        cell.x() = resolution * cos(sensorYaw_ + angle) + currX;
        cell.y() = resolution * sin(sensorYaw_ + angle) + currY;

        while (CELL(cell.x(), cell.y(), map2dPtr_)
            < static_cast<int8_t>(OCCUPIED_CELL_THRES * 100)
            && pandora_data_fusion::pandora_data_fusion_utils::Utils::
               distanceBetweenPoints2D(octomap::pointOctomapToMsg(sensorPosition_),
                 octomap::pointOctomapToMsg(cell)) < SENSOR_RANGE)
        {
          signed covered;
          if (binary_)
            covered = 100;
          else
            covered = static_cast<signed char>(
                round(cellCoverage(cell, robotPosition_.z()) * 100));
          if (covered > CELL(cell.x(), cell.y(), coveredSpace_))
          {
            CELL(cell.x(), cell.y(), coveredSpace_) = covered;
            CELL(cell.x(), cell.y(), fusedCoveragePtr_) = covered;
          }
          Utils::mapDilation(coveredSpace_, 1, COORDS(cell.x(), cell.y(), coveredSpace_));
          Utils::mapDilation(fusedCoveragePtr_, 1, COORDS(cell.x(), cell.y(), fusedCoveragePtr_));
          cell.x() += resolution * cos(sensorYaw_ + angle);
          cell.y() += resolution * sin(sensorYaw_ + angle);
        }
      }

      // Exclude all cells that are currently truly unknown or occupied. Count all those
      // that are covered indeed.
      unsigned int cellsCovered = 0;
      for (int ii = 0; ii < coveredSpace_->info.width; ++ii)
      {
        for (int jj = 0; jj < coveredSpace_->info.height; ++jj)
        {
          if (map2dPtr_->data[ii + jj * map2dPtr_->info.width] >= static_cast<int8_t>(OCCUPIED_CELL_THRES * 100))
          {
            coveredSpace_->data[ii + jj * coveredSpace_->info.width] = 0;
            fusedCoveragePtr_->data[ii + jj * coveredSpace_->info.width] = 0;
          }
          if (coveredSpace_->data[ii + jj * coveredSpace_->info.width] != 0)
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

      // Robot is standing in fully covered space (assumption).
      double xn = 0, yn = 0;
      for (double x = -FOOTPRINT_WIDTH / 2;
          x <= FOOTPRINT_WIDTH / 2; x += resolution)
      {
        for (double y = -FOOTPRINT_HEIGHT / 2;
            y <= FOOTPRINT_HEIGHT / 2; y += resolution)
        {
          xn = cos(robotYaw_) * x - sin(robotYaw_) * y + robotPosition_.x();
          yn = sin(robotYaw_) * x + cos(robotYaw_) * y + robotPosition_.y();
          CELL(xn, yn, coveredSpace_) = 100;
          CELL(xn, yn, fusedCoveragePtr_) = 100;
          Utils::mapDilation(coveredSpace_, 1, COORDS(xn, yn, coveredSpace_));
          Utils::mapDilation(fusedCoveragePtr_, 1, COORDS(xn, yn, fusedCoveragePtr_));
        }
      }
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
    template <class TreeType>
    float SpaceChecker<TreeType>::cellCoverage(
        const octomap::point3d& cell, float minHeight)
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
            // """ why set coversSpace to true?? """
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

    template <class TreeType>
    void SpaceChecker<TreeType>::publishCoverage(const std::string& frame)
    {
      coveredSpace_->header.stamp = ros::Time::now();
      coveredSpace_->header.frame_id = frame;
      coveragePublisher_.publish(*coveredSpace_);
      std_msgs::Float32 msg;
      msg.data = totalAreaCovered_;
      areaCoveragePublisher_.publish(msg);
    }

    template <class TreeType>
    void SpaceChecker<TreeType>::resetCoverage()
    {
      coveredSpace_.reset( new nav_msgs::OccupancyGrid );
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_exploration

