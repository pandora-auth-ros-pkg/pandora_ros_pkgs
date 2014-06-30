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
#include <utility>
#include <vector>

#include "sensor_coverage/surface_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    SurfaceChecker::SurfaceChecker(const NodeHandlePtr& nh, const std::string& frameName)
      : CoverageChecker(nh, frameName)
    {
      coveredSurface_.reset();

      std::string topic;
      if (nh_->getParam(frameName_+"/published_topic_names/surface_coverage", topic))
      {
        coveragePublisher_ = nh_->advertise<octomap_msgs::Octomap>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s surface coverage published topic name param not found",
            frameName_.c_str());
        ROS_BREAK();
      }

      if (!nh_->getParam(frameName_+"/binary/surface_coverage", binary_))
      {
        ROS_FATAL("%s binary surface coverage param not found", frameName_.c_str());
        ROS_BREAK();
      }

      if (!nh_->getParam(frameName_+"/surface_blur_factor", blurFactor_))
      {
        ROS_FATAL("%s blur surface factor param not found", frameName_.c_str());
        ROS_BREAK();
      }

      getParameters();
    }

    double SurfaceChecker::ORIENTATION_CIRCLE = 0.03;

    void SurfaceChecker::findCoverage(const tf::StampedTransform& transform)
    {
      //  Initialize surface coverage map, if uninitialized.
      if (coveredSurface_.get() == NULL)
      {
        double resolution = map3d_->getResolution() / blurFactor_;
        coveredSurface_.reset( new octomap::ColorOcTree(resolution) );
      }

      // Declare helper variables.
      CoverageChecker::findCoverage(transform);
      double yaw_curr = 0, pitch_curr = 0;
      double h_fov = (SENSOR_HFOV / 180.0) * PI;
      double v_fov = (SENSOR_VFOV / 180.0) * PI;
      octomap::point3d pointOnWall;

      // For every direction in sensor's field of view, ray trace on 3d map.
      // If it hits successfully find its corresponding coverage (metric) and
      // save ray to surface coverage.
      for (double h_angle = -h_fov / 2; h_angle < h_fov / 2;
          h_angle += blurFactor_ * DEGREE)
      {
        for (double v_angle = -v_fov / 2; v_angle < v_fov / 2;
            v_angle += blurFactor_ * DEGREE)
        {
          yaw_curr = yaw_ + h_angle;
          pitch_curr = pitch_ + v_angle;
          octomap::point3d direction(1, 0, 0);
          if (map3d_->castRay(position_,
                direction.rotate_IP(roll_, pitch_curr, yaw_curr),
                pointOnWall,
                false,
                SENSOR_RANGE))
          {
            if (coveredSurface_->insertRay(position_,
                pointOnWall,
                SENSOR_RANGE))
            {
              octomap::ColorOcTreeNode* node = coveredSurface_->search(pointOnWall);
              if (node != NULL)
              {
                unsigned char coverage = findPointCoverage(pointOnWall, direction);
                if (node->isColorSet() && node->getColor().r >= coverage)
                  continue;
                node->setColor(coverage, 0, 0);
              }
            }
          }
        }
      }
      // coveredSurface_->updateInnerOccupancy();
    }

    unsigned char SurfaceChecker::findPointCoverage(const octomap::point3d& pointOnWall,
        const octomap::point3d& direction)
    {
      // if surface coverage is to be taken as a binary quantity.
      if (binary_) return 255;

      // else it is assumed that coverage is a percentage of the best view
      // one can get at the wall.
      octomap::point3d normalOnWall;
      if (!findNormalVectorOnWall(pointOnWall, &normalOnWall))
        return 0;
      return metric(normalOnWall, direction);
    }

    unsigned char SurfaceChecker::metric(const octomap::point3d& normalOnWall,
        const octomap::point3d& direction)
    {
      octomap::point3d unitOnWall = normalOnWall.normalized();
      return static_cast<unsigned char>(
          floor(
            fabs(unitOnWall.dot(direction.normalized())) * 255));
    }

    bool SurfaceChecker::findNormalVectorOnWall(
        const octomap::point3d& point, octomap::point3d* normal)
    {
      std::vector<octomap::point3d> points;
      std::pair<octomap::point3d, octomap::point3d> pointsOnWall;

      // find for constant z = point.z, the normal vector on the line which
      // results from intersection of the surface-wall (approx. a plane) with the
      // plane z = point.z
      double x = 0, y = 0, dx = 0, dy = 0;
      for (unsigned int i = 0; i < 360; i += 5)
      {
        x = point.x() + ORIENTATION_CIRCLE * cos((i / 180.0) * PI);
        y = point.y() + ORIENTATION_CIRCLE * sin((i / 180.0) * PI);

        if (map3d_->search(x, y, point.z())->getOccupancy()
            > map3d_->getOccupancyThres())
        {
          octomap::point3d temp;
          temp.x() = x;
          temp.y() = y;
          temp.z() = point.z();
          points.push_back(temp);
        }
      }
      try
      {
        pointsOnWall = findDiameterEndPointsOnWall(points);
      }
      catch (std::runtime_error& ex)
      {
        return false;
      }
      float angle = atan2((pointsOnWall.second.y() - pointsOnWall.first.y()),
          (pointsOnWall.second.x() - pointsOnWall.first.x()));
      dx = ORIENTATION_CIRCLE * cos((PI / 2) + angle);
      dy = ORIENTATION_CIRCLE * sin((PI / 2) + angle);

      // find the intersection of the plane [1] (λx, λy, z) ~λ,z with the plane which
      // results from the far-most points on wall and on the plane [1]. This is going
      // to be wall's normal vector.
      points.clear();
      double z = 0, dz = 0;
      for (double lambda = -1.0; lambda <= 1; lambda += 0.005)
      {
        dz = sqrt((1 - pow(lambda, 2)) * pow(ORIENTATION_CIRCLE, 2));
        x = point.x() + lambda * dx;
        y = point.y() + lambda * dy;
        if (map3d_->search(x, y, point.z() + dz)->getOccupancy()
            > map3d_->getOccupancyThres())
        {
          octomap::point3d temp;
          temp.x() = x;
          temp.y() = y;
          temp.z() = point.z() + dz;
          points.push_back(temp);
        }
        if (map3d_->search(x, y, point.z() - dz)->getOccupancy()
            > map3d_->getOccupancyThres())
        {
          octomap::point3d temp;
          temp.x() = x;
          temp.y() = y;
          temp.z() = point.z() - dz;
          points.push_back(temp);
        }
      }
      try
      {
        pointsOnWall = findDiameterEndPointsOnWall(points);
      }
      catch (std::runtime_error& ex)
      {
        return false;
      }
      x = dx;
      y = dy;
      z = x * (pointsOnWall.second.x() - pointsOnWall.first.x()) +
        y * (pointsOnWall.second.y() - pointsOnWall.first.y());
      z /= pointsOnWall.second.z() - pointsOnWall.first.z();

      normal->x() = x;
      normal->y() = y;
      normal->z() = z;
      normal->normalize();
      return true;
    }

    std::pair<octomap::point3d, octomap::point3d> SurfaceChecker::
      findDiameterEndPointsOnWall(const std::vector<octomap::point3d>& points)
      {
        if (points.size() < 2)
        {
          throw std::runtime_error("Cannot calculate pair with less than 2 points.");
        }
        float maxDist = 0, dist = 0;
        std::pair<octomap::point3d, octomap::point3d> pointsOnWall;
        for (unsigned int i = 0; i < points.size(); i++)
        {
          for (unsigned int j = i + 1; j < points.size(); j++)
          {
            dist = points[i].distance(points[j]);
            if (dist > maxDist)
            {
              maxDist = dist;
              pointsOnWall = std::make_pair(points[i], points[j]);
            }
          }
        }
        return pointsOnWall;
      }

    void SurfaceChecker::publishCoverage(const std::string& frame)
    {
      coveredSurface_->toMaxLikelihood();
      coveredSurface_->prune();
      octomap_msgs::Octomap msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame;
      msg.binary = false;
      msg.id = coveredSurface_->getTreeType();
      msg.resolution = coveredSurface_->getResolution();
      if (octomap_msgs::fullMapToMsg(*coveredSurface_, msg))
        coveragePublisher_.publish(msg);
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

