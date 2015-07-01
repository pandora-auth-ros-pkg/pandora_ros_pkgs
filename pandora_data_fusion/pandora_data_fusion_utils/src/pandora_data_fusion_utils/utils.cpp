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
 *   Christos Zalidis <zalidis@gmail.com>
 *   Triantafyllos Afouras <afourast@gmail.com>
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Vector3.h>

#include "pandora_data_fusion_utils/utils.h"

namespace pandora_data_fusion
{
namespace pandora_data_fusion_utils
{

  geometry_msgs::Point Utils::point2DAndHeight2Point3D(geometry_msgs::Point position, float height)
  {
    position.z = height;
    return position;
  }

  float Utils::distanceBetweenPoints2D(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    float xDist = a.x - b.x;
    float yDist = a.y - b.y;

    return sqrt((xDist * xDist) + (yDist * yDist));
  }

  float Utils::distanceBetweenPoints3D(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    float xDist = a.x - b.x;
    float yDist = a.y - b.y;
    float zDist = a.z - b.z;

    return sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
  }

  float Utils::distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b, bool is3D)
  {
    float distance = 0;
    if (is3D)
    {
      distance = Utils::distanceBetweenPoints3D(a, b);
    }
    else
    {
      distance = Utils::distanceBetweenPoints2D(a, b);
    }
    return distance;
  }

  geometry_msgs::Quaternion Utils::calculateQuaternion(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    tfScalar yaw;

    yaw = atan2(b.y - a.y, b.x - a.x);

    return tf::createQuaternionMsgFromYaw(yaw);
  }

  geometry_msgs::Point Utils::vector3ToPoint(const tf::Vector3& vector)
  {
    geometry_msgs::Point point;
    point.x = vector[0];
    point.y = vector[1];
    point.z = vector[2];

    return point;
  }

  bool Utils::arePointsInRange(geometry_msgs::Point pointA, geometry_msgs::Point pointB,
      bool is3D, float sensor_range)
  {
    float dist = distanceBetweenPoints(pointA, pointB, is3D);
    return dist <= sensor_range;
  }

  /**
   * @details: reference and pose should have the same origin, e.g. /map
   */
  bool Utils::isPoseInBox2D(const geometry_msgs::Pose& reference,
      double length, double width, const geometry_msgs::Pose& pose)
  {
    double yaw = tf::getYaw(reference.orientation);
    double xDiff = pose.position.x - reference.position.x;
    double yDiff = pose.position.y - reference.position.y;
    double xn = cos(yaw) * xDiff - sin(yaw) * yDiff;
    double yn = sin(yaw) * xDiff + cos(yaw) * yDiff;
    return (fabs(xn) < length / 2 && fabs(yn) < width / 2);
  }

  bool Utils::isOrientationClose(geometry_msgs::Quaternion orientA,
      geometry_msgs::Quaternion orientB,
      float diff_thres)
  {
    double yawDiff = tf::getYaw(orientA) - tf::getYaw(orientB);
    if (yawDiff < 0)
      yawDiff += 2 * PI;
    if (yawDiff > PI)
      yawDiff = 2 * PI - yawDiff;
    return yawDiff < diff_thres;
  }

  float Utils::probabilityFromStdDev(float boundingRadius, float deviation)
  {
    if (boundingRadius <= 0)
    {
      throw std::range_error("Bounding radius haw always a positive value.");
    }
    if (deviation < 0)
    {
      throw std::range_error("Standard deviation is a positive value.");
    }
    if (deviation == 0)
      return 1;
    float x = boundingRadius / deviation;
    return 1 - exp(-pow(x, 2) / 2);
  }

  float Utils::stdDevFromProbability(float boundingRadius, float probability)
  {
    if (boundingRadius <= 0)
    {
      throw std::range_error("Bounding radius haw always a positive value.");
    }
    if (probability > 1 || probability < 0)
    {
      throw std::range_error("Probability value is between 0 and 1.");
    }
    return boundingRadius / sqrt(log(pow(1 - probability, -2)));
  }

}  // namespace pandora_data_fusion_utils
}  // namespace pandora_data_fusion
