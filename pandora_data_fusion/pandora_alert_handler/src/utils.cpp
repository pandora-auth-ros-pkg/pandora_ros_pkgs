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

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Point Utils::point2DAndHeight2Point3D(Point position, float height)
    {
      position.z = height;
      return position;
    }

    float Utils::distanceBetweenPoints2D(Point a, Point b)
    {
      float xDist = a.x - b.x;
      float yDist = a.y - b.y;

      return sqrt((xDist * xDist) + (yDist * yDist));
    }

    float Utils::distanceBetweenPoints3D(Point a, Point b)
    {
      float xDist = a.x - b.x;
      float yDist = a.y - b.y;
      float zDist = a.z - b.z;

      return sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
    }

    geometry_msgs::Quaternion Utils::calculateQuaternion(Point a, Point b)
    {
      tfScalar yaw;

      yaw = atan2(b.y - a.y, b.x - a.x);

      return tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    }

    Point Utils::vector3ToPoint(tf::Vector3 vector)
    {
      Point point;
      point.x = vector[0];
      point.y = vector[1];
      point.z = vector[2];

      return point;
    }

    bool Utils::arePointsInRange(Point pointA, Point pointB, float sensor_range)
    {
      float dist = distanceBetweenPoints2D(pointA, pointB);

      if (dist > sensor_range)
        return false;
      else
        return true;
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

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

