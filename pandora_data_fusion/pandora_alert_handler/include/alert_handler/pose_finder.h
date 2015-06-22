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

#ifndef ALERT_HANDLER_POSE_FINDER_H
#define ALERT_HANDLER_POSE_FINDER_H

#include <utility>
#include <vector>
#include <string>
#include <boost/utility.hpp>
#include <boost/array.hpp>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/OccupancyGrid.h>

#include "alert_handler/defines.h"
#include "alert_handler/tf_finder.h"
#include "alert_handler/tf_listener.h"
#include "alert_handler/utils.h"
#include "alert_handler/base_object.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  class PoseFinder : private boost::noncopyable
  {
   public:
    PoseFinder(const MapPtr& map, const std::string& mapType);

    geometry_msgs::Pose findAlertPose(double alertYaw, double alertPitch,
                       const tf::Transform& tfTransform);
    geometry_msgs::Pose findPoseFromPoints(
        const boost::array<double, 4>& pointsYaw,
        const boost::array<double, 4>& pointsPitch,
        const boost::array<double, 4>& pointsDepth,
        const tf::Transform& tfTransform, double* length);
    geometry_msgs::Point projectAlertPosition(double alertYaw, double alertPitch,
        double depth, const tf::Transform& tfTransform);
    tf::Transform lookupTransformFromWorld(const std_msgs::Header& header);

    geometry_msgs::Quaternion findAppropriateOrientation(
        const geometry_msgs::Point& framePoint, const geometry_msgs::Point& alertPoint);

    void updateParams(float occupiedCellThres,
                      float heightHighThres, float heightLowThres,
                      float orientationCircle);

   private:
    geometry_msgs::Point positionOnWall(const tf::Transform& transform);

    float calcHeight(const tf::Transform& transform, float distFromAlert);

    std::pair<geometry_msgs::Point, geometry_msgs::Point> findDiameterEndPointsOnWall(
        std::vector<geometry_msgs::Point> points);

   private:
    MapPtr map_;

    TfListenerPtr listener_;

    /*  Parameters  */
    float ORIENTATION_CIRCLE;
    float HEIGHT_HIGH_THRES;
    float HEIGHT_LOW_THRES;
    float OCCUPIED_CELL_THRES;

   private:
    friend class PoseFinderTest;
  };

  typedef boost::scoped_ptr< PoseFinder > PoseFinderPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_POSE_FINDER_H
