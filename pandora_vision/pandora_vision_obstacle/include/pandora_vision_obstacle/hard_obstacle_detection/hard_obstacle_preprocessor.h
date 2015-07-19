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
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PREPROCESSOR_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PREPROCESSOR_H

#include <string>
#include <limits>

#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include "sensor_processor/preprocessor.h"
#include "sensor_processor/handler.h"
#include "pandora_vision_common/cv_mat_stamped.h"

#include "pandora_vision_obstacle/elevation_mapConfig.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class HardObstaclePreProcessor : public sensor_processor::PreProcessor<sensor_msgs::PointCloud2,
  CVMatStamped>
  {
   public:
    typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;
    typedef boost::shared_ptr<sensor_msgs::PointCloud2 const> PointCloud2ConstPtr;

   public:
    HardObstaclePreProcessor();
    void
    initialize(const std::string& ns, sensor_processor::Handler* handler);

    virtual bool preProcess(const PointCloud2ConstPtr& input,
        const CVMatStampedPtr& output);

    /**
      * @brief Converts an Point Cloud to a local elevation map in OpenCV matrix
      * format.
      * @param inputPointCloud[const PointCloud2ConstPtr&] The Point Cloud received
      * from the RGBD sensor.
      * @param outputImgPtr[const CVMatStampedPtr&] The resulting elevation
      * map as an OpenCV matrix.
      * @return bool True if the conversion was successful, false otherwise.
      */
    bool PointCloudToCvMat(const PointCloud2ConstPtr& inputPointCloud,
        const CVMatStampedPtr& outputImgPtr);

    void reconfCallback(const ::pandora_vision_obstacle::elevation_mapConfig params,
        uint32_t level);

    void viewElevationMap(const CVMatStampedPtr& elevationMapStamped);

   private:
    inline int convertToXCoord(double meters);
    inline int convertToYCoord(double meters);

   private:
    ros::Publisher imagePublisher_;

    /// The maximum distance of a point from the range sensor.
    double maxAllowedDist_;

    /// The minimum elevation from the base of the robot.
    double minElevation_;

    /// The maximum elevation from the base of the robot.
    double maxElevation_;

    /// The frame Id for the elevation Map frame of reference.
    std::string baseFootPrintFrameId_;

    /// The frame Id for the Point Cloud sensor.
    std::string pclSensorFrameId_;

    /// The ros subscriber used to get the transformation from the range sensor
    /// to the elevation Map frame of reference.
    tf::TransformListener tfListener_;

    /// The width of the elevation map.
    int elevationMapWidth_;

    /// The height of the elevation map.
    int elevationMapHeight_;

    /// Flag used to view the elevation map.
    bool visualisationFlag_;

    /// Meters of pointcloud measurements per grid cell
    double gridResolution_;

    /// The dynamic reconfigure server used to changed the parameters for the elevation map
    /// on runtime.
    boost::shared_ptr< dynamic_reconfigure::Server< ::pandora_vision_obstacle::elevation_mapConfig > >
      reconfServerPtr_;

    /// The callback for the dynamic reconfigure server.
    dynamic_reconfigure::Server< ::pandora_vision_obstacle::elevation_mapConfig >::CallbackType
      reconfCallback_;
  };

  const double unknownElevation = - std::numeric_limits<double>::max();

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PREPROCESSOR_H
