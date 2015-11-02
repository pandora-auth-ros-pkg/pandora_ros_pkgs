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

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_POSTPROCESSOR_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_POSTPROCESSOR_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

#include "sensor_processor/postprocessor.h"
#include "sensor_processor/handler.h"
#include "pandora_vision_common/cv_mat_stamped.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class HardObstaclePostProcessor :
    public sensor_processor::PostProcessor<CVMatStamped, nav_msgs::OccupancyGrid>
  {
   public:
    HardObstaclePostProcessor();

    void
    initialize(const std::string& ns, sensor_processor::Handler* handler);

    /**
      * @brief Converts an OpenCV matrix that represents a traversability map
      * to a Occupancy Grid Format.
      * @param inputImage[const CVMatStampedConstPtr&] The input elevation map that will
      * converted.
      * @param outputOccupancyGrid[const nav_msgs::OccupancyGridPtr&] The resulting elevation
      * map in Occupancy Grid format.
      * @return bool True if the conversion was successful, false otherwise.
      */
    virtual bool
    postProcess(const CVMatStampedConstPtr& input, const nav_msgs::OccupancyGridPtr& output);

    void
    updateMap(const nav_msgs::OccupancyGridConstPtr& mapConstPtr);

    void obstacleBlur(const nav_msgs::OccupancyGridPtr& inputOutput);


   private:
    void
    obstacleDilation(const nav_msgs::OccupancyGridPtr& output, int steps, int coords);

   private:
    nav_msgs::OccupancyGridConstPtr map_const_ptr_;

    ros::Subscriber map_subscriber_;
    std::string map_topic_;

    tf::TransformListener tfListener_;

    int UNKNOWN_VALUE;
    double MAT_RESOLUTION;
    std::string LOCAL_FRAME;
  };

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_POSTPROCESSOR_H
