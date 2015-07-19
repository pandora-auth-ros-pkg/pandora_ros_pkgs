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

#include <string>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "sensor_processor/handler.h"
#include "sensor_processor/processor_error.h"
#include "pandora_vision_common/cv_mat_stamped.h"

#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_postprocessor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  HardObstaclePostProcessor::
  HardObstaclePostProcessor() :
    sensor_processor::PostProcessor<CVMatStamped, nav_msgs::OccupancyGrid>(),
    tfListener_(this->getProcessorNodeHandle())
  {
  }

  void
  HardObstaclePostProcessor::
  initialize(const std::string& ns, sensor_processor::Handler* handler)
  {
    sensor_processor::PostProcessor<CVMatStamped, nav_msgs::OccupancyGrid>::
      initialize(ns, handler);

    ros::NodeHandle processor_nh = this->getProcessorNodeHandle();

    if (!processor_nh.getParam("map_topic", map_topic_))
    {
      NODELET_FATAL("[%s] Could not find map topic param", this->getName().c_str());
      ROS_BREAK();
    }

    processor_nh.param<int>("unknown_value", UNKNOWN_VALUE, 51);
    processor_nh.param<double>("mat_resolution", MAT_RESOLUTION, 0.02);
    processor_nh.param<std::string>("local_frame", LOCAL_FRAME, "/base_foorprint");

    map_subscriber_ = this->getPublicNodeHandle().subscribe(map_topic_, 1,
        &HardObstaclePostProcessor::updateMap, this);

    tf::StampedTransform tfTransform;
    try
    {
      tfListener_.waitForTransform("/world", "/map", ros::Time(0), ros::Duration(2));
      tfListener_.lookupTransform("/world", "/map", ros::Time(0), tfTransform);
    }
    catch (const tf::TransformException& ex)
    {
      sensor_processor::processor_error(ex.what());
    }
  }

  /**
   * @brief Converts an OpenCV matrix that represents a traversability map
   * to a Occupancy Grid Format.
   * @param inputImage[const CVMatStampedConstPtr&] The input elevation map that will
   * converted.
   * @param outputOccupancyGrid[const nav_msgs::OccupancyGridPtr&] The resulting elevation
   * map in Occupancy Grid format.
   * @return bool True if the conversion was successful, false otherwise.
  */
  bool
  HardObstaclePostProcessor::
  postProcess(const CVMatStampedConstPtr& input, const nav_msgs::OccupancyGridPtr& output)
  {
    if (map_const_ptr_.get() == NULL)
    {
      NODELET_ERROR("[%s] Map pointer is still uninitialized!", this->getName().c_str());
      throw sensor_processor::processor_error("Map pointer is still uninitialized!");
    }
    NODELET_INFO("[%s] postprocessing to costmap", this->getName().c_str());

    output->header.frame_id = map_const_ptr_->header.frame_id;
    output->header.stamp = input->getHeader().stamp;
    output->info = map_const_ptr_->info;

    output->data.clear();
    output->data.resize(map_const_ptr_->data.size(), UNKNOWN_VALUE);

    // Get robot base footprint transform
    tf::StampedTransform baseTransform;
    try
    {
      tfListener_.waitForTransform(LOCAL_FRAME, output->header.frame_id,
          output->header.stamp, ros::Duration(0.2));
      tfListener_.lookupTransform(LOCAL_FRAME, output->header.frame_id,
          output->header.stamp, baseTransform);
    }
    catch (const tf::TransformException& ex)
    {
      throw sensor_processor::processor_error(ex.what());
    }

    tf::Quaternion baseOrientation;
    baseTransform.getBasis().getRotation(baseOrientation);
    tf::Vector3 origin = baseTransform.getOrigin();

    double yawDiff = tf::getYaw(map_const_ptr_->info.origin.orientation) -
      tf::getYaw(baseOrientation);
    double xDiff = map_const_ptr_->info.origin.position.x - origin[0];
    double yDiff = map_const_ptr_->info.origin.position.y - origin[1];

    int trans_ii, trans_jj;
    double x, y, xn, yn;
    for (int ii = 0; ii < input->image.cols; ++ii) {
      for (int jj = 0; jj < input->image.rows; ++jj) {
        trans_ii = ii - input->image.cols / 2;
        trans_jj = jj - input->image.rows / 2;
        x = trans_ii * MAT_RESOLUTION;
        y = trans_jj * MAT_RESOLUTION;
        xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
        yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
        int coords = static_cast<int>(round(xn / output->info.resolution)) +
              static_cast<int>(round(yn / output->info.resolution)) * output->info.width;
        if (coords >= output->data.size() || coords < 0)
        {
          NODELET_WARN("[%s] Error resizing to: %d\nCoords Xn: %f, Yn: %f\n",
              this->getName().c_str(), static_cast<int>(map_const_ptr_->data.size()), xn, yn);
        }
        else
        {
          output->data[coords] = static_cast<int8_t>(input->image.at<uchar>(jj, ii));
          obstacleDilation(output, 1, coords);
        }
      }
    }

    return true;
  }

  void
  HardObstaclePostProcessor::
  updateMap(const nav_msgs::OccupancyGridConstPtr& mapConstPtr)
  {
    map_const_ptr_ = mapConstPtr;
  }

  void
  HardObstaclePostProcessor::
  obstacleDilation(const nav_msgs::OccupancyGridPtr& output, int steps, int coords)
  {
    if (steps == 0)
      return;

    int8_t cell = output->data[coords];

    if (cell != 0 && cell != UNKNOWN_VALUE)  // That's foreground
    {
      // Check for all adjacent
      if (output->data[coords + output->info.width + 1] == 0)
      {
        output->data[coords + output->info.width + 1] = cell;
        obstacleDilation(output, steps - 1, coords + output->info.width + 1);
      }
      if (output->data[coords + output->info.width] == 0)
      {
        output->data[coords + output->info.width] = cell;
      }
      if (output->data[coords + output->info.width - 1] == 0)
      {
        output->data[coords + output->info.width - 1] = cell;
        obstacleDilation(output, steps - 1, coords + output->info.width - 1);
      }
      if (output->data[coords + 1] == 0)
      {
        output->data[coords + 1] = cell;
      }
      if (output->data[coords - 1] == 0)
      {
        output->data[coords - 1] = cell;
      }
      if (output->data[coords - output->info.width + 1] == 0)
      {
        output->data[coords - output->info.width + 1] = cell;
        obstacleDilation(output, steps - 1, coords - output->info.width + 1);
      }
      if (output->data[coords - output->info.width] == 0)
      {
        output->data[coords - output->info.width] = cell;
      }
      if (output->data[coords - output->info.width - 1] == 0)
      {
        output->data[coords - output->info.width - 1] = cell;
        obstacleDilation(output, steps - 1, coords - output->info.width - 1);
      }
    }
  }

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
