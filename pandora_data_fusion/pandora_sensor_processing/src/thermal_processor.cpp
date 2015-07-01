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

#include "pandora_sensor_processing/thermal_processor.h"

namespace pandora_sensor_processing
{

  ThermalProcessor::
    ThermalProcessor(const std::string& ns) :
      SensorProcessor<ThermalProcessor>(ns, "thermal")
  {
    MAX_CLUSTER_MEMORY = 3;
    MAX_CLUSTER_ITERATIONS = 100;
    OPTIMAL_HEAT_DIFFERENCE = 5;
    OPTIMAL_TEMPERATURE = 36;
    THERMAL_STD_DEV = 1;
    THERMAL_X_FOV = 45 * PI / 180;
    THERMAL_Y_FOV = 60 * PI / 180;
  }

  /**
   * @details thermal processor keeps track of frame_ids, and as a consequence
   * of the existing thermal cameras, by registering a new clusterer for each
   * new-comer frame-id (through raw data message). Keeping distinct clusterers
   * is a necessity because each clusterer keeps history of data from a thermal,
   * so that data is as continuous as possible.
   */
  void ThermalProcessor::
    sensorCallback(const sensor_msgs::Image& msg)
  {
    ROS_DEBUG_NAMED("SENSOR_PROCESSING",
        "[%s] Incoming thermal raw measurement.", name_.c_str());
    if (frameToClusterer_.find(msg.header.frame_id) ==
        frameToClusterer_.end())
    {
      frameToClusterer_[msg.header.frame_id] = ClustererPtr(
          new Clusterer(msg.height * msg.width,
            MAX_CLUSTER_MEMORY, MAX_CLUSTER_ITERATIONS) );
    }
      FrameToClusterer::const_iterator
        it = frameToClusterer_.find(msg.header.frame_id);
      if (analyzeImage(msg, (*it).second))
      {
        if (getResults(msg, (*it).second))
        {
          publishAlert();
        }
      }
  }

  void ThermalProcessor::
    dynamicReconfigCallback(
      const SensorProcessingConfig& config, uint32_t level)
  {
    MAX_CLUSTER_MEMORY = config.thermal_max_cluster_memory;
    MAX_CLUSTER_ITERATIONS = config.thermal_max_cluster_iterations;
    OPTIMAL_HEAT_DIFFERENCE = config.optimal_heat_difference;
    OPTIMAL_TEMPERATURE = config.optimal_temperature;
    THERMAL_STD_DEV = config.thermal_std_dev;
    THERMAL_X_FOV = config.thermal_x_fov_degrees * PI / 180;
    THERMAL_Y_FOV = config.thermal_y_fov_degrees * PI / 180;
  }

  /**
   * @details It returns false if measurement is not of consistent size.
   * This is spotted by an exception that is raised by renewDataSet.
   */
  bool ThermalProcessor::
    analyzeImage(const sensor_msgs::Image& msg,
      const ClustererPtr& clusterer)
  {
    Eigen::MatrixXf measurement(4, msg.height * msg.width);
    for (int ii = 0; ii < msg.height; ++ii)
    {
      for (int jj = 0; jj < msg.width; ++jj)
      {
        measurement(0, ii * msg.width + jj) = jj;
        measurement(1, ii * msg.width + jj) = ii;
        measurement(2, ii * msg.width + jj) = msg.header.stamp.toSec();
        measurement(3, ii * msg.width + jj) = msg.data[ii * msg.width + jj];
      }
    }
    try
    {
      clusterer->renewDataSet(measurement);
      clusterer->cluster();
    }
    catch (std::exception& err)
    {
      ROS_DEBUG_NAMED("SENSOR_PROCESSING",
          "[%s/ANALYZE_IMAGE] %s", name_.c_str(), err.what());
      return false;
    }
    return true;
  }

  bool ThermalProcessor::
    getResults(const sensor_msgs::Image& msg,
        const ClustererConstPtr& clusterer)
    {
      Eigen::Vector4f center;
      float mean1 = clusterer->getMean1()(3);
      float mean2 = clusterer->getMean2()(3);
      float diff = mean1 - mean2;

      if (fabs(diff) > OPTIMAL_HEAT_DIFFERENCE)
      {
        if (mean1 > mean2)
        {
          if (!clusterer->getCurrentMean1(&center))
            return false;
        }
        else
        {
          if (!clusterer->getCurrentMean2(&center))
            return false;
        }
      }
      else
        return false;

      // OK. Warmer cluster has a cell from current measurement.
      // Considering cluster to be a valid alert!
      ROS_INFO_THROTTLE_NAMED(10, "SENSOR_PROCESSING",
          "[%s] Found thermal alert with temperature: %f", name_.c_str(), center(3));
      alert_.info.probability = Utils::normalPdf(center(3),
          OPTIMAL_TEMPERATURE, THERMAL_STD_DEV);
      float x = center(0) - static_cast<float>(msg.width) / 2;
      float y = static_cast<float>(msg.height) / 2 - center(1);
      alert_.info.yaw = atan(2 * x / msg.width * tan(THERMAL_X_FOV / 2));
      alert_.info.pitch = atan(2 * y / msg.height * tan(THERMAL_Y_FOV / 2));
      alert_.header = msg.header;
      return true;
    }

}  // namespace pandora_sensor_processing
