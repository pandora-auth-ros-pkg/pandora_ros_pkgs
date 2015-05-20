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

#ifndef SENSOR_PROCESSING_THERMAL_PROCESSOR_H
#define SENSOR_PROCESSING_THERMAL_PROCESSOR_H

#include <map>
#include <string>

#include "sensor_msgs/Image.h"
#include "pandora_sensor_processing/clusterer.h"
#include "pandora_sensor_processing/sensor_processor.h"

namespace pandora_sensor_processing
{

  //!< Type Definitions
  typedef std::map<std::string, ClustererPtr> FrameToClusterer;

  class ThermalProcessor : public SensorProcessor<ThermalProcessor>
  {
    public:
      /**
       * @brief Constructor
       * @param ns [std::string const&] Has the namespace of the node.
       */
      explicit ThermalProcessor(const std::string& ns);

      /* Methods that SensorProcessor needs */

      /**
       * @brief callback to co2SensorSubscriber_
       * @param msg [sensor_msgs::Image const&] msg containing a grid
       * (as image) of pixels representing temperatures.
       * @return void
       */
      void sensorCallback(const sensor_msgs::Image& msg);

      void dynamicReconfigCallback(
          const SensorProcessingConfig& config, uint32_t level);

    private:
      /**
       * @brief Converses msg to Eigen::MatrixXf form to fit clusterer,
       * updates clusterer with the new data and calls it to cluster.
       * @param msg [sensor_msgs::Image const&] new measurement
       * @param clusterer [ClustererPtr const&] clusterer responsible for
       * clustering measurements from the same frame as msg.
       * @return bool true if clustering was done, false if a problem aroused.
       */
      bool analyzeImage(const sensor_msgs::Image& msg,
          const ClustererPtr& clusterer);

      /**
       * @brief Extracts info if clustering contains useful information
       * and thus can be made into an alert.
       * @param msg [sensor_msgs::Image const&] new measurement
       * @param clusterer [ClustererPtr const&] clusterer responsible for
       * clustering measurements from the same frame as msg.
       * @return bool true if a cluster progresses to an alert.
       */
      bool getResults(const sensor_msgs::Image& msg,
          const ClustererConstPtr& clusterer);

    private:
      FrameToClusterer frameToClusterer_;

      //!< params

      //!< how many measurements clusterer will track to cluster.
      int MAX_CLUSTER_MEMORY;
      //!< iteration limit of clustering algorithm.
      int MAX_CLUSTER_ITERATIONS;
      //!< how many degrees warmer should be the candidate cluster (alert)
      //!< from the other (environment) to be valid.
      float OPTIMAL_HEAT_DIFFERENCE;
      //!< actual thermal source's temperature. Peak of discrete gaussian
      //!< probability metric.
      float OPTIMAL_TEMPERATURE;
      //!< chosen standard deviation of gaussian metric.
      float THERMAL_STD_DEV;
      //!< Horizontal field of view of thermal camera (in radians).
      float THERMAL_X_FOV;
      //!< Vertical field of view of thermal camera (in radians).
      float THERMAL_Y_FOV;

    private:
      friend class ThermalProcessorTest;
  };

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_THERMAL_PROCESSOR_H
