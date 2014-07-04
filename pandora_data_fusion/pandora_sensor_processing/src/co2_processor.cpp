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
#include <limits>

#include "sensor_processing/co2_processor.h"

namespace pandora_sensor_processing
{

  Co2Processor::Co2Processor(const std::string& ns)
    : SensorProcessor<Co2Processor>(ns, "co2")
  {
    ambientCo2_ = std::numeric_limits<double>::max();
    spikeFound_ = false;
    spikeTime_ = 0;
  }

  /**
   * @details Weibull distribution is used for calculating probability.
   */
  void Co2Processor::sensorCallback(
      const pandora_arm_hardware_interface::Co2Msg& msg)
  {
    ROS_DEBUG_NAMED("SENSOR_PROCESSING", "[%s] Incoming co2 raw info.", name_.c_str());
    alert_.yaw = 0;
    alert_.pitch = 0;
    alert_.header = msg.header;
    // Measurement has no information because it has the same value with the ambience.
    if (msg.co2_percentage < ambientCo2_)
    {
      ambientCo2_ = msg.co2_percentage;
      spikeFound_ = false;
    }
    else if (msg.co2_percentage > ambientCo2_)
    {
      if (!spikeFound_)
      {
        spikeFound_ = true;
        spikeTime_ = ros::Time::now().toSec();
        return;
      }
      double timeFromSpike = ros::Time::now().toSec() - spikeTime_;
      alert_.probability = Utils::weibullPdf(timeFromSpike,
          SHAPE_PARAMETER, TIME_CONSTANT);
      publishAlert();
      if (alert_.probability < PROBABILITY_THRES)
      {
        ambientCo2_ = msg.co2_percentage;
        spikeFound_ = false;
      }
    }
    else
    {
      spikeFound_ = false;
    }
  }

  void Co2Processor::dynamicReconfigCallback(
      const SensorProcessingConfig& config, uint32_t level)
  {
    TIME_CONSTANT = config.co2_time_constant;
    SHAPE_PARAMETER = config.co2_pdf_shape;
    PROBABILITY_THRES = config.co2_probability_threshold;
  }

}  // namespace pandora_sensor_processing
