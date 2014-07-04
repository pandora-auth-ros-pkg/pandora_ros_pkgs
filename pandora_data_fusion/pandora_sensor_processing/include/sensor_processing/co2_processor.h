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

#ifndef SENSOR_PROCESSING_CO2_PROCESSOR_H
#define SENSOR_PROCESSING_CO2_PROCESSOR_H

#include <string>
#include <limits>

#include "pandora_arm_hardware_interface/Co2Msg.h"
#include "sensor_processing/utils.h"
#include "sensor_processing/sensor_processor.h"

namespace pandora_sensor_processing
{

  class Co2Processor : public SensorProcessor<Co2Processor>
  {
    public:
      /**
       * @brief Constructor
       * @param ns [std::string const&] Has the namespace of the node.
       */
      explicit Co2Processor(const std::string& ns);

      /* Methods that SensorProcessor needs */

      /**
       * @brief callback to co2SensorSubscriber_
       * @param msg [pandora_arm_hardware_interface::Co2MsgConstPtr const&] contains
       * co2 percentage (v/v) in the air
       * @return void
       */
      void sensorCallback(
          const pandora_arm_hardware_interface::Co2Msg& msg);

      void dynamicReconfigCallback(
          const SensorProcessingConfig& config, uint32_t level);

    private:
      //!< Estimated ambient co2 concentration.
      double ambientCo2_;
      //!< Spike found?
      bool spikeFound_;
      //!< Time measurement spike was found.
      double spikeTime_;

      /*  Parameters  */
      //!< Time constant of weibull pdf
      double TIME_CONSTANT;
      //!< Shape constant of weibull pdf
      double SHAPE_PARAMETER;
      //!< Probability which determines whether a spike is ambience or not
      float PROBABILITY_THRES;
  };

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_CO2_PROCESSOR_H
