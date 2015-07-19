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
 *     from this hardware without specific prior written permission.
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

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PROCESSOR_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PROCESSOR_H

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include "sensor_processor/processor.h"
#include "sensor_processor/handler.h"
#include "pandora_vision_common/cv_mat_stamped.h"

#include "pandora_vision_obstacle/hard_obstacle_cfgConfig.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class HardObstacleProcessor : public sensor_processor::Processor<CVMatStamped, CVMatStamped>
  {
   public:
    void
    initialize(const std::string& ns, sensor_processor::Handler* handler);
    HardObstacleProcessor();

   public:
    virtual bool process(const CVMatStampedConstPtr& input,
        const CVMatStampedPtr& output);

    private:
    /**
      * @brief The function called when a parameter is changed
      * @param config [const pandora_vision_obstacle::hard_obstacle_cfgConfig&]
      * @param level [const uint32_t] The level
      **/
    void parametersCallback(const ::pandora_vision_obstacle::hard_obstacle_cfgConfig& config,
        uint32_t level);

   private:
    /// The dynamic reconfigure parameters' server
    boost::shared_ptr< dynamic_reconfigure::Server< ::pandora_vision_obstacle::hard_obstacle_cfgConfig > >
      serverPtr_;

    /// The dynamic reconfigure parameters' callback
    dynamic_reconfigure::Server< ::pandora_vision_obstacle::hard_obstacle_cfgConfig >
      ::CallbackType f;

    boost::shared_ptr<HardObstacleDetector> detector_;
  };
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_PROCESSOR_H
