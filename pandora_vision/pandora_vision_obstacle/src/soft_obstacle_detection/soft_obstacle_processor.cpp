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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include <string>
#include "pandora_vision_obstacle/soft_obstacle_detection/soft_obstacle_processor.h"

namespace pandora_vision
{
  SoftObstacleProcessor::SoftObstacleProcessor(const std::string& ns,
    sensor_processor::Handler* handler) : sensor_processor::Processor<ImagesStamped,
    POIsStamped>(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
      this->accessProcessorNh()->getNamespace());

    detector_.reset(new SoftObstacleDetector(this->getName(),
          *this->accessPublicNh()));

    server.setCallback(boost::bind(&SoftObstacleProcessor::parametersCallback,
        this, _1, _2));
  }

  SoftObstacleProcessor::SoftObstacleProcessor() : sensor_processor::Processor<ImagesStamped,
    POIsStamped>() {}

  void SoftObstacleProcessor::parametersCallback(
      const pandora_vision_obstacle::soft_obstacle_cfgConfig& config,
      const uint32_t& level)
  {
    detector_->setShowOriginalImage(config.showOriginalImage);
    detector_->setShowDWTImage(config.showDWTImage);
    detector_->setShowOtsuImage(config.showOtsuImage);
    detector_->setShowDilatedImage(config.showDilatedImage);
    detector_->setShowVerticalLines(config.showVerticalLines);
    detector_->setShowROI(config.showROI);
  }

  bool SoftObstacleProcessor::process(const ImagesStampedConstPtr& input,
    const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getRgbImage().cols;
    output->frameHeight = input->getRgbImage().rows;

    output->pois = detector_->detectSoftObstacle(
        input->getRgbImage(), input->getDepthImage());

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision
