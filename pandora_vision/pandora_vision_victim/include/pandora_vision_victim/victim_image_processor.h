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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_VICTIM_VICTIM_IMAGE_PROCESSOR_H
#define PANDORA_VISION_VICTIM_VICTIM_IMAGE_PROCESSOR_H

#include <string>
#include <vector>

#include "sensor_processor/processor.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_victim/enhanced_image_stamped.h"
#include "pandora_vision_victim/victim_poi.h"
#include "pandora_vision_victim/victim_parameters.h"
#include "pandora_vision_victim/classifiers/svm_validator.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class VictimImageProcessor : public sensor_processor::Processor<EnhancedImageStamped, POIsStamped>
  {
    public:
      VictimImageProcessor(const std::string& ns, sensor_processor::Handler* handler);
      VictimImageProcessor();

      virtual ~VictimImageProcessor();

      virtual bool process(const EnhancedImageStampedConstPtr&  input, const POIsStampedPtr& output);

    private:
      VictimParameters params_;

      /**
      @brief
      @return void
      **/
      std::vector<VictimPOIPtr> detectVictims(const EnhancedImageStampedConstPtr& input);

      /**
      @brief Function that enables suitable subsystems, according
      to the current State 
      @param
      @return void
      **/ 
      std::vector<VictimPOIPtr> victimFusion(const EnhancedImageStampedConstPtr& input,
        bool depthEnable);

      std::vector<cv::Mat> _rgbdImages;

      /// Instance of RGB SVM Validator
      boost::shared_ptr<SvmValidator> rgbSvmValidatorPtr_;
      /// Instance of Depth SVM Validator
      boost::shared_ptr<SvmValidator> depthSvmValidatorPtr_;

      /// Debug purposes
      image_transport::Publisher _debugVictimsPublisher;
      cv::Mat debugImage;
      std::vector<cv::KeyPoint> rgb_svm_keypoints;
      std::vector<cv::KeyPoint> depth_svm_keypoints;
      std::vector<cv::Rect> rgb_svm_bounding_boxes;
      std::vector<cv::Rect> depth_svm_bounding_boxes;
      std::vector<float> rgb_svm_p;
      std::vector<float> depth_svm_p;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_IMAGE_PROCESSOR_H
