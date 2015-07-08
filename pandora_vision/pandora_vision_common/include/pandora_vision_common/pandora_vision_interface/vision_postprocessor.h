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

#ifndef PANDORA_VISION_COMMON_PANDORA_VISION_INTERFACE_VISION_POSTPROCESSOR_H
#define PANDORA_VISION_COMMON_PANDORA_VISION_INTERFACE_VISION_POSTPROCESSOR_H

#include <string>

#include "sensor_processor/handler.h"
#include "sensor_processor/postprocessor.h"

#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_common/pandora_vision_utilities/general_alert_converter.h"

namespace pandora_vision
{
  template <class VisionAlertMsg>
  class VisionPostProcessor : public sensor_processor::PostProcessor<POIsStamped, VisionAlertMsg>
  {
   private:
    typedef boost::shared_ptr<VisionAlertMsg> VisionAlertMsgPtr;

   public:
    /**
     * @brief Constructor
     * @param ns [const std::string&] The namespace of this postprocessor's nodeHandle
     * @param handler [sensor_processor::AbstractHandler*] A pointer of the class that
     * handles this postprocessor
     **/
    VisionPostProcessor();

   protected:
    /**
     * @brief Function that calculates parameters yaw and pitch for every POI, given its
     * coordinates, and puts them in a structure with POI's probability and timestamp
     * @param result [const POIsStampedConstPtr&] A constant reference to a constant shared
     * pointer of a POI with timestamp
     * @return [pandora_common_msgs::GeneralAlertInfoVector] ROS message type that contains
     * yaw, pitch and probability of every POI in the processed frame and the frame's header
     **/
    pandora_common_msgs::GeneralAlertVector
    getGeneralAlertInfo(const POIsStampedConstPtr& result);

   private:
    boost::shared_ptr<GeneralAlertConverter> converter_;
  };

  template <class VisionAlertMsg>
  VisionPostProcessor<VisionAlertMsg>::
  VisionPostProcessor() :
    sensor_processor::PostProcessor<POIsStamped, VisionAlertMsg>(),
    converter_(new GeneralAlertConverter)
  {}

  template <class VisionAlertMsg>
  pandora_common_msgs::GeneralAlertVector
  VisionPostProcessor<VisionAlertMsg>::
  getGeneralAlertInfo(const POIsStampedConstPtr& result)
  {
    return converter_->getGeneralAlertVector(this->getPublicNodeHandle(), *result);
  }

}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_PANDORA_VISION_INTERFACE_VISION_POSTPROCESSOR_H
