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
 *   Choutas Vassilis
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include "pandora_vision_hazmat/detection/hazmat_postprocessor.h"

namespace pandora_vision
{
  HazmatPostProcessor::HazmatPostProcessor(const std::string& ns, sensor_processor::Handler* handler) :
    VisionPostProcessor<pandora_vision_msgs::HazmatAlertVector>(ns, handler)
  {
  }

  HazmatPostProcessor::~HazmatPostProcessor()
  {
  }

  bool HazmatPostProcessor::postProcess(const POIsStampedConstPtr& input, const HazmatAlertVectorPtr& output)
  {
    pandora_common_msgs::GeneralAlertVector alertVector = getGeneralAlertInfo(input);
    output->header = alertVector.header;

    for (int ii = 0; ii < alertVector.generalAlerts.size(); ii++)
    {
      pandora_vision_msgs::HazmatAlert hazmatAlert;

      hazmatAlert.info.yaw = alertVector.generalAlerts[ii].yaw;
      hazmatAlert.info.pitch = alertVector.generalAlerts[ii].pitch;

      boost::shared_ptr<HazmatPOI> hazmatPOI(boost::dynamic_pointer_cast<HazmatPOI>(input->pois[ii]));
      hazmatAlert.patternType = hazmatPOI->getPattern();

      output->hazmatAlerts.push_back(hazmatAlert);
    }
    return true;
  }
}  // namespace pandora_vision