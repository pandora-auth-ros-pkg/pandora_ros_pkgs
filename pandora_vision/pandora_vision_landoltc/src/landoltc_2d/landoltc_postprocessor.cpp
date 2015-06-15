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

#include "pandora_vision_landoltc/landoltc_2d/landoltc_postprocessor.h"

namespace pandora_vision
{
  LandoltCPostProcessor::LandoltCPostProcessor(const std::string& ns, sensor_processor::Handler* handler) :
    VisionPostProcessor<pandora_vision_msgs::LandoltcAlertVector>(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] postprocessor nh processor : " +
      this->accessProcessorNh()->getNamespace());
  }

  LandoltCPostProcessor::~LandoltCPostProcessor() {}

  bool LandoltCPostProcessor::postProcess(const POIsStampedConstPtr& input, const LandoltcAlertVectorPtr& output)
  {
    pandora_common_msgs::GeneralAlertVector alertVector = getGeneralAlertInfo(input);
    output->header = alertVector.header;
    if (alertVector.alerts.size() == 0)
      return false;

    for (int ii = 0; ii < alertVector.alerts.size(); ii++)
    {
      pandora_vision_msgs::LandoltcAlert landoltcAlert;

      landoltcAlert.info.yaw = alertVector.alerts[ii].yaw;
      landoltcAlert.info.pitch = alertVector.alerts[ii].pitch;
      landoltcAlert.info.probability = alertVector.alerts[ii].probability;

      boost::shared_ptr<LandoltCPOI> landoltCPOI(boost::dynamic_pointer_cast<LandoltCPOI>(input->pois[ii]));
      landoltcAlert.angles = landoltCPOI->getAngles();
      landoltcAlert.posterior = landoltCPOI->getProbability();

      output->alerts.push_back(landoltcAlert);
    }
    return true;
  }
}  // namespace pandora_vision
