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
 * Authors: Choutas Vassilis
 *********************************************************************/

#ifndef PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_PROCESSOR_H
#define PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_PROCESSOR_H

#include <string>
#include <dynamic_reconfigure/server.h>

#include "pandora_vision_common/pandora_vision_interface/vision_processor.h"
#include "pandora_vision_hazmat/DisplayConfig.h"
#include "pandora_vision_hazmat/HazmatNodeConfig.h"
#include "pandora_vision_hazmat/detection/detector_factory.h"
#include "pandora_vision_hazmat/detection/hazmat_poi.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    class HazmatProcessor : public VisionProcessor
    {
      public:
        /**
         * @brief : Default Constructor that initiliazes the hazmat detector
         * and the necessary ROS objects for communication.
         */
        HazmatProcessor(const std::string& ns, sensor_processor::Handler* handler);

        /**
         * @brief : Constructor used for testing.
         */
        HazmatProcessor();

        /**
         * @brief : Class destructor that destroys the current detector.
         */
        virtual ~HazmatProcessor();

        /**
         * @brief : Class method that is used by the dynamic reconfigure
         * server to change object parameters.
         * @param config[const pandora_vision_hazmat::DisplayConfig&] :
         *  The message containing the new configuration for the node.
         * @param level[uint32_t]: Flag used by the dynamic reconfigure
         * server.
         *
         */
        void dynamicReconfigCallback(
            const ::pandora_vision_hazmat::DisplayConfig& config,
            uint32_t level);

        /**
         * @brief
         */
        virtual bool process(const CVMatStampedConstPtr& input,
          const POIsStampedPtr& output);

        private:
          dynamic_reconfigure::Server< ::pandora_vision_hazmat::DisplayConfig>
          /// Reconfigure server for changing object params
          dynamicReconfServer_;

          dynamic_reconfigure::Server< ::pandora_vision_hazmat::DisplayConfig>::
            CallbackType f_detector_;

          bool visualizationFlag_;
          /// Flag that toggles the execution time printing.
          bool execTimerFlag_;

          bool debugMsgFlag_;  //!< Flag that toggles debug messages that contain
          //!< for the detected patterns.

          DetectorFactory factory_;  //!< The factory that produces the detectors.

          PlanarObjectDetector* detector_;  //!< Pointer to the detector used.
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_DETECTION_HAZMAT_PROCESSOR_H
