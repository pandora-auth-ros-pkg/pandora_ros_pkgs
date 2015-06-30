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

#ifndef PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_FACTORY_H
#define PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_FACTORY_H

#include <locale>
#include <string>

#include "pandora_vision_hazmat/detection/planar_object_detector.h"
#include "pandora_vision_hazmat/detection/sift_detector.h"
#include "pandora_vision_hazmat/detection/surf_detector.h"
#include "pandora_vision_hazmat/detection/orb_detector.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @class DetectorFactory
     * @brief The class that is used to produce the detectors.
     */
    class DetectorFactory
    {
      public:
        /**
         * @brief Default Constructor
         */
        DetectorFactory()
        {
        };

        /**
         * @brief : The main factory method that creates the different feature
         * detectors
         * @param featureType(const std::string&): The name of the feature
         * that will be used.
         */
        PlanarObjectDetector* createDetectorObject(
            const std::string& featureType);
      private:
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_FACTORY_H
