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

#ifndef PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_INTERFACE_H
#define PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_INTERFACE_H

#include <vector>

#include "pandora_vision_hazmat/detection/utilities.h"
#include "pandora_vision_hazmat/detection/hazmat_poi.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @class Detector
     * @brief Interface for the detector objects.
     */
    class Detector
    {
      public:
        /**
         * @brief : This function is used to detect the desired
         * objects in the current frame .
         * @param frame[const cv::Mat&] : The frame that will be processed.
         * @param detectedObjects[std::vector<Object>*] : A vector containing
         * all the objects detected in the scene.
         * @return bool : True if an object has been detected,false otherwise.
         */
        bool virtual detect(const cv::Mat &frame, std::vector<POIPtr>*
            detectedObjects) = 0;

        /**
         * @brief Function used to read the necessary training data for
         * the detector to function.
         * @return [bool] : A flag that tells us whether we succeeded in
         * reading the data.
         */
        bool virtual readData(void) = 0;

        /**
         * @brief : The default empty constructor.
         */
        Detector()
        {
        }

        /**
         * @brief : The default destructor.
         */
        virtual ~Detector()
        {
        };
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_DETECTION_DETECTOR_INTERFACE_H
