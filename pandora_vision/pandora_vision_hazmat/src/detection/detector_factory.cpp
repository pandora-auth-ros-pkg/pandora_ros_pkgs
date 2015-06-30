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

#include <string>

#include "pandora_vision_hazmat/detection/detector_factory.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @brief : The main factory method that creates the different feature
     * detectors
     * @param featureType(const std::string&): The name of the feature that
     * will be used.
     */
    PlanarObjectDetector* DetectorFactory::createDetectorObject(
        const std::string& featureType)
    {
      std::locale loc;
      std::string feature;
      // Convert the input string to upper case format.
      for (std::string::size_type i = 0; i < featureType.length(); ++i)
        feature += std::toupper(featureType[i], loc);

      if (!feature.compare("SIFT"))
      {
        ROS_INFO_STREAM("[Hazmat Detection]: Creating new SIFT Detector!");
        return new SiftDetector();
      }
      else if (!feature.compare("SURF"))
      {
        ROS_INFO_STREAM("[Hazmat Detection]: Creating new SURF Detector!");
        return new SurfDetector();
      }
      else if (!featureType.compare("ORB"))
      {
        ROS_INFO_STREAM("[Hazmat Detection]: Creating new ORB Detector!");
        return new OrbDetector();
      }
      else
      {
        ROS_FATAL_STREAM("[Hazmat Detection]: Invalid feature type!"
            << " Detection cannot continue!");
        return NULL;
      }
      return NULL;
    }

}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
