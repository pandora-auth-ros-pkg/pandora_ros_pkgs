/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,Angelos Triantafyllidis
 *********************************************************************/

#ifndef THERMAL_NODE_HOLE_DETECTOR_H
#define THERMAL_NODE_HOLE_DETECTOR_H

#include "utils/blob_detection.h"
#include "utils/holes_conveyor.h"
#include "utils/hole_filters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleDetector
    @brief Provides the functionalities for detecting holes via analysis
    of a thermal image
   **/
  class HoleDetector
  {
    public:

      /**
        @brief Finds the holes provided a thermal image in CV_8UC1 format

        First, the edges of the thermal image are detected.
        Then, keypoints of blobs are detected in the above image.
        Finally, the potential holes' outline is found, along with the bounding
        boxes of those outlines.
        @param[in] thermalImage [const cv::Mat&] The thermal image in CV_8UC1 format
        @return HolesConveyor The struct that contains the holes found
       **/
      static HolesConveyor findHoles(const cv::Mat& thermalImage);

  };

} // namespace pandora_vision

#endif  // THERMAL_NODE_HOLE_DETECTOR_H
