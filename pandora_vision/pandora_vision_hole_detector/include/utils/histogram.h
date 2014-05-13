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
 * Authors: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#ifndef UTILS_HISTOGRAM_H
#define UTILS_HISTOGRAM_H

#include <dirent.h>
#include <ros/package.h>
#include "utils/defines.h"
#include "utils/parameters.h"
#include "utils/visualization.h"

namespace pandora_vision
{
  /**
    @class Histogram
    @brief Provides methods pertinent to histograms
   **/
  class Histogram
  {
    public:

      /**
        @brief Function for calculating the backprojection of an image,
        based on the histogram @param modelHistogram
        @param[in] inImage [const cv::Mat&] Current frame to be processed
        @param[in] modelHistogram [const cv::MatND&] A model histogram
        based on which the @param backprojection is produced
        @param[out] backprojection [cv::Mat*] Backprojection of the current
        frame
        @param[in] secondaryChannel [const int&] Which channel to use, aside the
        hue one. 1 for the Saturation channel, 2 for the Value channel
        @return void
       **/
      static void getBackprojection(const cv::Mat& inImage,
        const cv::MatND& modelHistogram,
        cv::Mat* backprojection,
        const int& secondaryChannel);

      /**
        @brief Computes a cv::MatND histogram from images loaded in directory
        ${pandora_vision_hole_detector}/src/walls
        @param[out] The calculated histogram
        @param[in] secondaryChannel [const int&] Which channel to use, aside the
        hue one. 1 for the Saturation channel, 2 for the Value channel
        @return void
       **/
      static void getHistogram (
        cv::MatND* histogram,
        const int& secondaryChannel);

  };

} // namespace pandora_vision

#endif  // UTILS_HISTOGRAM_H
