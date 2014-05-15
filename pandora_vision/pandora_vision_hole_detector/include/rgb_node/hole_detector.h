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
 * Author: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#ifndef RGB_NODE_HOLE_DETECTOR_H
#define RGB_NODE_HOLE_DETECTOR_H

#define SHOW_DEBUG_IMAGE

#include "utils/hole_filters.h"
#include "utils/histogram.h"

namespace pandora_vision
{
  class HoleDetector
  {
    private:

      // Calculated histogramm according to given images
      cv::MatND histogram_;


    public:

      /**
        @brief Class constructor
       **/
      HoleDetector();

      /**
        @brief Class destructor
       **/
      ~HoleDetector();

      /**
        @brief Function that locates the position of potentional holes
        in the current frame.
        @param holeFrame [const cv::Mat&] current frame to be processed
        @return void
       **/
      HolesConveyor findHoles(const cv::Mat& holeFrame);

      /**
        @brief Fills an image with random colours per image segment
        @param[in,out] image [cv::Mat*] The image to be processed
        @param[in] colorDiff [const cv::Scalar&] Maximal upper and lower
        brightness/color difference between the currently observed pixel
        and one of its neighbors belonging to the component.
        (see http://docs.opencv.org/modules/imgproc/doc/
        miscellaneous_transformations.html#floodfill)
        @return void
       **/
      static void floodFillPostprocess(cv::Mat* image,
        const cv::Scalar& colorDiff = cv::Scalar::all(1));

      /**
        @brief Posterizes a RGB image via segmentation
        @param[in] inImage [const cv::Mat&] The RGB image to be posterized
        @param[out] outImage [cv::Mat*] The posterized image
        @return void
       **/
      static void segmentation(const cv::Mat& inImage,
        cv::Mat* outImage);

  };

} // namespace pandora_vision

#endif  // RGB_NODE_HOLE_DETECTOR_H
