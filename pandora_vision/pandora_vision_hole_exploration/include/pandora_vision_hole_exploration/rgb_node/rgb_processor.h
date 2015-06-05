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
 * Authors: Vasilis Bosdelekidis, Despoina Paschalidou, Alexandros Philotheou 
 *********************************************************************/

#ifndef RGB_NODE_RGB_PROCESSOR_H
#define RGB_NODE_RGB_PROCESSOR_H

#include "pandora_vision_common/bbox_poi.h"
#include "pandora_vision_common/pandora_vision_interface/vision_processor.h"
#include "utils/message_conversions.h"
#include "utils/holes_conveyor.h"
#include "utils/haralickfeature_extractor.h"
#include "utils/rgb_parameters.h"
#include "utils/visualization.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class RgbProcessor
    @brief Provides functionalities for locating holes via
    analysis of a RGB image
   **/
  class RgbProcessor : public VisionProcessor
  {
    public:
      // The constructors
      RgbProcessor(const std::string& ns, sensor_processor::Handler* handler);
      RgbProcessor();

      // The destructor
      virtual ~RgbProcessor();

      /**
        @brief The function called to extract holes from RGB image
        @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
        in CV_8UC3 format
        @return [HolesConveyor] A struct with useful info about each hole.
       **/
      HolesConveyor findHoles(const cv::Mat& rgbImage);

      /**
        @brief The function called to calculate the variance in the RGB image, at overlapping windows of specific size.
        @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
        in CV_8UC3 format
        @param[in] bigVarianceContours [cv::Mat* bigVarianceContours] The output binary image thresholded based on the variance of the RGB image.
        @return void
       **/
      void computeVarianceImage(
          const cv::Mat& rgbImage, 
          cv::Mat* bigVarianceContours);


      /**
        @brief The function called to extract contours that were not ignored by the initial filtering held in computeVarianceImage function.
        @param[in] bigVarianceContours [cv::Mat& bigVarianceContours] The variance image represented as edges. Remember that there were only edges left, which represent big variance.
        @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
        @return void
       **/
      void detectContours(
          const cv::Mat& bigVarianceContours, 
          std::vector<std::vector<cv::Point> >* contours);


      /**
        @brief The function called to estimate center of mass for each contour found and bounding boxes.
        @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
        @param[in] mc [std::vector<cv::Point2f>*] Center of mass of each contour as x, y coordinates..
        @return void
       **/
      void getContourInfo(
          const std::vector<std::vector<cv::Point> >& contours, 
          std::vector<cv::Point2f>* mc, 
          std::vector<cv::Rect>* boundRect);

      /**
        @brief The function called to make validation of found contours
        @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour similarity etc.
        @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
        @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
        @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
        @return void
       **/
      void validateContours(
          const cv::Mat& image, 
          const std::vector<std::vector<cv::Point> >& contours, 
          std::vector<cv::Point2f>* mc, std::vector<int>* contourHeight, 
          std::vector<int>* contourWidth, 
          std::vector<bool>* realContours, 
          const std::vector<cv::Rect>& boundRect);


      void mergeContours(
          std::vector<std::vector<int> >* mergables,
          std::vector<int>* contourWidth,
          std::vector<int>* contourHeight,
          std::vector<cv::Point2f>* mc, 
          const std::vector<cv::Rect>& boundRect,
          std::vector<bool>* realContours);


      void validateShape(
          const cv::Mat& image, 
          const std::vector<std::vector<cv::Point> >& outline, 
          const std::vector<cv::Rect>& boundRect, 
          int ci, 
          std::vector<bool>* realContours);

      virtual bool process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output);

    private:
      RgbParametersHandler* RgbParametersHandler_;

  };

}  // namespace pandora_vision

#endif  // RGB_NODE_RGB_PROCESSOR_H
