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
 * Authors: Vasilis Bosdelekidis, Alexandros Philotheou
 *********************************************************************/

#ifndef DEPTH_NODE_DEPTH_PROCESSOR_H
#define DEPTH_NODE_DEPTH_PROCESSOR_H

#include "pandora_vision_common/pandora_vision_interface/vision_processor.h"
#include "pandora_vision_common/bbox_poi.h"
#include "utils/message_conversions.h"
#include "utils/holes_conveyor.h"
#include "utils/depth_parameters.h"
#include "utils/visualization.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class DepthProcessor
    @brief Provides functionalities for locating holes via
    analysis of a Depth image
   **/
  class DepthProcessor : public VisionProcessor
  {
    public:
      // The constructors
      DepthProcessor(
          const std::string& ns, 
          sensor_processor::Handler* handler);
      
      DepthProcessor();
      
      // The destructor
      virtual ~DepthProcessor();

      /**
        @brief Finds holes, provided a depth image in CV_32FC1 format.

        1. some basic eroding and dilation to eliminate isolated noise pixels.
        2. eliminate huge contours and tiny contours without filtering. There is no problem with the huge and no problem with the tiny for distances less than 2.5m (in any case, in such distances holes are not obvious at all).
        3. Eliminate contours which have an over 4x bounding box height/ width fraction or the inverse. There was no TP loss by this progress, instead wall edges were eliminated.
        4. Merge contours. Check only in small distance. Firstly label contours to merge together with a probability. The probability consists of a weighted sum of some features inside a box between the two contours' keypoints. Currently, features are the inverse of the euclidean distance between keypoints and the sum of blacks in the original image. 
        5. The contours checked at each step are those that were not eliminated by previous steps.
        @param[in] depthImage [const cv::Mat&] The depth image in CV_32FC1 format
        @return HolesConveyor The struct that contains the holes found
       **/
      HolesConveyor findHoles(const cv::Mat& depthImage);

      /**
        @brief The function called to filter the depth image
        @details Based on filtering_type param applying simple thresholding at 0, simple morhology transformations and to eliminate (make dark) the region at borders or simple edge detection method.
        @param[in] depthImage [const cv::Mat&] The Depth image to be processed,
        in CV_8UC3 format
        @param[in] filteredImage [cv::Mat* filteredImage] The output filtered binary image.
        @return void
       **/
      void filterImage(
          const cv::Mat& depthImage, 
          cv::Mat* filteredImage);


      /**
        @brief The function called to extract contours that were not ignored by the initial filtering held in filterImage function.
        @param[in] filteredImage [cv::Mat&] The filtered image represented as white edges where there was a small depth. Remember that there were only edges left, which were black at the original depth image and were not random black pixels.
        @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
        @return void
       **/
      void detectContours(
          const cv::Mat& filteredImage, 
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
        @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
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
          std::vector<cv::Point2f>* mc, 
          std::vector<int>* contourHeight, 
          std::vector<int>* contourWidth, 
          std::vector<bool>* realContours, 
          const std::vector<cv::Rect>& boundRect);

      /**
        @brief The function called by validateContours to make validation of a single contour
        @param[in] image [const cv::Mat&] The depth image 
        @param[in] ci [int] Current contour index in contours vector 
        @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
        @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
        @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
        @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. 
        @return void
       **/
      bool validateContour(
          const cv::Mat& image, 
          int ci, std::vector<cv::Point2f>* mcv, 
          std::vector<int>* contourHeight, 
          std::vector<int>* contourWidth, 
          std::map<std::pair<int, int>, float>* contourLabel, 
          std::vector<int>* numLabels, 
          const std::vector<cv::Rect>& boundRect, 
          const std::vector<std::vector<cv::Point> >& contours, 
          std::vector<bool>* realContours);

      void validateShape(
          const cv::Mat& image, 
          const std::vector<std::vector<cv::Point> >& outline, 
          const std::vector<cv::Rect>& boundRect, 
          int ci, 
          std::vector<bool>* realContours);
      /**
        @brief The function called by validateContours to do the final merging after the probabilities for each pair were found in validateContour. New contourwidths and heights are calculated for merged contours. New coordinates for the merged contour as the average of all the contours that consist it are calculated.
        @param[in] ci [int] Current contour index in contours vector 
        @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
        @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
        @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
        @return void
       **/
      void mergeContours(
          int ci, 
          std::vector<cv::Point2f>* mcv, 
          std::vector<int>* contourHeight, 
          std::vector<int>* contourWidth, 
          std::map<std::pair<int, int>, 
          float>* contourLabel, 
          std::vector<int>* numLabels, 
          std::vector<bool>* realContours, 
          const std::vector<std::vector<cv::Point> >& contours);
    
      virtual bool process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output);

    private:
      DepthParametersHandler* DepthParametersHandler_;

  };

}  // namespace pandora_vision

#endif  // DEPTH_NODE_DEPTH_PROCESSOR_H
