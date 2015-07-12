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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H

#include <string>
#include <vector>
#include <boost/array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include "pandora_vision_common/pandora_vision_utilities/discrete_wavelet_transform.h"
#include "pandora_vision_obstacle/obstacle_poi.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class SoftObstacleDetector
  {
    public:
      /**
       * @brief Constructor that initializes dwt Haar kernels
       **/
      SoftObstacleDetector(const std::string& name, const ros::NodeHandle& nh);
      /**
       * @brief Constructor without NodeHandle used for testing
       **/
      SoftObstacleDetector();

      /**
       * @brief Destructor
       **/
      ~SoftObstacleDetector() {}

    public:
      /**
       * @brief Detect soft obstacle by performing a number of
       * operations to the rgb and depth image
       * @param rgbImage [const cv::Mat&] The input rgb image
       * @param depthImage [const cv::Mat&] The input depth image
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<POIPtr>] The Point Of Interest that
       * includes the soft obstacle
       **/
      std::vector<POIPtr> detectSoftObstacle(const cv::Mat& rgbImage,
          const cv::Mat& depthImage, int level = 1);

      void setGaussianKernel(int size);

      void setShowOriginalImage(bool arg);
      void setShowDWTImage(bool arg);
      void setShowOtsuImage(bool arg);
      void setShowDilatedImage(bool arg);
      void setShowVerticalLines(bool arg);
      void setShowROI(bool arg);

    private:
      /**
       * @brief Invert image if necessary and dilate it
       * @param image [const pandora_vision_common::MatPtr&] The image to be dilated
       **/
      void dilateImage(const pandora_vision_common::MatPtr& image);

      /**
       * @brief Find out if a line is almost the same with another
       * that is already added to the vector of vertical lines
       * @param lineCoeffs [const std::vector<cv::Vec2f>&] The
       * vector containing the grad and beta of each vertical line
       * @param grad [float] The grad of the current line
       * @param beta [float] The beta parameter of the current line
       * @return [bool] Whether the current vertical line is not very
       * close to another and so it should be added to the vector
       **/
      bool findNonIdenticalLines(const std::vector<cv::Vec2f> lineCoeffs,
          float grad, float beta);

      /**
       * @brief Find out if a line intersects with another
       * that is already added to the vector of vertical lines
       * @param verticalLines [const std::vector<cv::Vec4i>&] The
       * input vector that contains the vertical lines found so far
       * @param line [cv::Vec4i&] The current line
       * @return [bool] Whether the current line intersects with
       * another detected line
       **/
      bool detectLineIntersection(const std::vector<cv::Vec4i>& verticalLines,
          const cv::Vec4i& line);

      /**
       * @brief Find the line's color so that it can be included or
       * excluded from the list of vertical lines
       * @param hsvImage [const cv::Mat&] The image used to find each
       * line's color
       * @param line [cv::Vec4i&] The current line
       * @param level [int] The number of stages of the DWT
       * @return [bool] Whether the current line has the desired color.
       * In this case it is close to white
       **/
      bool pickLineColor(const cv::Mat& hsvImage, const cv::Vec4i& line,
          int level = 1);

      /**
       * @brief Perform Probabilistic Hough Lines Transform and
       * keep only vertical lines
       * @param rgbImage [const cv::Mat&] The image used to find each
       * line's color
       * @param binaryImage [const cv::Mat&] The image that the transform
       * is applied to
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<cv::Vec4i>] The vector containing each
       * vertical line's start and end point
       **/
      std::vector<cv::Vec4i> performProbHoughLines(const cv::Mat& rgbImage,
          const cv::Mat& binaryImage, int level = 1);

      /**
       * @brief Create the bounding box that includes the soft obstacle
       * @param verticalLines [const std::vector<cv::Vec4i>&] The
       * input vector that contains the vertical lines found
       * @param frameHeight [int] The image height
       * @param roiPtr [const boost::shared_ptr<cv::Rect>&] The
       * bounding box that is returned
       * @return [float] The probability of the soft obstacle
       * considering the line's length
       **/
      float detectROI(const std::vector<cv::Vec4i>& verticalLines,
          int frameHeight, const boost::shared_ptr<cv::Rect>& roiPtr);

      /**
       * @brief Calculate the median value around the center of a line
       * @param resizedDepthImage [const cv::Mat&] The input depth image
       * values that are used to determine the median
       * @param line [const cv::Vec4i&] The coordinates of the line
       * around which the median is calculated
       * @param level [int] The number of stages of the DWT
       * @return [float] The median value that is found for the line
       **/
      float calculateLineMedian(const cv::Mat& depthImage,
          const cv::Vec4i& line, int level = 1);

      /**
       * @brief Find depth distance of soft obstacle bounding box edges
       * @param depthImage [const cv::Mat&] The input depth image
       * @param verticalLines [const std::vector<cv::Vec4i>&] The
       * input vector that contains the vertical lines found
       * @param roi [const cv::Rect&] The bounding box that is
       * used to find the depth distance
       * @param level [int] The number of stages of the DWT
       * @return [boost::array<float, 4>] The depth distance of each
       * point of the bounding box
       **/
      boost::array<float, 4> findDepthDistance(const cv::Mat& depthImage,
          const std::vector<cv::Vec4i> verticalLines, const cv::Rect& roi,
          int level = 1);

      /**
       * @brief Calculate the average depth distance of bounding
       * box and check whether it is different from the depth
       * distance of vertical lines' points in the bounding box
       * @param depthImage [const cv::Mat&] The input depth image
       * @param verticalLines [const std::vector<cv::Vec4i>&] The
       * input vector that contains the vertical lines found
       * @param roi [const cv::Rect&] The bounding box that is
       * used to find the depth distance
       * @param level [int] The number of stages of the DWT
       * @return [bool] Whether all the points of the bounding box
       * have different depth distance from the average depth distance of
       * vertical lines in the bounding box
       **/
      bool findDifferentROIDepth(const cv::Mat& depthImage,
          const std::vector<cv::Vec4i>& verticalLines, const cv::Rect& roi,
          int level = 1);

    private:
      /// The DWT class object used to perform this operation
      boost::shared_ptr<pandora_vision_common::DiscreteWaveletTransform> dwtPtr_;

      /// The node's name
      std::string nodeName_;

      /// The minimum length that a line should have to be detected by Hough
      /// transform
      int minLineLength_;

      /// The saturation threshold of HSV color used to pick the color of a line
      int sValueThreshold_;
      /// The value threshold of HSV color used to pick the color of a line
      int vValueThreshold_;

      /// The size of the kernel of the Gaussian filter used to blur image
      int gaussianKernelSize_;

      /// The maximum gradient difference that two lines should have to be
      /// considered almost identical
      float gradientThreshold_;
      /// The maximum beta difference that two lines should have to be
      /// considered almost identical
      float betaThreshold_;

      /// The minimum depth difference for a line to be considered valid
      double minDepthThreshold_;
      /// The maximum depth difference for a bounding box to be valid
      double maxDepthThreshold_;

      /// The size of the kernel used to erode the image
      cv::Size erodeKernelSize_;
      /// The size of the kernel used to dilate the image
      cv::Size dilateKernelSize_;

      /// The width of the rectangle in which the depth is calculated around the
      /// line center
      int centerWidth_;
      /// The height of the rectangle in which the depth is calculated around the
      /// line center
      int centerHeight_;

      /// The minimum number of lines for a soft obstacle to be detected
      int linesThreshold_;

      /// Debug parameters
      bool showOriginalImage_;
      bool showDWTImage_;
      bool showOtsuImage_;
      bool showDilatedImage_;
      bool showVerticalLines_;
      bool showROI_;

      friend class SoftObstacleDetectorTest;
  };
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H
