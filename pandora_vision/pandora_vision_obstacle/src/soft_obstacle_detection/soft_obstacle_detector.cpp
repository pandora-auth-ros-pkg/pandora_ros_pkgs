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

#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include "pandora_vision_msgs/ObstacleAlert.h"
#include "pandora_vision_obstacle/soft_obstacle_detection/soft_obstacle_detector.h"

namespace pandora_vision
{
  SoftObstacleDetector::SoftObstacleDetector(const std::string& name,
      const ros::NodeHandle& nh)
  {
    nodeName_ = name;

    nh.param("hsvColor/sThreshold", sValueThreshold_, 95);
    nh.param("hsvColor/vThreshold", vValueThreshold_, 130);

    nh.param("gaussianKernelSize", gaussianKernelSize_, 13);

    int rows, cols;
    nh.param("erodeKernelSize/rows", rows, 3);
    nh.param("erodeKernelSize/cols", cols, 3);
    erodeKernelSize_ = cv::Size(rows, cols);

    nh.param("dilateKernelSize/rows", rows, 3);
    nh.param("dilateKernelSize/cols", cols, 3);
    dilateKernelSize_ = cv::Size(rows, cols);

    double param;
    nh.param("gradientThreshold", param, 2.0);
    gradientThreshold_ = param;
    nh.param("betaThreshold", param, 3.0);
    betaThreshold_ = param;

    nh.param("depthThreshold", depthThreshold_, 0.3);

    showOriginalImage_ = false;
    showDWTImage_ = false;
    showOtsuImage_ = false;
    showDilatedImage_ = false;
    showVerticalLines_ = false;
    showROI_ = false;

    float invRootTwo = 1.0f / static_cast<float>(std::sqrt(2));
    cv::Mat kernelLow = (cv::Mat_<float>(2, 1) << invRootTwo, invRootTwo);
    cv::Mat kernelHigh = (cv::Mat_<float>(2, 1) << invRootTwo, - invRootTwo);

    dwtPtr_.reset(new DiscreteWaveletTransform(kernelLow, kernelHigh));
  }

  SoftObstacleDetector::SoftObstacleDetector() {}

  void SoftObstacleDetector::setShowOriginalImage(bool arg)
  {
    showOriginalImage_ = arg;
  }

  void SoftObstacleDetector::setShowDWTImage(bool arg)
  {
    showDWTImage_ = arg;
  }

  void SoftObstacleDetector::setShowOtsuImage(bool arg)
  {
    showOtsuImage_ = arg;
  }

  void SoftObstacleDetector::setShowDilatedImage(bool arg)
  {
    showDilatedImage_ = arg;
  }

  void SoftObstacleDetector::setShowVerticalLines(bool arg)
  {
    showVerticalLines_ = arg;
  }

  void SoftObstacleDetector::setShowROI(bool arg)
  {
    showROI_ = arg;
  }

  void SoftObstacleDetector::setGaussianKernel(int size)
  {
    gaussianKernelSize_ = size;
  }

  void SoftObstacleDetector::dilateImage(const MatPtr& image)
  {
    int nonZero = cv::countNonZero(*image);
    if (nonZero > image->rows * image->cols / 2)
    {
      *image = 255 - *image;
    }

    // First erode image to eliminate noise and then dilate
    cv::Mat erodedImage;
    cv::Mat erodeKernel = cv::getStructuringElement(cv::MORPH_RECT, erodeKernelSize_);
    cv::erode(*image, erodedImage, erodeKernel);

    cv::Mat dilatedImage;
    cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_RECT, dilateKernelSize_);
    cv::dilate(erodedImage, dilatedImage, dilateKernel);
    *image = dilatedImage;
  }

  bool SoftObstacleDetector::findNonIdenticalLines(const std::vector<cv::Vec2f> lineCoeffs,
      float grad, float beta)
  {
    for (size_t ii = 0; ii < lineCoeffs.size(); ii++)
    {
      if (fabs(fabs(grad) - fabs(lineCoeffs[ii][0])) < gradientThreshold_
          && fabs(fabs(beta) - fabs(lineCoeffs[ii][1])) < betaThreshold_)
      {
        return false;
      }
    }
    return true;
  }

  bool SoftObstacleDetector::detectLineIntersection(const std::vector<cv::Vec4i>& verticalLines,
      const cv::Vec4i& line)
  {
    for (size_t ii = 0; ii < verticalLines.size(); ii++)
    {
      cv::Point startPoint(verticalLines[ii][0], verticalLines[ii][1]);
      cv::Point endPoint(verticalLines[ii][2], verticalLines[ii][3]);

      cv::Point verticalLineDiff = endPoint - startPoint;
      cv::Point currentLineDiff(line[2] - line[0], line[3] - line[1]);

      double det0 = verticalLineDiff.x * currentLineDiff.y
        - verticalLineDiff.y * currentLineDiff.x;

      if (fabs(det0) > 1e-6)
      {
        return true;
      }
    }
    return false;
  }

  bool SoftObstacleDetector::pickLineColor(const cv::Mat& hsvImage, const cv::Vec4i& line,
      int level)
  {
    cv::Point center((line[0] + line[2]) / 2 * pow(2, level),
        (line[1] + line[3]) / 2 * pow(2, level));

    cv::Vec3b hsvValue = hsvImage.at<cv::Vec3b>(center.y, center.x);

    if (hsvValue[1] < sValueThreshold_ && hsvValue[2] > vValueThreshold_)
    {
      return true;
    }
    return false;
  }

  std::vector<cv::Vec4i> SoftObstacleDetector::performProbHoughLines(const cv::Mat& rgbImage,
      const cv::Mat& binaryImage, int level)
  {
    /// Perform Hough Transform
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 100, 100, 10);

    cv::Mat imageToShow;
    cv::cvtColor(binaryImage, imageToShow, CV_GRAY2BGR);

    cv::Mat hsvImage;
    cv::cvtColor(rgbImage, hsvImage, CV_BGR2HSV);

    std::vector<cv::Vec4i> verticalLines;
    std::vector<cv::Vec2f> lineCoefficients;

    /// Keep only vertical lines
    for (size_t ii = 0; ii < lines.size(); ii++)
    {
      cv::Vec4i line = lines[ii];
      bool awayFromBorder = (line[0] > 10 && line[0] < binaryImage.cols - 10) ||
        (line[2] > 10 && line[2] < binaryImage.cols - 10);

      float grad, beta;
      if (line[0] == line[2])
      {
        grad = 90.0f;
        beta = line[0];
      }
      else
      {
        grad = static_cast<float>(line[3] - line[1]) / static_cast<float>(
            line[2] - line[0]);

        beta = line[0] - line[1] / grad;  //!< The point that the line intersects with the x-axis
        grad = static_cast<float>(fabs(atan(grad) * 180.0f / CV_PI));
      }

      /// If line is almost vertical and not close to image borders
      if ((grad > 80.0f && grad < 100.0f) && awayFromBorder)
      {
        if (findNonIdenticalLines(lineCoefficients, grad, beta)
            // && !detectLineIntersection(verticalLines, line)
            && pickLineColor(hsvImage, line, level))
        {
          lineCoefficients.push_back(cv::Vec2f(grad, beta));

          verticalLines.push_back(line);
          cv::line(imageToShow, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
              cv::Scalar(255, 0, 0), 3, 8);
        }
      }
    }
    if (showVerticalLines_)
    {
      cv::imshow("[" + nodeName_ + "] : Vertical Lines Detected", imageToShow);
      cv::waitKey(10);
    }
    return verticalLines;
  }

float SoftObstacleDetector::detectROI(const std::vector<cv::Vec4i>& verticalLines,
      int frameHeight, const boost::shared_ptr<cv::Rect>& roiPtr)
  {
    float probability = 0.0f;

    int minx = std::numeric_limits<int>::max();
    int maxx = std::numeric_limits<int>::min();
    int miny = std::numeric_limits<int>::max();
    int maxy = std::numeric_limits<int>::min();

    for (size_t ii = 0; ii < verticalLines.size(); ii++)
    {
      /// Calculate min and max of line coordinates
      int x0 = verticalLines[ii][0];
      int x1 = verticalLines[ii][2];
      int y0 = verticalLines[ii][1];
      int y1 = verticalLines[ii][3];

      minx = x0 < minx ? x0 : minx;
      minx = x1 < minx ? x1 : minx;

      maxx = x0 > maxx ? x0 : maxx;
      maxx = x1 > maxx ? x1 : maxx;

      miny = y0 < miny ? y0 : miny;
      miny = y1 < miny ? y1 : miny;

      maxy = y0 > maxy ? y0 : maxy;
      maxy = y1 > maxy ? y1 : maxy;

      /// Calculate ROI probability
      probability += static_cast<float>(abs(y1 - y0)) / static_cast<float>(frameHeight);
    }
    probability /= static_cast<float>(verticalLines.size());

    int width = maxx - minx;
    int height = maxy - miny;

    // The point inside this Rect is the roi center, now it is the
    // upper left point in order to visualize
    cv::Rect roi(minx, miny, width, height);
    *roiPtr = roi;

    return probability;
  }

  std::vector<float> SoftObstacleDetector::findDepthDistance(const cv::Mat& depthImage,
      const std::vector<cv::Vec4i> verticalLines, const cv::Rect& roi, int level)
  {
    std::vector<float> depth;

    int minLinePosition = 0;
    int maxLinePosition = 0;

    for (size_t ii = 1; ii < verticalLines.size(); ii++)
    {
      int x0 = verticalLines[ii][0];
      int x1 = verticalLines[ii][2];

      minLinePosition = (x0 == roi.x || x1 == roi.x) ? ii : minLinePosition;
      maxLinePosition = (x0 == roi.x + roi.width || x1 == roi.x + roi.width)
        ? ii : maxLinePosition;
    }

    int xx = (roi.x + roi.width / 2) * pow(2, level);
    int yy = roi.y * pow(2, level);
    depth.push_back(depthImage.at<float>(yy, xx));

    int lineCenterX = (verticalLines[maxLinePosition][0]
        + verticalLines[maxLinePosition][2]) / 2;
    int lineCenterY = (verticalLines[maxLinePosition][1]
        + verticalLines[maxLinePosition][3]) / 2;

    xx = lineCenterX * pow(2, level);
    yy = lineCenterY * pow(2, level);
    depth.push_back(depthImage.at<float>(yy, xx));

    xx = (roi.x + roi.width / 2) * pow(2, level);
    yy = (roi.y + roi.height) * pow(2, level);
    depth.push_back(depthImage.at<float>(yy, xx));

    lineCenterX = (verticalLines[minLinePosition][0]
        + verticalLines[minLinePosition][2]) / 2;
    lineCenterY = (verticalLines[minLinePosition][1]
        + verticalLines[minLinePosition][3]) / 2;

    xx = lineCenterX * pow(2, level);
    yy = lineCenterY * pow(2, level);
    depth.push_back(depthImage.at<float>(yy, xx));

    return depth;
  }

  bool SoftObstacleDetector::findDifferentROIDepth(const cv::Mat& depthImage,
      const cv::Rect& roi, float firstLineDepth, float lastLineDepth)
  {
    cv::Mat depthROI = depthImage(roi);
    cv::Scalar meanValue = cv::mean(depthROI);

    if (fabs(meanValue[0] - firstLineDepth) < depthThreshold_
        || fabs(meanValue[0] - lastLineDepth) < depthThreshold_)
    {
      return false;
    }
    return true;
  }

  std::vector<POIPtr> SoftObstacleDetector::detectSoftObstacle(const cv::Mat& rgbImage,
      const cv::Mat& depthImage, int level)
  {
    if (showOriginalImage_)
    {
      cv::imshow("[" + nodeName_ + "] : Original Image with Soft Obstacle Bounding Box",
          rgbImage);
      cv::waitKey(10);
    }

    // Convert rgb image to grayscale
    cv::Mat grayScaleImage;
    cv::cvtColor(rgbImage, grayScaleImage, CV_BGR2GRAY);

    cv::Mat imageFloat;
    grayScaleImage.convertTo(imageFloat, CV_32FC1);

    // Blur image using Gaussian filter
    cv::Mat blurImage;
    cv::GaussianBlur(imageFloat, blurImage, cv::Size(gaussianKernelSize_,
          gaussianKernelSize_), 0);

    // Perform DWT
    std::vector<MatPtr> LHImages = dwtPtr_->getLowHigh(blurImage, level);
    MatPtr lhImage(LHImages[LHImages.size() - 1]);

    // Normalize image [0, 255]
    cv::Mat normalizedImage;
    cv::normalize(*lhImage, normalizedImage, 0, 255, cv::NORM_MINMAX);
    normalizedImage.convertTo(normalizedImage, CV_8UC1);

    if (showDWTImage_)
    {
      cv::imshow("[" + nodeName_ + "] : DWT Normalized Image", normalizedImage);
      cv::waitKey(10);
    }

    // Convert image to binary with Otsu thresholding
    MatPtr otsuImage(new cv::Mat());
    cv::threshold(normalizedImage, *otsuImage, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    if (showOtsuImage_)
    {
      cv::imshow("[" + nodeName_ + "] : Image after Otsu Thresholding", *otsuImage);
      cv::waitKey(10);
    }

    // Dilate Image
    dilateImage(otsuImage);

    if (showDilatedImage_)
    {
      cv::imshow("[" + nodeName_ + "] : Dilated Image", *otsuImage);
      cv::waitKey(10);
    }

    // Perform Hough Transform to detect lines (keep only vertical)
    std::vector<cv::Vec4i> verticalLines = performProbHoughLines(rgbImage, *otsuImage);

    std::vector<POIPtr> pois;

    if (verticalLines.size() > 2)
    {
      ROS_INFO("Detected Vertical Lines");

      // Detect bounding box that includes the vertical lines
      boost::shared_ptr<cv::Rect> roi(new cv::Rect());
      float probability = detectROI(verticalLines, otsuImage->rows, roi);

      // Find the depth distance of the soft obstacle
      std::vector<float> depthDistance = findDepthDistance(depthImage, verticalLines,
          *roi, level);

      // Examine whether the points of the bounding box have difference in depth
      // distance
      bool differentDepth = findDifferentROIDepth(depthImage, *roi, depthDistance[1],
            depthDistance[3]);

      if (probability > 0.0f && differentDepth)
      {
        ROS_INFO("Soft Obstacle Detected!");

        if (showROI_)
        {
          cv::Mat imageToShow = rgbImage.clone();

          cv::Rect bbox(roi->x * pow(2, level), roi->y * pow(2, level),
              roi->width * pow(2, level), roi->height * pow(2, level));
          cv::rectangle(imageToShow, bbox, cv::Scalar(0, 255, 0), 4);

          cv::imshow("[" + nodeName_ + "] : Original Image with Soft Obstacle Bounding Box",
              imageToShow);
          cv::waitKey(10);
        }

        for (int ii = 0; ii < depthDistance.size(); ii++)
        {
          ObstaclePOIPtr poi(new ObstaclePOI);

          poi->setPoint(cv::Point(
                (roi->x + (1 - (ii == 3)) * roi->width / pow(2, !(ii % 2))) * pow(2, level),
                (roi->y + (1 - (ii == 0)) * roi->height / pow(2, ii % 2)) * pow(2, level)));

          poi->setProbability(probability);
          poi->setType(pandora_vision_msgs::ObstacleAlert::SOFT_OBSTACLE);

          poi->setDepth(depthDistance[ii]);
          pois.push_back(poi);
        }
      }
    }
    return pois;
  }
}  // namespace pandora_vision
