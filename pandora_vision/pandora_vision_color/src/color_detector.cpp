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
* Author:  Marios Protopapas, <protopapas_marios@hotmail.com>
*********************************************************************/

#include <vector>
#include "pandora_vision_color/color_detector.h"

namespace pandora_vision
{
namespace pandora_vision_color
{
  /**
    @brief Class Constructor
    Initializes all varialbes for thresholding
  */

  ColorDetector::ColorDetector(void)
  {
    bounding_box_.reset( new BBoxPOI() );
    bounding_box_->setPoint(cv::Point(0, 0));
    bounding_box_->setWidth(0);
    bounding_box_->setHeight(0);
    bounding_box_->setProbability(0.0f);
    visualization_ = false;
    iLowH = 120;
    iHighH = 180;
    iLowS = 82;
    iHighS = 232;
    iLowV = 0;
    iHighV = 194;
    ROS_INFO_STREAM("Created ColorDetector instance" << visualization_);
  }

  
  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  ColorDetector::~ColorDetector()
  {
    ROS_INFO("Destroying ColorDetector instance");
  }
  
  BBoxPOIPtr ColorDetector::getColorPosition(void)
  {
    return bounding_box_;
  }

   /**
    @brief Function that detects color,
    @param frame [&cv::Mat] current frame to be processed
    @return void.
  */
  void ColorDetector::detectColor(const cv::Mat& frame)
  {
    frame_ = frame.clone();
    /// Check that frame has data and that image has 3 channels
    if (frame_.data && frame_.channels() == 3)
    {
      /// blur the image using GaussianBlur
      GaussianBlur(frame_, frame_, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

      /// convert RGB image into HSV image
      cvtColor(frame, hsvFrame_, CV_BGR2HSV);

      /// get binary image
      inRange(hsvFrame_, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), binary_);  // pink*/
      //  inRange(hsvFrame_, cv::Scalar(110,50,50), cv::Scalar(130,255,255), binary_);  // blue

      /// morphological opening (remove small objects from the foreground)
      erode(binary_, binary_, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      dilate(binary_, binary_, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

      /// morphological closing (fill small holes in the foreground)
      dilate(binary_, binary_, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      erode(binary_, binary_, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));


      detectColorPosition();
      if (visualization_)
        debugShow();
    }
  }

  /**
    @brief Function that calculates motion's position
    @param diff: [&cv::Mat] frame that represents
      the thresholded difference between current frame and computed
      background.
    @return void
  */
  void ColorDetector::detectColorPosition()
  {
    std::vector< std::vector<cv::Point> > contours;
    findContours(binary_, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // find contours
    std::vector<double> areas(contours.size());
    /// find largest contour area

    for (int ii = 0; ii < contours.size(); ii++)
    {
      areas[ii] = contourArea(cv::Mat(contours[ii]));
    }

    /// get index of largest contour
    double max;
    cv::Point maxPosition;
    minMaxLoc(cv::Mat(areas), 0, &max, 0, &maxPosition);

    /// draw largest contour.
    drawContours(binary_, contours, maxPosition.y, cv::Scalar(255), CV_FILLED);

    /// draw bounding rectangle around largest contour
    cv::Point center;
    cv::Rect r;
    if (contours.size() >= 1)
    {
      r = boundingRect(contours[maxPosition.y]);
      cv::rectangle(frame_, r.tl(), r.br(), CV_RGB(255, 0, 0), 3, 8, 0);  // draw rectangle
       bounding_box_->setPoint(cv::Point(r.x, r.y));
      bounding_box_->setWidth(r.width);
      bounding_box_->setHeight(r.height);
      bounding_box_->setProbability(1.0f);
    }
  }

  /**
    @brief Function used for debug reasons
    @return void
  */
  void ColorDetector::debugShow()
  {
    if (visualization_)
    {
      /* cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); */
      // cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
      // cv::createTrackbar("HighH", "Control", &iHighH, 179);

      // cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
      // cv::createTrackbar("HighS", "Control", &iHighS, 255);

      // cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
      /* cv::createTrackbar("HighV", "Control", &iHighV, 255); */
      cv::Mat temp1[] = {binary_, binary_, binary_};
      cv::Mat diff;
      cv::merge(temp1, 3, diff);
      cv::Mat displayImage = cv::Mat(frame_.rows, frame_.cols * 2, frame_.type() );
      frame_.copyTo(displayImage(cv::Rect(0, 0, frame_.cols, frame_.rows)));
      diff.copyTo(displayImage(cv::Rect(diff.cols, 0, diff.cols, diff.rows)));
      imshow("ColorDetection", displayImage);
      cv::waitKey(10);
    }
  }
}  // namespace pandora_vision_color
}  // namespace pandora_vision
