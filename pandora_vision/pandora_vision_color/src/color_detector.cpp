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
    visualization_ = false;
    minArea_ = 2500;
    iLowH = 110;
    iHighH = 130;
    iLowS = 180;
    iHighS = 255;
    iLowV = 60;
    iHighV = 100;
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
  
  std::vector<POIPtr> ColorDetector::getColorPosition(void)
  {
    return bounding_boxes_;
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
    bounding_boxes_.clear();
    findContours(binary_.clone(), contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // find contours
    std::vector<double> areas(contours.size());
    /// find largest contour area

    for (int ii = 0; ii < contours.size(); ii++)
    {
      areas[ii] = contourArea(cv::Mat(contours[ii]));
    }

    /// get index of largest contour
   /*  double max; */
    // cv::Point maxPosition;
    /* minMaxLoc(cv::Mat(areas), 0, &max, 0, &maxPosition); */

    /// draw largest contour.
    drawContours(binary_, contours, -1, cv::Scalar(255), CV_FILLED);
    cv::Point center;
    cv::Rect r;
    // ROS_INFO_STREAM("AREAS=");
    /// draw bounding rectangle around largest contour
    for (int i = 0; i < contours.size(); i++)
    {
      if (areas[i] > minArea_)
      {
        // ROS_INFO_STREAM("CONTOUR NO "<< i << " area="<<areas[i]);
        ObstaclePOIPtr bounding_box(new ObstaclePOI);
        r = cv::boundingRect(contours[i]);
        cv::rectangle(frame_, r.tl(), r.br(), CV_RGB(255, 0, 0), 3);  // draw rectangle
        bounding_box->setPoint(cv::Point(r.x + r.width / 2, r.y + r.height/2));
       /*  bounding_box->setWidth(r.width); */
        /* bounding_box->setHeight(r.height); */
        bounding_box->setProbability(1.0f);
        bounding_box->setType(pandora_vision_msgs::ObstacleAlert::BARREL);
        bounding_box->setDepth(0);
        bounding_boxes_.push_back(bounding_box);
      }
    }
  }

  /**
    @brief Function used for debug reasons
    @return void
  */
  void   ColorDetector::debugShow()
  {
    if (visualization_)
    {
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
