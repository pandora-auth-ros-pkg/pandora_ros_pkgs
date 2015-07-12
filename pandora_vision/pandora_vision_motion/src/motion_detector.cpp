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
 * Authors:
 *   Despoina Paschalidou
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *   Protopapas Marios <protopapas_marios@hotmail.com>
 *********************************************************************/

#include <vector>
#include "pandora_vision_motion/motion_detector.h"

namespace pandora_vision
{
namespace pandora_vision_motion
{
  MotionDetector::MotionDetector()
  {
    kernel_erode_ = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

    history_ = 10;
    varThreshold_ = 16;
    bShadowDetection_ = false;
    numMixtures_ = 3;
    diffThreshold_ = 45;
    highMotionThreshold_ = 7500;
    lowMotionThreshold_ = 200;

    visualization = false;
    show_image = false;
    show_background = false;
    show_diff_image = false;
    show_moving_objects_contours = false;
    enableDBSCAN_ = false;
    maxDeviation_ = 25;
    bg_ = cv::BackgroundSubtractorMOG2(history_, varThreshold_,
        bShadowDetection_);

    ROS_INFO("Created Motion Detector instance");
  }

  MotionDetector::~MotionDetector()
  {
    ROS_INFO("Destroying MotionDetector instance");
  }

  std::vector<BBoxPOIPtr> MotionDetector::getMotionPosition() const
  {
    return boundingBoxes_;
  }

  void MotionDetector::setDiffThreshold(int diffThreshold)
  {
    diffThreshold_ = diffThreshold;
  }

  void MotionDetector::setHighMotionThreshold(double highMotionThreshold)
  {
    highMotionThreshold_ = highMotionThreshold;
  }

  void MotionDetector::setLowMotionThreshold(double lowMotionThreshold)
  {
    lowMotionThreshold_ = lowMotionThreshold;
  }

  void MotionDetector::setMaxDeviation(double maxDeviation)
  {
    maxDeviation_ = maxDeviation;
  }

  void MotionDetector::setEnableDBSCAN(bool enableDBSCAN)
  {
    enableDBSCAN_ = enableDBSCAN;
  }

  void MotionDetector::detectMotion(const cv::Mat& frame)
  {
    frame_ = frame.clone();
    /// Check that frame_ has data and that image has 3 channels
    if (frame_.data && frame_.channels() == 3)
    {
      movingObjects_ = frame_.clone();
      /// Update the background model and create
      /// binary mask for foreground objects
      bg_.operator()(frame_, foreground_);
      bg_.getBackgroundImage(background_);

      cv::subtract(frame_, background_, thresholdedDifference_);
      cv::cvtColor(thresholdedDifference_, thresholdedDifference_, CV_BGR2GRAY);

      cv::threshold(thresholdedDifference_, thresholdedDifference_,
        diffThreshold_, 255, cv::THRESH_BINARY);

      cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3),
        cv::Point(-1, -1));

      cv::morphologyEx(thresholdedDifference_, thresholdedDifference_,
       cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 8);

      detectMotionPosition();

      if (visualization || show_image ||
          show_background || show_diff_image ||
          show_moving_objects_contours)
      {
        debugShow();
      }
    }
    else
    {
      finalBoxes_.clear();
      boundingBoxes_.clear();
    }
  }

  void MotionDetector::detectMotionPosition()
  {
    /// Check that the thresholded difference image has data
    finalBoxes_.clear();
    boundingBoxes_.clear();
    if (thresholdedDifference_.data)
    {
      /// Calculate the standard deviation
      cv::Scalar mean, stdDev;
      meanStdDev(thresholdedDifference_, mean, stdDev);
      // ROS_INFO_STREAM("Motion stdev=" << stdDev[0] << " max_deviation= " << maxDeviation_);
      /// If not to much changes then the motion is real
      if (stdDev[0] < maxDeviation_)
      {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        /// find motion Contours
        cv::findContours(thresholdedDifference_.clone(), contours, hierarchy,
            CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::Mat mask = cv::Mat::zeros(thresholdedDifference_.size(), CV_8UC1);
        cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
        // ROS_INFO_STREAM("Contours found=" << contours.size());

        /// find motion Points
        std::vector<cv::Point> motionPoints;
        for (int i = 0; i < thresholdedDifference_.cols; i++)
        {
          for (int j = 0; j < thresholdedDifference_.rows; j++)
          {
            if (static_cast<int>(mask.at<uchar>(j, i)) == 255)
            {
              motionPoints.push_back(cv::Point(i, j));
            }
          }
        }
        // ROS_INFO_STREAM("motion points=" << motionPoints.size());

        if (contours.size() > 1 && enableDBSCAN_)
        {
          /// Start dbscan to cluster motion Points for localization
          // clusters_.reset(new ClusteredPoints);
          DBSCAN dbscan(motionPoints, 50.0, 1);
          dbscan.cluster();
          std::vector<std::vector<cv::Point> > clusters_ = dbscan.getClusters();

          // dbscan.getClusters(clusters_);
          // ROS_INFO_STREAM("CLUSTERS FOUND=" << clusters_.size());
          // if (clusters->size() > 0)
          // {
            // cohesion = dbscan.getCohesion(clusters);
            // for (int i = 0; i < cohesion.size(); i++)
            // {
              // std::cout << cohesion[i] << std::endl;
            // }
          // }

          // Bound clusters into a box
          for (int i = 0; i < clusters_.size(); i++)
          {
            finalBoxes_.push_back(cv::boundingRect(clusters_.at(i)));
          }
        }
        else if (motionPoints.size() != 0)
        {
          finalBoxes_.push_back(cv::boundingRect(motionPoints));
        }

        float probability;

        for (int i = 0; i < finalBoxes_.size(); i++)
        {
          probability = calculateMotionProbability(finalBoxes_[i], mask);
          if (probability == 0.0f)
            continue;

          BBoxPOIPtr boundingBox(new BBoxPOI());
          cv::rectangle(movingObjects_, finalBoxes_[i].tl(), finalBoxes_[i].br(),
              cv::Scalar(0, 255, 255), 1);

          boundingBox->setWidth(finalBoxes_[i].width);
          boundingBox->setHeight(finalBoxes_[i].height);
          boundingBox->setPoint(cv::Point(finalBoxes_[i].tl().x + finalBoxes_[i].width / 2,
                                finalBoxes_[i].tl().y + finalBoxes_[i].height / 2));
          boundingBox->setProbability(probability);
          boundingBoxes_.push_back(boundingBox);
        }
      }
    }
  }

  float MotionDetector::calculateMotionProbability(const cv::Rect& bbox, const cv::Mat& img)
  {
    int points = 0;
    for (int i = bbox.x; i < bbox.x + bbox.width; i++)
    {
      for (int j = bbox.y; j < bbox.y + bbox.height; j++)
      {
        if (static_cast<int>(img.at<uchar>(j, i)) == 255)
        {
          points++;
        }
      }
    }

    float probability = 0.0f;
    if (points > highMotionThreshold_)
      probability = 0.51f;
    else if (points >= lowMotionThreshold_)
      probability = 1.0f;
    // ROS_INFO_STREAM("PROB=" << probability << " points=" << points);
    return probability;
  }

  int MotionDetector::motionIdentification()
  {
    /// Counts value of non zero pixels in binary image in order
    /// to find the exact number of pixels, that differ from current
    /// frame_ and background
    int countDiff = countNonZero(thresholdedDifference_);
    if (countDiff > highMotionThreshold_)
      return 2;
    else if (countDiff > lowMotionThreshold_)
      return 1;
    else
      return 0;
  }

  void MotionDetector::debugShow()
  {
    std::vector<cv::Scalar> colors;
    cv::RNG rng(3);

    for (int i = 0; i <= finalBoxes_.size(); i++)
    {
      colors.push_back(HSVtoRGBcvScalar(rng(255), 255, 255));
    }
    for (int i = 0; i < finalBoxes_.size(); i++)
    {
      cv::Scalar color;
      int label = i;
      color = colors[label];
      putText(thresholdedDifference_, to_string(i), finalBoxes_[i].tl(), cv::FONT_HERSHEY_COMPLEX, .5, color, 1);
      cv::rectangle(thresholdedDifference_, finalBoxes_[i].tl(), finalBoxes_[i].br(), color, 2, CV_AA);
    }
    cv::Mat frame, background, temp, movingObjects;
    cv::resize(frame_, frame, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(background_, background, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(thresholdedDifference_, temp, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(movingObjects_, movingObjects, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);

    cv::Mat temp1[] = {temp, temp, temp};
    cv::Mat diff;
    cv::merge(temp1, 3, diff);

    cv::Mat displayImage = cv::Mat(frame.rows * 2, frame.cols * 2, frame.type());
    frame.copyTo(displayImage(cv::Rect(0, 0, frame.cols, frame.rows)));
    background.copyTo(displayImage(cv::Rect(background.cols, 0, background.cols, background.rows)));
    diff.copyTo(displayImage(cv::Rect(0, diff.rows, diff.cols, diff.rows)));
    movingObjects.copyTo(displayImage(cv::Rect(movingObjects.cols, movingObjects.rows,
                                      movingObjects.cols, movingObjects.rows)));

    if (visualization)
      cv::imshow("Original/Background/ThresholdedDiff/MovingObjects", displayImage);
    if (show_image)
      cv::imshow("Frame", frame);
    if (show_background)
      cv::imshow("Background", background);
    if (show_diff_image)
      cv::imshow("Thresholded difference between background and current frame_", diff);
    if (show_moving_objects_contours)
      cv::imshow("Moving objects in current frame", movingObjects);
    cv::waitKey(10);
  }

  cv::Scalar MotionDetector::HSVtoRGBcvScalar(int H, int S, int V)
  {
    int bH = H;  // H component
    int bS = S;  // S component
    int bV = V;  // V component
    double fH, fS, fV;
    double fR, fG, fB;
    const double double_TO_BYTE = 255.0f;
    const double BYTE_TO_double = 1.0f / double_TO_BYTE;

    // Convert from 8-bit integers to doubles
    fH = static_cast<double>(bH) * BYTE_TO_double;
    fS = static_cast<double>(bS) * BYTE_TO_double;
    fV = static_cast<double>(bV) * BYTE_TO_double;

    // Convert from HSV to RGB, using double ranges 0.0 to 1.0
    int iI;
    double fI, fF, p, q, t;

    if (bS == 0)
    {
      // achromatic (grey)
      fR = fG = fB = fV;
    }
    else
    {
      // If Hue == 1.0, then wrap it around the circle to 0.0
      if (fH >= 1.0f)
        fH = 0.0f;

      fH *= 6.0;  // sector 0 to 5
      fI = floor(fH);  // integer part of h (0,1,2,3,4,5 or 6)
      iI = static_cast<int>(fH);  // " " " "
      fF = fH - fI;  // factorial part of h (0 to 1)

      p = fV * (1.0f - fS);
      q = fV * (1.0f - fS * fF);
      t = fV * (1.0f - fS * (1.0f - fF));

      switch (iI)
      {
      case 0:
        fR = fV;
        fG = t;
        fB = p;
        break;
      case 1:
        fR = q;
        fG = fV;
        fB = p;
        break;
      case 2:
        fR = p;
        fG = fV;
        fB = t;
        break;
      case 3:
        fR = p;
        fG = q;
        fB = fV;
        break;
      case 4:
        fR = t;
        fG = p;
        fB = fV;
        break;
      default:  // case 5 (or 6):
        fR = fV;
        fG = p;
        fB = q;
        break;
      }
    }

    // Convert from doubles to 8-bit integers
    int bR = static_cast<int>(fR * double_TO_BYTE);
    int bG = static_cast<int>(fG * double_TO_BYTE);
    int bB = static_cast<int>(fB * double_TO_BYTE);

    // Clip the values to make sure it fits within the 8bits.
    if (bR > 255)
      bR = 255;
    if (bR < 0)
      bR = 0;
    if (bG >255)
      bG = 255;
    if (bG < 0)
      bG = 0;
    if (bB > 255)
      bB = 255;
    if (bB < 0)
      bB = 0;

    // Set the RGB cvScalar with G B R, you can use this values as you want too..
    return cv::Scalar(bB, bG, bR);  // R component
  }

}  // namespace pandora_vision_motion
}  // namespace pandora_vision
