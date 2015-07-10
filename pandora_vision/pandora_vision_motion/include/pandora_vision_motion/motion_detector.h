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
 *   Despoina Pascahlidou,
 *   Kofinas Miltiadis <mkofinas@gmail.com>,
 *   Protopapas Marios <protopapas_marios@hotmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_MOTION_MOTION_DETECTOR_H
#define PANDORA_VISION_MOTION_MOTION_DETECTOR_H

#include <string>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "pandora_vision_common/cv_mat_stamped.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_common/bbox_poi.h"
#include "pandora_vision_motion/dbscan.h"

namespace pandora_vision
{
namespace pandora_vision_motion
{
  template <class T>
  inline std::string to_string(const T& t)
  {
      std::stringstream ss;
      ss << t;
      return ss.str();
  }

  class MotionDetector
  {
    public:
      /**
       * @brief Class Constructor
       * Initializes all parameters for motion detection.
       */
      MotionDetector();

      /**
       * @brief Class Destructor
       */
      virtual ~MotionDetector();

      /**
       * @brief This function returns the bounding boxes of the detected moving
       * objects.
       * @return [std::vector<BBoxPOIPtr>] The bounding boxes of the moving
       * objects.
       */
      std::vector<BBoxPOIPtr> getMotionPosition() const;

      /**
       * @brief This function sets the threshold on which the background
       * subtracted frame values will be thresholded to create a binary image.
       * @param diffThreshold [int] The background-foreground difference
       * threshold.
       * @return void
       */
      void setDiffThreshold(int diffThreshold);

      /**
       * @brief This function sets the threshold above which motion is
       * considered to be high.
       * @param highMotionThreshold [double] The high motion threshold.
       * @return void
       */
      void setHighMotionThreshold(double highMotionThreshold);

      /**
       * @brief This function sets the threshold above which motion is
       * considered to exist.
       * @param lowMotionThreshold [double] The low motion threshold.
       * @return void
       */
      void setLowMotionThreshold(double lowMotionThreshold);

      /**
       * @brief This function sets the maximum standard deviation above which
       * the motion is considered to be noise.
       * @param maxDeviation [double] The maximum standard deviation.
       * @return void
       */
      void setMaxDeviation(double maxDeviation);

      /**
       * @brief This function sets the variable deciding whether to perform
       * DBSCAN clustering in the moving objects contours or not.
       * @param enableDBSCAN [bool] The parameter responsible for the DBSCAN
       * clustering.
       * @return void
       */
      void setEnableDBSCAN(bool enableDBCAN);

      /**
       * @brief Function that detects motion, according to substraction
       * between background image and current frame. The existence and the type
       * of motion are detected according to predefined thresholds.
       * @param frame [const cv::Mat&] Current frame to be processed.
       * @return void
       */
      void detectMotion(const cv::Mat& frame);

    private:
      /**
       * @brief This function defines the type of movement according to the
       * number of pixels that differ from the current frame and the background.
       * @return [int] The type of movement. 0 corresponds to stationary scene,
       * 1 corresponds to low mobility in the scene, whereas 2 corresponds to
       * high mobility in the scene.
       */
      int motionIdentification();

      /**
       * @brief This function shows background, foreground and contours of
       * motion trajectories in the current frame, according to a set of
       * parameters.
       * @return void
       */
      void debugShow();

      /**
       * @brief This function estimates the moving objects in a frame and
       * calculates their position in the frame.
       * @return void
       */
      void detectMotionPosition();

      /**
       * @brief This function randomly picks a color in the RGB color space.
       * @param H [int] The H value.
       * @param S [int] The S value.
       * @param V [int] The V value.
       * @return [cv::Scalar] The chosen color.
       */
      cv::Scalar HSVtoRGBcvScalar(int H, int S, int V);

      /**
       * @brief This function estimates the probability of an object moving.
       * @param bbox [const cv::Rect&] The bounding box of a moving object.
       * @param img [const cv::Mat&] The image containing the contours of the
       * moving objects.
       * @return [float] The estimated probability.
       */
      float calculateMotionProbability(const cv::Rect& bbox, const cv::Mat& img);

    public:
      //@{
      /** Visualization parameters */
      bool visualization;
      bool show_image;
      bool show_background;
      bool show_diff_image;
      bool show_moving_objects_contours;
      //@}

    private:
      //@{
      /** Background segmentation parameters. */
      /// Length of the history.
      int history_;
      /// Threshold on the squared Mahalanobis distance to decide whether it is
      /// well described by the background model.
      int varThreshold_;
      /// Parameter defining whether shadow detection should be enabled.
      bool bShadowDetection_;
      /// Number of Gaussian mixtures.
      int numMixtures_;
      //@}

      //@{
      /** Threshold parameters. */
      int diffThreshold_;
      double highMotionThreshold_;
      double lowMotionThreshold_;
      //@}

      /// Current frame to be processed
      cv::Mat frame_;
      /// Background image
      cv::Mat background_;
      /// Foreground mask
      cv::Mat foreground_;
      cv::Mat movingObjects_;

      /// Thresholded difference image
      cv::Mat thresholdedDifference_;
      /// Class instance for Gaussian Mixture-based background
      /// and foreground segmentation
      cv::BackgroundSubtractorMOG2 bg_;
      /// Erode kernel
      cv::Mat kernel_erode_;
      /// Maximum deviation for calculation position of moving objects.
      double maxDeviation_;
      /// Bounding box of moving objects.
      std::vector<BBoxPOIPtr> boundingBoxes_;

      std::vector<cv::Rect> finalBoxes_;
      std::vector<double> cohesion;

      /// Enable dbscan
      bool enableDBSCAN_;

      // ClusteredPointsPtr clusters_;

      friend class MotionDetectorTest;
  };
}  // namespace pandora_vision_motion
}  // namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_MOTION_DETECTOR_H
