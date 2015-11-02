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
* Author:  Despoina Pascahlidou
*********************************************************************/

#ifndef PANDORA_VISION_COLOR_COLOR_DETECTOR_H
#define PANDORA_VISION_COLOR_COLOR_DETECTOR_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "pandora_vision_common/cv_mat_stamped.h"
#include "pandora_vision_color/obstacle_poi.h"
#include "pandora_vision_msgs/ObstacleAlert.h"

namespace pandora_vision
{
namespace pandora_vision_color
{
  class ColorDetector
  {
    public:
      /**
        @brief Class Constructor
        Initializes all varialbes for thresholding
      */
      ColorDetector(void);

      /**
        @brief Class Destructor
      */
      virtual ~ColorDetector();

      std::vector<POIPtr> getColorPosition(void);

      /**
        @brief Function that detects color
        @return void
      */
      void detectColor(const cv::Mat& frame);

      bool visualization_;

      //!< HSV range values
      int iLowH;
      int iHighH;

      int iLowS;
      int iHighS;

      int iLowV;
      int iHighV;
      
      double minArea_;

    protected:
       /**
        @brief Function used for debug reasons
        @return void
      */
      void debugShow();

      /**
        @brief Function that calculates color's position
        @return void
      */
      void detectColorPosition();

    private:
      //!< Current frame to be processed
      cv::Mat frame_;
      //!< HSV image
      cv::Mat hsvFrame_;
      //!< binary image
      cv::Mat binary_;
      //!< Bounding box of moving objects.
      std::vector<POIPtr> bounding_boxes_;

     // friend class ColorDetectorTest;
  };
}  // namespace pandora_vision_color
}  // namespace pandora_vision
#endif  // PANDORA_VISION_COLOR_COLOR_DETECTOR_H
