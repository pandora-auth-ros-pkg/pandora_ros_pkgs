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
* Author: Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTOR_H 
#define PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTOR_H 

#include <iostream>
#include <stdlib.h>
#include <dmtx.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "pandora_vision_common/pandora_vision_interface/vision_processor.h"
#include "pandora_vision_datamatrix/datamatrix_poi.h"

#define DEBUG_MODE false

namespace pandora_vision 
{  
  class DatamatrixDetector : public VisionProcessor
  {
    public:
      /**
        @brief Default Constructor
        @return void
      **/
      DatamatrixDetector(const std::string& ns, sensor_processor::Handler* handler);
      
      /**
        @brief Constructor
      **/
      DatamatrixDetector();
      
      /**
        @brief Default Destructor
        @return void
      **/
      virtual ~DatamatrixDetector();
      
      /**
        @brief Detects datamatrixes and stores them in a vector.
        @param image [cv::Mat] The image in which the datamatrixes are detected
        @return void
      **/
      std::vector<POIPtr> detect_datamatrix(cv::Mat image);
      
      /**
      @brief Function that finds the position of datamatrixe's center
      @param image [cv::Mat] The image in which the datamatrixes are detected
      @return void
      */
      void locate_datamatrix(cv::Mat image);
      
      /**
      @brief Function that creates view for debugging purposes.
      @param image [cv::Mat] The image in which the datamatrixes are detected
      @param datamatrixVector [std::vector<cv::Point2f>] The vector of 4 corners 
      of datamatrix image, according to which i draw lines for debug reasons
      @return debug_frcame [cv::Mat], frame with rotated rectangle
      and center of it
      */
      void debug_show(cv::Mat image, std::vector<cv::Point2f> datamatrixVector);
      
      virtual bool process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output);
      
    private:
      DmtxMessage *msg;
      DmtxImage *img;
      DmtxDecode *dec;
      DmtxRegion *reg;
      DmtxTime timeout;
      
      cv::Mat datamatrix_frame;
      cv::Mat debug_frame;
      
      image_transport::Publisher _datamatrixPublisher;
      
      DataMatrixPOIPtr detected_datamatrix;
      
      //!< List of detected datamatrixes
      std::vector<POIPtr> datamatrix_list;  
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTOR_H
