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
 * Authors: Choutas Vassilis 
 *********************************************************************/


#ifndef PANDORA_VISION_HAZMAT_DETECTION_UTILITIES_H
#define PANDORA_VISION_HAZMAT_DETECTION_UTILITIES_H

/**
  Header file used for initializing useful data structures and 
  including the necessary libraries.
**/

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <limits>

#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Bool.h"
#include "cv_bridge/cv_bridge.h"

// Filter Libraries.
#include "pandora_vision_hazmat/filters/image_signature.h"



//#define CHRONO 
#define DEBUG 
#define FEATURES_CHRONO

#define PI boost::math::constants::pi<float>()

#if defined(CHRONO) || defined(FEATURES_CHRONO) 
#include "sys/time.h"
#endif

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /*
     * @struct Object
     * @brief : Contains object information.
     */
    struct Object
    {
     public:
       std::string name; //<! Name of the detected object.
       int id; //<! A numerical id for the object.
       cv::Point2f position; //<! The object's position on the image.
       /*
        * @brief Costructor for an object instance.
        * @param objectName[const std::string&]: The name of the object
        * @param objectPosition[const cv::Point2f&]: The position of the 
        * object.
        * param objectID[const int&]: The id of the object.
        */
       Object(const std::string& objectName,
           const cv::Point2f& objectPosition, const int& objectID)
       {
         name = objectName;
         position = objectPosition;
         id = objectID;
       }
    };

    struct Pattern
    {
     public: 

      std::string name; //<! Name of the pattern.

      std::vector<cv::Point2f> boundingBox; //<! A vector of 2D points that
                                            //<! contains the bounding box
                                            //<! and the center of 
                                            //<!the pattern.

      std::vector<cv::Point2f> keyPoints; //<! Vector of detected keypoints 
                                          //<! in the pattern.

      cv::Mat descriptors; //<! Matrix of image descriptors .

      cv::Mat histogram; //<! Color histogram of the pattern.
    };
  
} // namespace pandora_vision_hazmat
} // namespace pandora_vision

#endif  // PANDORA_VISION_HAZMAT_DETECTION_UTILITIES_H
