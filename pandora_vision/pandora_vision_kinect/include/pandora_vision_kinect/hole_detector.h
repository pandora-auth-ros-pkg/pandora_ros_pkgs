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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#ifndef KINECT_HOLE_DETECTOR
#define KINECT_HOLE_DETECTOR

#include "pandora_vision_kinect/hole_filters.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace vision
{
  /**
    @class HoleDetector
    @brief Provides the functionalities for detecting holes [functional]
   **/
  class HoleDetector
  {
    public:

      static ParticleFilter pf;

      /**
        @brief The HoleDetector constructor
       **/
      HoleDetector(void);

      /**
        @brief Finds the holes provided a depth image in CV_32FC1 format
        @param[in] depthImage [cv::Mat] The depth image in CV_32FC1 format
        @param[in] pointCloud [pcl::PointCloud<pcl::PointXYZ>::Ptr] The point
        cloud from which the depth image is extracted
        @return std::vector<cv::Point2f> Centers of the possible holes
       **/
      static HoleFilters::HolesConveyor findHoles(cv::Mat depthImage,
          pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
  };
}

#endif
