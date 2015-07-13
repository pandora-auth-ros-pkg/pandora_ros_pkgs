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
 *   Bosdelekidis Vasilis <vasilis1bos@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_DETECTOR_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include "pandora_vision_obstacle/obstacle_poi.h"
#include <ros/ros.h>
#include "pandora_vision_obstacle/barrel_detection/fast_symmetry_detector.h"
#include "pandora_vision_obstacle/barrel_detection/parameters.h"
#include "pandora_vision_msgs/ObstacleAlert.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class BarrelDetector
  {
    public:
      /**
       * @brief Constructor
       **/
      BarrelDetector(const std::string& name, const ros::NodeHandle& nh);
      /**
       * @brief Constructor without NodeHandle used for testing
       **/
      BarrelDetector();

      /**
       * @brief Destructor
       **/
      ~BarrelDetector() {}

    public:
      /**
       * @brief Detect barrel by performing a number of
       * operations to the rgb and depth image
       * @param rgbImage [const cv::Mat&] The input rgb image
       * @param depthImage [const cv::Mat&] The input depth image
       * @return [std::vector<POIPtr>] The Point Of Interest that
       * includes the barrel
       **/
      std::vector<POIPtr> detectBarrel(const cv::Mat& rgbImage,
          const cv::Mat& depthImage);

      /**
        @brief Find symmetric object inside frame
        @description Use fast symmetry detector algorith to find symmetric objects
        based on edges extracted from Canny edge detector  
        @param[in] inputImage [const cv::Mat&] Input depth image where we do the 
        processing
        @param[in out] roi [cv::Rect*] Here the candidate roi is stored
        @param[in] symmetricStartPoint [cv::Point*] The symmetry's line start point
        @param[in] symmetricEndPoint [cv::Point*] The symmetry's line end point
        @return void
       **/
      void getSymmetryObject(
          const cv::Mat& inputImage,
          cv::Rect* roi,
          cv::Point* symmetricStartPoint,
          cv::Point* symmetricEndPoint);

      /**
        @brief Validates the ROI for barrel existence
        @description Keep; 
        1. Homogeneous regions in rgb
        2. Regions with decreasing depth from left to the symmetry line and increasing
        depth from symmetry line to right
        3. Regions with almost identical variation between abovementioned two parts
        4. Regions with almost stable depth through the symmetry line
        5. Regions with points through the perpendicular to the symmetry line that
        belong to the equation of a circle
        6. Regions which do not contain many corners
        @param[in] rgbImage [const cv::Mat&] The rgb image
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] rectRoi [const cv::Rect&] The ROI to validate
        @param[in] symmetricStartPoint [const cv::Point&] The symmetry's line start point
        @param[in] symmetricEndPoint [const cv::Point&] The symmetry's line end point
        @return [bool] A flag indicating roi's validity
       **/
      bool validateRoi(
          const cv::Mat& rgbImage,
          const cv::Mat& depthImage,
          const cv::Rect& rectRoi,
          const cv::Point& symmetricStartPoint,
          const cv::Point& symmetricEndPoint);

      /**
        @brief Finds if a point belongs to a 3D circle's equation. All points
        given are 2D and the third coordinate is depth.
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] point [const cv::Point&] The point to check
        @param[in] center [const cv::Point&] The center of the circle
        @param[in] normalLineStartPoint [const cv::Point&] The first point of the line perpendicular to the circle's plane
        @param[in] normalLineEndPoint [const cv::Point&] The second point of the line perpendicular to the circle's plane
        @param[in] centerDepth [float] The depth of the center
        @param[in] radius [float] The radius of the circle
        @return [bool] A flag indicating whether point belongs in the circle
       **/
      bool isInCircle(
          const cv::Mat& depthImage,
          const cv::Point& point,
          const cv::Point& center,
          const cv::Point& normalLineStartPoint,
          const cv::Point& normalLineEndPoint,
          float centerDepth,
          float radius);

      float findDepthDistance(const cv::Mat& depthImage,
          const cv::Rect& roi);

      /**
        @brief Used for visualization of the desired bounding rectangle on the input
        image
        @param[in] image [const cv::Mat&] The input image where the roi will be
        visualized
        @param[in] rectRoi [const cv::Rect&] The ROI to show
        @param[in] visualizationMode [int] What we want to visualize; 1 - respective
        2 - valid
        @return void
       **/
      void debugShow(
          const cv::Mat& image,
          const cv::Rect& rectRoi,
          int visualizationMode);

    private:
      /// The node's name
      std::string nodeName_;
  };
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_BARREL_DETECTION_BARREL_DETECTOR_H

