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
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *  Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H

#include <string>
#include <vector>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class HardObstacleDetector
  {
    public:
      HardObstacleDetector();

      HardObstacleDetector(const std::string& name,
          const ros::NodeHandle& nh);

      virtual ~HardObstacleDetector()
      {
      }

      /**
        @brief Start of the hard obstacle detection process.
        @param[in] inputImage [const cv::Mat&] The input image from preprocessor
        @return cv::Mat
       **/
      cv::Mat startDetection(const cv::Mat& inputImage);

      /**
       * @brief Creates a traversability map using the input image.
       * @description Creates the traversability map for the current elevation map by iterating over the
       * input image and applying the robot's mask only it's non zero entries.
       * @param inputImage[const cv::Mat&] The input image whose non zero entries represent candidate obstacle cells.
       * It can be the elevation map itself or a processed image, such as an edge map from the elevation map.
       * @param traversabilityMap[cv::Mat*] The resulting traversability map.
       * @return void
       */
      void createTraversabilityMap(const cv::Mat& inputImage, cv::Mat* traversabilityMap);

      /**
      * @brief Creates a traversability map using the input image.
      * @param inputImage[const cv::Mat&] The input image whose non zero entries represent candidate obstacle cells.
      * It can be the elevation map itself or a processed image, such as an edge map from the elevation map.
      * @param traversabilityMap[cv::Mat*] The resulting traversability map.
      * @return void
      */
      void createEdgeTraversabilityMap(const cv::Mat& inputImage, cv::Mat* traversabilityMap);

      /**
       @brief Functions used to set debug show configuration parameters.
       **/
      inline void setShowInputImage(bool value){show_input_image = value;}
      inline void setShowEdgesImage(bool value){show_edges_image = value;}
      inline void setShowEdgesThresholdedImage(bool value){show_edges_thresholded_image = value;}
      inline void setShowEdgesUnknownImage(bool value){show_edges_and_unknown_image = value;}
      inline void setShowUnknownProbabilities(bool value){show_unknown_probabilities = value;}
      inline void setShowNewMapImage(bool value){show_new_map_image = value;}

      /**
        @brief Functions used to set edge detection configuration parameters.
       **/
      inline void setEdgeMethod(int value){edge_method_ = value;}
      inline void setEdgesThreshold(int value){edges_threshold_ = value;}

      inline void setLocalityRadius(double value){locality_radius_ = value;}
      inline void setHeightDiff(double value){height_diff_ = value;}
      inline void setGradDiff(double value){grad_diff_ = value;}

      inline void setCannyKernelSize(int value){cannyKernelSize_ = value;}
      inline void setCannyLowThreshold(int value){cannyLowThreshold_ = value;}
      inline void setCannyBlurNoiseKernelSize(int value){cannyBlurKernelSize_ = value;}

      inline void setMinInputImageValue(int value){min_input_image_value_ = value;}
      inline void setEdgeDetectionEnableFlag(bool edgeDetectionEnabledFlag)
      {
        edgeDetectionEnabled_ = edgeDetectionEnabledFlag;
      }
      inline void setTraversabilityMaskEnableFlag(bool traversabilityMaskEnableFlag)
      {
        traversabilityMaskEnabled_ = traversabilityMaskEnableFlag;
      }
      inline void setEdgeTraversabilityMaskEnableFlag(bool edgeTraversabilityMaskEnableFlag)
      {
        edgeTraversabilityMaskEnabled_ = edgeTraversabilityMaskEnableFlag;
      }
      inline void setTraversabilityMaskDisplay(bool traversabilityMaskDisplay)
      {
        displayTraversabilityMapEnabled_ = traversabilityMaskDisplay;
      }

      inline void setElevationDifferenceHighOccupiedThreshold(double elevationDifferenceHighOccupiedThreshold)
      {
        traversabilityMaskPtr_->setElevationDifferenceHighOccupiedThreshold(elevationDifferenceHighOccupiedThreshold);
      }
      inline void setElevationDifferenceLowOccupiedThreshold(double elevationDifferenceLowOccupiedThreshold)
      {
        traversabilityMaskPtr_->setElevationDifferenceLowOccupiedThreshold(elevationDifferenceLowOccupiedThreshold);
      }

      inline void setElevationDifferenceHighFreeThreshold(double elevationDifferenceHighFreeThreshold)
      {
        traversabilityMaskPtr_->setElevationDifferenceHighFreeThreshold(elevationDifferenceHighFreeThreshold);
      }
      inline void setElevationDifferenceLowFreeThreshold(double elevationDifferenceLowFreeThreshold)
      {
        traversabilityMaskPtr_->setElevationDifferenceLowFreeThreshold(elevationDifferenceLowFreeThreshold);
      }

      inline void setInflationRadius(double radius)
      {
        inflationRadius_ = radius;
      }

      inline void setDetectRamps(bool detectRamps)
      {
        detectRamps_ = detectRamps;
      }

    private:
      void displayTraversabilityMap(const cv::Mat& map);

      void
      scaleInputImage(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Visualization of an image with CV_32FC1 or CV_8UC1 type.
        If there is a negative value in mat aka unknown area set it 255 for
        visualization.
        @param[in] title [const std::string&] The title of image to be shown.
        @param[in] image [const cv::Mat&] The image to be shown.
        @param[in] time [int] The time that imshow function lasts in ms.
        @return void
       **/
      void showImage(
        const std::string& title, const cv::Mat& image, int time);

      /**
        @brief This function takes runs on every unknown pixel the folded normal
        distribution and based on its probability gives color to them. If
        opencv_method is enabled the above procedure is done via opencv functions.
        @param[in] title [const std::string&] The title of image to be shown.
        @param[in] image [const cv::Mat&] The image to be shown.
        @param[in] time [int] The time that imshow function lasts in ms.
        @param[in] opencv_method [bool] If true enables opencv functions
        for visualization.
        @return void
       **/
      void visualizeUnknownProbabilities(
        const std::string& title, const cv::Mat& image,
        int time, bool opencv_method);

      /**
        @brief Apply the Folded Normal Distribution to find the probability of
        each unknown pixel. The extracted probability range is between [0 - 1].
        The bigger the probability means that this pixel is very optimistic
        because many of it's neighboors are free (safe) for the robot.
        @param[in] inValue [double] The input value to find it's probability.
        @return double. The extracted probability.
       **/
      double applyFoldedNormalDistribution(double inValue);

      /**
        @brief Converts an image of CV_32FC1 type to CV_8UC1. Negative values
        will be replaced with zero values.
        @param[in] inImage [const cv::Mat&] The input image.
        @return cv::Mat
       **/
      cv::Mat scaleFloatImageToInt(const cv::Mat& inImage);

      /**
        @brief If method is 0 checks if there is any negative value in the input
        mat, if yes fill the peer pixel in outImage with the appropriate value
        to continue the process. If method is 1 find negative values and set
        them to -1.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] method [int] The method to be used.
        @param[out] outImage [cv::Mat*] The output image with unknown areas.
        @return void
       **/
      void fillUnknownAreas(const cv::Mat& inImage, cv::Mat* outImage, int method);

      /**
        @brief Now that we have a complete map, aka dangerous, safe and unknown
        areas we will include the robot's size. A convolution between the map
        and robot's mask will make the new map that informs us if we can access
        a particular point(territory). The outImage will have -1 values for
        unknown, 0 for free and 1 values for dangerous territories.
        Even if 1 dangerous pixel exists (value > 0 from the edges that
        we found) the output image pixel will be set as obstacle.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[out] outImage [cv::Mat*] The output edges image will unknown areas.
        @return void
       **/
      void robotMaskOnMap(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the canny edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[out] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applyCanny(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the sharr edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[out] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applyScharr(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the sobel edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[out] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applySobel(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply edge detection algorithm. Based on configuration parameter,
        select the desired method. Finally apply threshold on the extracted
        edges to keep the ones that we consider as dangerous areas.
        The outImage will have double values 0 or 1 for desired edges.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[out] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void detectEdges(const cv::Mat& inImage, cv::Mat* outImage);
    private:
      std::string nodeName_;

      TraversabilityMaskPtr traversabilityMaskPtr_;

      // The robots mask dimentions found as robotDimention / ogm_cell_resolution
      int robotRows_;
      int robotCols_;
      cv::Mat robotMask_;

      // This variable defines the probability of the extracted unknown areas
      // when the robot is near unknown and free space.
      // It is set to robotRows * robotCols in constructor and this is its
      // minimum acceptable value. Recommended not to change.
      // The higher the value, the lower the probability.
      int robotStrength_;

      // The parameter that defines the edge detection algorithm to be used
      int edge_method_;

      // Parameters used for canny edge detection method
      int cannyKernelSize_;
      int cannyLowThreshold_;
      int cannyBlurKernelSize_;

      // The parameter used to apply threshold  on the edges image
      int edges_threshold_;

      double locality_radius_;
      double height_diff_;
      double grad_diff_;

      double min_input_image_value_;

      // Debug parameters
      bool show_input_image;
      bool show_edges_image;
      bool show_edges_thresholded_image;
      bool show_edges_and_unknown_image;
      bool show_new_map_image;
      bool show_unknown_probabilities;

      bool edgeDetectionEnabled_;
      bool traversabilityMaskEnabled_;
      bool edgeTraversabilityMaskEnabled_;
      bool displayTraversabilityMapEnabled_;

      double inflationRadius_;

      double resolution_;

      bool detectRamps_;
  };

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H
