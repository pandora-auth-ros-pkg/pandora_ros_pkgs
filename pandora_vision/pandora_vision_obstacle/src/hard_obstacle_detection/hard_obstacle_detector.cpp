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

#include <string>
#include <cmath>
#include <limits>

#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  HardObstacleDetector::HardObstacleDetector()
  {
  }

  HardObstacleDetector::HardObstacleDetector(const std::string& name,
      const ros::NodeHandle& nh)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node is on");

    nodeName_ = name;

    double robotXDimention, robotYDimention;
    nh.param("robot/robotXDimention", robotXDimention, 0.345);
    nh.param("robot/robotYDimention", robotYDimention, 0.345);

    nh.param("cellResolution", resolution_, 0.02);

    robotRows_ = robotXDimention / resolution_;
    robotCols_ = robotYDimention / resolution_;

    // Minimum acceptable value for the process to work properly
    robotStrength_ = robotRows_ * robotCols_;

    robotMask_ = cv::Mat::ones(robotRows_, robotCols_, CV_64FC1);

    edge_method_ = 1;
    edges_threshold_ = 30;
    edgeDetectionEnabled_ = true;

    show_input_image = false;
    show_edges_image = false;
    show_edges_thresholded_image = false;
    show_edges_and_unknown_image = false;
    show_new_map_image = false;
    show_unknown_probabilities = false;

    traversabilityMaskEnabled_ = false;
    edgeTraversabilityMaskEnabled_ = true;
    edgeDetectionEnabled_ = false;
    displayTraversabilityMapEnabled_ = true;
    traversabilityMaskPtr_.reset(new TraversabilityMask);
    // Load the robot's description and create it's 2d Elevation Mask.
    ROS_INFO("Loading robot description!");
    traversabilityMaskPtr_->loadGeometryMask(nh);
    traversabilityMaskPtr_->createMaskFromDesc();
    traversabilityMaskPtr_->setDetectRamps(detectRamps_);
    // ROS_INFO("Finished loading robot description and creating robot mask!");
    ROS_INFO("[Hard Obstacle Detector]: Created Detector Object!");
  }

  cv::Mat HardObstacleDetector::startDetection(const cv::Mat& inputImage)
  {
    // Check if input type is CV_64FC1
    if (inputImage.depth() != CV_64FC1)
    {
      ROS_ERROR_NAMED(nodeName_, "Hard obstacle node input image type was wrong");
      ROS_BREAK();
    }

    ROS_INFO_NAMED(nodeName_, "Hard obstacle detection has started");

    cv::Mat scaledImage(inputImage.size(), inputImage.depth());
    scaleInputImage(inputImage, &scaledImage);

    if (show_input_image)
    {
      // Show the input image
      showImage("The input image", scaledImage, 1);
    }


    TraversabilityMask::MatPtr elevationMapPtr_(new cv::Mat(inputImage));
    // *elevationMapPtr_ = inputImage;
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr_);
    cv::Mat edgesImage = inputImage.clone();
    if (edgeDetectionEnabled_)
    {
      detectEdges(scaledImage, &edgesImage);
      // Pass the unknown areas in edges image.
      fillUnknownAreas(inputImage, &edgesImage, 0);
    }
    else
    {
      edgesImage = inputImage;
    }

    if (show_edges_and_unknown_image)
    {
      showImage("The edges image with unknown areas", edgesImage, 1);
    }

    // Pass the robot mask on the complete area that was made.
    cv::Mat newMap;
    if (edgeTraversabilityMaskEnabled_)
    {
      createEdgeTraversabilityMap(edgesImage, &newMap);
      if (displayTraversabilityMapEnabled_)
        displayTraversabilityMap(newMap);
    }
    else if (traversabilityMaskEnabled_)
    {
      createTraversabilityMap(edgesImage, &newMap);
      if (displayTraversabilityMapEnabled_)
        displayTraversabilityMap(newMap);
    }
    else
    {
      robotMaskOnMap(edgesImage, &newMap);
    }

    return newMap;
  }

  /**
   * @brief Creates a traversability map using the input image.
   * @description Creates the traversability map for the current elevation map by iterating over the
   * input image and applying the robot's mask only it's non zero entries.
   * @param inputImage[const cv::Mat&] The input image whose non zero entries represent candidate obstacle cells.
   * It can be the elevation map itself or a processed image, such as an edge map from the elevation map.
   * @param traversabilityMap[cv::Mat*] The resulting traversability map.
   * @return void
   */
  void HardObstacleDetector::createTraversabilityMap(const cv::Mat& inputImage, cv::Mat* traversabilityMap)
  {
    // Initialize the output traversability map
    traversabilityMap->create(inputImage.size(), CV_8UC1);
    // Set all of it's cells to unknown.
    traversabilityMap->setTo(unknownArea);
    int robotMaskHeight = traversabilityMaskPtr_->getRobotMaskPtr()->rows;
    int robotMaskWidth = traversabilityMaskPtr_->getRobotMaskPtr()->cols;
    // Iterate over the map
    for (int i = robotMaskHeight / 2; i < inputImage.rows - robotMaskHeight / 2; ++i)
    {
      for (int j = robotMaskWidth / 2; j < inputImage.cols - robotMaskWidth / 2; ++j)
      {
        // Check that we are on a valid cell.
        if (inputImage.at<double>(i, j) == 0 || inputImage.at<double>(i, j) == unknownArea)
          traversabilityMap->at<int8_t>(i, j) = unknownArea;
        else
          traversabilityMap->at<uchar>(i, j) = traversabilityMaskPtr_->findTraversability(cv::Point(j, i));
      }
    }
  }

  /**
   * @brief Creates a traversability map using the input image.
   * @param inputImage[const cv::Mat&] The input image whose non zero entries represent candidate obstacle cells.
   * It can be the elevation map itself or a processed image, such as an edge map from the elevation map.
   * @param traversabilityMap[cv::Mat*] The resulting traversability map.
   w* @return void
   */
  void HardObstacleDetector::createEdgeTraversabilityMap(const cv::Mat& inputImage, cv::Mat* traversabilityMap)
  {
    const int local_radius = static_cast<int>(ceil(locality_radius_ / resolution_));
    // Initialize the output traversability map
    traversabilityMap->create(inputImage.size(), CV_8UC1);
    // Set all of it's cells to unknown.
    traversabilityMap->setTo(unknownArea);
    cv::Mat nonTraversableRowPoints = traversabilityMaskPtr_->findRowTraversability(inputImage);
    cv::Mat nonTraversableColPoints = traversabilityMaskPtr_->findColTraversability(inputImage);

    std::vector<cv::Point> occupiedPoints;
    std::vector<cv::Point> rampPoints;
    // Iterate over the map
    for (int ii = 0; ii < inputImage.rows; ++ii)  // this is y
    {
      for (int jj = 0; jj < inputImage.cols; ++jj)  // this is x
      {
        int8_t traversablePoint;
        if (ii < inputImage.rows - 1 && jj < inputImage.cols - 1)
        {
         /* if (detectRamps_ &&
              ((static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj)) == rampArea) ||
               (static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj)) == rampArea)))
          {
            traversablePoint = rampArea;
            rampPoints.push_back(cv::Point(jj, ii));
          } */
          if ((static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj)) == occupiedArea) ||
              (static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj)) == occupiedArea))
          {
            traversablePoint = occupiedArea;
            occupiedPoints.push_back(cv::Point(jj ,ii));
          }
          else if ((static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj)) == freeArea) ||
                   (static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj)) == freeArea))
          {
            traversablePoint = freeArea;
          }
          else
          {
            traversablePoint = unknownArea;
          }
        }
        else if (ii == inputImage.rows - 1)
        {
          traversablePoint = static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj));
          if (traversablePoint == occupiedArea)
            occupiedPoints.push_back(cv::Point(jj, ii));
          else if (traversablePoint == rampArea)
            rampPoints.push_back(cv::Point(jj, ii));
        }
        else
        {
          traversablePoint = static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj));
          if (traversablePoint == occupiedArea)
            occupiedPoints.push_back(cv::Point(jj, ii));
          else if (traversablePoint == rampArea)
            rampPoints.push_back(cv::Point(jj, ii));
        }
        traversabilityMap->at<int8_t>(ii, jj) = traversablePoint;
      }
    }

    // for (int ii = local_radius; ii < nonTraversableRowPoints.rows - local_radius; ++ii) {
      // for (int jj = local_radius; jj < nonTraversableRowPoints.cols - local_radius; ++jj) {
        // if (static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj)) == occupiedArea)
            // (static_cast<int8_t>(nonTraversableRowPoints.at<uchar>(ii, jj)) == freeArea))
  /*       { */
          // double maxHeight = -std::numeric_limits<double>::max();
          // double minHeight = std::numeric_limits<double>::max();
          // double mean = 0.0;
          // int count = 0;
          // for (double theta = 0; theta < 2 * CV_PI; theta += 0.2) {
            // int iic = ii + static_cast<int>(cos(theta) * local_radius);
            // int jjc = jj + static_cast<int>(sin(theta) * local_radius);
            // double height = inputImage.at<double>(iic, jjc);
            // if (height != -std::numeric_limits<double>::max())
            // {
              // mean += height;
              // count++;
              // if (height > maxHeight) maxHeight = height;
              // if (height < minHeight) minHeight = height;
            // }
          // }
          // // Escape if information is not considered sufficient
          // if (count < static_cast<int>((2 * CV_PI / 0.2) / 2))
            // continue;
          // mean /= count;
          // double heightDiff = inputImage.at<double>(ii, jj) - mean;
          // // Escape if edge point does not lie in high ground... (error)
          // if (heightDiff <= 0)
            // continue;
          // if (heightDiff <= height_diff_)
          // {
            // traversabilityMap->at<int8_t>(ii, jj) = freeArea;
            // continue;
          // }
          // // Escape if max is smaller than min
          // if (maxHeight < minHeight)
            // continue;
          // double grad = maxHeight - minHeight;
          // if (grad >= grad_diff_)
          // {
            // traversabilityMap->at<int8_t>(ii, jj) = freeArea;
            // // continue;
          // }

          // // Edge is definetely a hard obstacle
          // // occupiedPoints.push_back(cv::Point(jj ,ii));
        // }
      // }
    // }
    // for (int ii = local_radius; ii < nonTraversableColPoints.rows - local_radius; ++ii) {
      // for (int jj = local_radius; jj < nonTraversableColPoints.cols - local_radius; ++jj) {
        // if ((static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj)) == rampArea) ||
            // (static_cast<int8_t>(nonTraversableColPoints.at<uchar>(ii, jj)) == freeArea))
        // {
          // double maxHeight = -std::numeric_limits<double>::max();
          // double minHeight = std::numeric_limits<double>::max();
          // double mean = 0.0;
          // int count = 0;
          // for (double theta = 0; theta < 2 * CV_PI; theta += 0.2) {
            // int iic = ii + static_cast<int>(cos(theta) * local_radius);
            // int jjc = jj + static_cast<int>(sin(theta) * local_radius);
            // double height = inputImage.at<double>(iic, jjc);
            // if (height != -std::numeric_limits<double>::max())
            // {
              // mean += height;
              // count++;
              // if (height > maxHeight) maxHeight = height;
              // if (height < minHeight) minHeight = height;
            // }
          // }
          // // Escape if information is not considered sufficient
          // if (count < static_cast<int>((2 * CV_PI / 0.2) / 2))
            // continue;
          // mean /= count;
          // double heightDiff = inputImage.at<double>(ii, jj) - mean;
          // // Escape if edge point does not lie in high ground... (error)
          // if (heightDiff <= 0)
            // continue;
          // if (heightDiff <= height_diff_)
            // continue;
          // // Escape if max is smaller than min
          // if (maxHeight < minHeight)
            // continue;
          // double grad = maxHeight - minHeight;
          // if (grad >= grad_diff_)
            // continue;

          // // Edge is definetely a hard obstacle
          // traversabilityMap->at<int8_t>(ii, jj) = occupiedArea;
          // // occupiedPoints.push_back(cv::Point(jj ,ii));
        // }
      // }
  /*   } */
/*    if (detectRamps_)
    {
      for (int ii = 0; ii < rampPoints.size(); ++ii)
      {
        if (traversabilityMap->at<int8_t>(rampPoints[ii].y, rampPoints[ii].x) != rampArea)
          continue;
        for (int jj = 0; jj < occupiedPoints.size(); ++jj)
        {
          if (cv::norm(rampPoints[ii] - occupiedPoints[jj]) * resolution_ <= inflationRadius_)
          {
            traversabilityMap->at<int8_t>(occupiedPoints[jj].y, occupiedPoints[jj].x) = freeArea;
          }
        }
      }
    } */
  }

  void
  HardObstacleDetector::
  scaleInputImage(const cv::Mat& inImage, cv::Mat* outImage)
  {
    for (unsigned int rows = 0; rows < inImage.rows; rows++)
    {
      for (unsigned int cols = 0; cols < inImage.cols; cols++)
      {
        if (inImage.at<double>(rows, cols) >= min_input_image_value_)
        {
          outImage->at<double>(rows, cols) = inImage.at<double>(rows, cols) +
            fabs(min_input_image_value_);
        }
        else
        {
          outImage->at<double>(rows, cols) = 0.0;
        }
      }
    }
  }

  /*****************************************************************************
   *                         Visualization  methods                            *
   ****************************************************************************/
  void HardObstacleDetector::showImage(
    const std::string& title, const cv::Mat& image, int time)
  {
    cv::Mat scaledImage;
    if (image.depth() == CV_64FC1)
    {
      scaledImage = scaleFloatImageToInt(image);
      cv::cvtColor(scaledImage, scaledImage, CV_GRAY2BGR);

      // If value is negative, make it green for visualization
      for (unsigned int rows = 0; rows < image.rows; rows++)
      {
        for (unsigned int cols = 0; cols < image.cols; cols++)
        {
          if (image.at<double>(rows, cols) < 0)
          {
            scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 0;
            scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 255;
            scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
          }
        }
      }
    }
    else
    {
     image.copyTo(scaledImage);
    }
    //cv::imshow(title, scaledImage);
    cv::waitKey(time);
  }

  void HardObstacleDetector::displayTraversabilityMap(const cv::Mat& map)
  {
    cv::namedWindow("Traversability Map", cv::WINDOW_NORMAL);
    cv::Mat traversabilityVisualization(map.size(), CV_8UC3);
    traversabilityVisualization.setTo(0);
    for (int i = 0; i < map.rows; ++i)
    {
      for (int j = 0; j < map.cols; ++j)
      {
        // If the map value is unknown then paint it black.
        if (map.at<uchar>(i, j) == unknownArea)
        {
          traversabilityVisualization.at<cv::Vec3b>(i, j)[0] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[1] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[2] = 0;
        }
        // Paint the occupied Areas red
        else if (map.at<uchar>(i, j) == occupiedArea)
        {
          traversabilityVisualization.at<cv::Vec3b>(i, j)[0] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[1] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[2] = 255;
        }
        // Paint the free areas blue
        else if (map.at<uchar>(i, j) == freeArea)
        {
          traversabilityVisualization.at<cv::Vec3b>(i, j)[0] = 255;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[1] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[2] = 0;
        }
        else if (map.at<uchar>(i, j) == rampArea)
        {
          traversabilityVisualization.at<cv::Vec3b>(i, j)[0] = 0;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[1] = 255;
          traversabilityVisualization.at<cv::Vec3b>(i, j)[2] = 255;
        }
      }
    }
    //cv::imshow("Traversability Map", traversabilityVisualization);
    //
    cv::waitKey(5);
  }

  void HardObstacleDetector::visualizeUnknownProbabilities(
    const std::string& title, const cv::Mat& image,
    int time, bool opencv_method)
  {
    if (image.depth() == CV_64FC1)
    {
      cv::Mat scaledImage;

      if (opencv_method)
      {
        // Use opencv function for visualization
        cv::normalize(image, scaledImage, 0, 1, cv::NORM_MINMAX);
      }

      // Convert CV_8UC1 to CV_8UC3
      scaledImage = scaleFloatImageToInt(image);
      cv::cvtColor(scaledImage, scaledImage, CV_GRAY2RGB);

      if (opencv_method)
      {
        // use Opencv function for visualization
        cv::applyColorMap(scaledImage, scaledImage, cv::COLORMAP_JET);
      }

      if (!opencv_method)
      {
        // If value is negative give color
        for (unsigned int rows = 0; rows < image.rows; rows++)
        {
          for (unsigned int cols = 0; cols < image.cols; cols++)
          {
            double pixelValue = image.at<double>(rows, cols);
            double probability = applyFoldedNormalDistribution(pixelValue);

            // Check the probability of the pixel and give color for visualization
            if (probability < 0.35)
            {
              // Give Red color
              scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 0;
              scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 0;
              scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 255;
            }
            else if (probability < 0.67)
            {
              // Give Yellow color
              scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 255;
              scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 255;
              scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
            }
            else if (probability < 1.0)
            {
              // Give Blue color
              scaledImage.at<unsigned char>(rows, 3 * cols + 0) = 255;
              scaledImage.at<unsigned char>(rows, 3 * cols + 1) = 0;
              scaledImage.at<unsigned char>(rows, 3 * cols + 2) = 0;
            }
          }
        }
      }
      //cv::imshow(title, scaledImage);
      cv::waitKey(time);
    }
  }

  double HardObstacleDetector::applyFoldedNormalDistribution(double inValue)
  {
    double probability;

    return probability;
  }

  /*****************************************************************************
   *                     End of Visualization methods                          *
   ****************************************************************************/

  cv::Mat HardObstacleDetector::scaleFloatImageToInt(const cv::Mat& inImage)
  {
    cv::Mat outImage;

    double min;
    double max;
    cv::minMaxIdx(inImage, &min, &max);

    inImage.convertTo(outImage, CV_8UC1, 255.0 / max);

    return outImage;
  }

  void HardObstacleDetector::fillUnknownAreas(
    const cv::Mat& inImage, cv::Mat* outImage, int method)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node fills unknown area");

    switch (method)
    {
      case 0 :
        for (unsigned int rows = 0; rows < inImage.rows; rows++)
        {
          for (unsigned int cols = 0; cols < inImage.cols; cols++)
          {
            // Pass from the input mat the negative values as our policy dictates.
            if (inImage.at<double>(rows, cols) == -std::numeric_limits<double>::max())
            {
              outImage->at<double>(rows, cols) = -0.5 / robotStrength_;
            }
          }
        }
        break;
      case 1 :
        inImage.copyTo(*outImage);
        for (unsigned int rows = 0; rows < inImage.rows; rows++)
        {
          for (unsigned int cols = 0; cols < inImage.cols; cols++)
          {
            // If negative values in the input image, convert them to -1.
            if (outImage->at<double>(rows, cols) < 0)
            {
              outImage->at<double>(rows, cols) = -1;
            }
          }
        }
        break;
    }
  }

  void HardObstacleDetector::robotMaskOnMap(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node convolutes map with robot");

    cv::Mat newMap;

    cv::filter2D(inImage, newMap, -1, robotMask_,
      cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    if (show_new_map_image)
    {
      showImage("The new map after robot mask convolution", newMap, 1);
    }

    if (show_unknown_probabilities)
    {
      // Visualization of unknown areas based on their optimistic probabilities
      visualizeUnknownProbabilities("Unknown areas probabilities",
        newMap, 1, true);
    }

    // After convolution there might be negative values, so we need
    // to set them to -1.
    fillUnknownAreas(newMap, outImage, 1);
  }


  /*****************************************************************************
   *                         Edge Detection methods                            *
   ****************************************************************************/
  void HardObstacleDetector::applyCanny(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At canny detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Canny edge detection is called");

    // Reduce the noise of the input Image
    cv::blur(inImage, *outImage,
      cv::Size(cannyBlurKernelSize_, cannyBlurKernelSize_));

    // Detect edges with canny
    cv::Canny(*outImage, *outImage, cannyLowThreshold_,
      cannyLowThreshold_ * 3, cannyKernelSize_);
  }

  void HardObstacleDetector::applyScharr(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At scharr detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Scharr edge detection is called");

    cv::Mat blured;
    // Reduce the noise of the input image
    cv::GaussianBlur(inImage, blured, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    //        src, dst, ddepth, 1, 0, scale, delta, border_type
    cv::Scharr(blured, grad_x, CV_16S, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Scharr(blured, grad_y, CV_16S, 0, 1, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *outImage);
  }

  void HardObstacleDetector::applySobel(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    if (inImage.depth() != CV_8U)
    {
      ROS_ERROR_NAMED(nodeName_, "At sobel detection, inappropriate image depth");
      ROS_BREAK();
    }
    ROS_INFO_NAMED(nodeName_, "Sobel edge detection is called");

    cv::Mat blured;
    // Reduce the noise of the input image
    cv::GaussianBlur(inImage, blured, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // Gradient X
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Sobel(blured, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    // Gradient Y
    //        src, dst, ddepth, 1, 0, kernelSize, scale, delta, border_type
    cv::Sobel(blured, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    // Total Gradient (approximate)
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, *outImage);
  }

  void HardObstacleDetector::detectEdges(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    ROS_INFO_NAMED(nodeName_, "Hard obstacle node detects edges");

    cv::Mat scaledImage = scaleFloatImageToInt(inImage);

    switch (edge_method_)
    {
      case 0 :
        applyCanny(scaledImage, outImage);
        break;
      case 1 :
        applyScharr(scaledImage, outImage);
        break;
      case 2 :
        applySobel(scaledImage, outImage);
        break;
    }

    if (show_edges_image)
    {
      showImage("The edges image", *outImage, 1);
    }

    // Apply threshold to the edges
    cv::threshold(*outImage, *outImage, edges_threshold_, 255, CV_THRESH_BINARY);

    if (show_edges_thresholded_image)
    {
      showImage("The thresholded edges image", *outImage, 1);
    }

    // Convert the type of the output image to CV_64FC1.
    outImage->convertTo(*outImage, CV_64FC1, 1.0 / 255.0, 0.0);
  }

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
