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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Vassilis Choutas <vasilis4ch@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 **********************************************************************/

#include <limits>
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  TraversabilityMask::
    TraversabilityMask()
    {
    }

  TraversabilityMask::TraversabilityMask(const RobotGeometryMaskDescriptionPtr& descriptionPtr)
  {
    ROS_INFO("[Traversability Mask]: Creating Traversability Mask object!");
    description_ = descriptionPtr;

    // Create the robot height mask.
    createMaskFromDesc(description_);

    ROS_INFO("[Traversability Mask]: Finished constructing Traversability Mask Object.");
  }

  TraversabilityMask::~TraversabilityMask() {}

  void TraversabilityMask::createMaskFromDesc()
  {
    // Calculate the values for the size of the robot parts.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    int barrelSize = metersToSteps(description_->barrelD);
    int totalSize = robotSize + 2 * (barrelSize + wheelSize);

    // Initialize the mask of the robot.
    robotGeometryMask_.reset(new cv::Mat(totalSize, totalSize, CV_64FC1, cv::Scalar(0)));

    // Assign the values for the motors of the robots.
    // Top Left Barrels
    (*robotGeometryMask_)(cv::Rect(0, wheelSize, wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize, 0, wheelSize, wheelSize)) = description_->barrelH;

    // Top Right Barrels
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, 0,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize,
          wheelSize, wheelSize)) = description_->barrelH;

    // robotGeometryMask_->at<double>(i + wheelSize, j) = description_->barrelH;
    // Top Right Barrel Height
        // Bottom Left Barrels.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize,
          wheelSize + 2 * barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Right Barrels.
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, wheelSize + 2 * barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize,
          wheelSize + barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;

    // Robot sidelines.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize, wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize + barrelSize,
          wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, 0, robotSize, wheelSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, wheelSize + 2 * barrelSize + robotSize,
          robotSize, wheelSize)) = description_->robotH;

    // Robot Main body.
    (*robotGeometryMask_)(cv::Rect(wheelSize, wheelSize, 2 * barrelSize + robotSize,
          2 * barrelSize + robotSize)) = description_->robotH;
    return;
  }

  /**
   * @brief Creates the Robot Height Mask
   * @description Creates the height mask for the robot according to the description file.
   * @param inputOutputMap[const MatPtr&] A pointer to the matrix that contains the mask.
   * @param description[RobotGeometryMaskDescriptionPtr] A pointer to the parameter structure
   * that approximately describes the robot.
   * @return void
   */
  void TraversabilityMask::createMaskFromDesc(const RobotGeometryMaskDescriptionPtr& description)
  {
    // Calculate the values for the size of the robot parts.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    int barrelSize = metersToSteps(description_->barrelD);
    int totalSize = robotSize + 2 * (barrelSize + wheelSize);
    // Initialize the mask of the robot.
    robotGeometryMask_.reset(new cv::Mat(totalSize, totalSize, CV_64FC1, cv::Scalar(0)));

    // Assign the values for the motors of the robots.
    // Top Left Barrels
    (*robotGeometryMask_)(cv::Rect(0, wheelSize, wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize, 0, wheelSize, wheelSize)) = description_->barrelH;

    // Top Right Barrels
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, 0,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize,
          wheelSize, wheelSize)) = description_->barrelH;

    // robotGeometryMask_->at<double>(i + wheelSize, j) = description_->barrelH;
    // Top Right Barrel Height
        // Bottom Left Barrels.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize,
          wheelSize + 2 * barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Right Barrels.
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, wheelSize + 2 * barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize,
          wheelSize + barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;

    // Robot sidelines.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize, wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize + barrelSize,
          wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, 0, robotSize, wheelSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, wheelSize + 2 * barrelSize + robotSize,
          robotSize, wheelSize)) = description_->robotH;

    // Robot Main body.
    (*robotGeometryMask_)(cv::Rect(wheelSize, wheelSize, 2 * barrelSize + robotSize,
          2 * barrelSize + robotSize)) = description_->robotH;
    return;
  }


  /**
   * @brief Check if the provided point is traversible by the robot or not
  */
  int8_t
  TraversabilityMask::findTraversability(const cv::Point& center)
  {
    center_ = center;

    int maskSize = robotGeometryMask_->rows;
    // Calculate the current position of the Upper Left Wheel.
    cv::Point upperLeftWheelPos(
        center_.x - maskSize / 2,
        center_.y - maskSize / 2);
    // Calculate the current position of the Lower Left Wheel.
    cv::Point lowerLeftWheelPos(
        center_.x - maskSize / 2,
        center_.y + maskSize / 2 - metersToSteps(description_->wheelD));
    cv::Point upperRightWheelPos(
        center_.x + maskSize / 2  - metersToSteps(description_->wheelD),
        center_.y - maskSize / 2);
    // Calculate the current position of the Lower Right Wheel.
    cv::Point lowerRightWheelPos(
        center_.x + maskSize / 2  - metersToSteps(description_->wheelD),
        center_.y + maskSize / 2  - metersToSteps(description_->wheelD));

    double upperLeftWheelMeanHeight, upperLeftWheelStdDev;
    bool upperLeftWheelValid = findHeightOnWheel(upperLeftWheelPos, &upperLeftWheelMeanHeight,
        &upperLeftWheelStdDev);

    double lowerLeftWheelMeanHeight, lowerLeftWheelStdDev;
    bool lowerLeftWheelValid = findHeightOnWheel(lowerLeftWheelPos, &lowerLeftWheelMeanHeight,
        &lowerLeftWheelStdDev);


    double upperRightWheelMeanHeight, upperRightWheelStdDev;
    bool upperRightWheelValid = findHeightOnWheel(upperRightWheelPos, &upperRightWheelMeanHeight,
        &upperRightWheelStdDev);


    double lowerRightWheelMeanHeight, lowerRightWheelStdDev;
    bool lowerRightWheelValid = findHeightOnWheel(lowerRightWheelPos, &lowerRightWheelMeanHeight,
        &lowerRightWheelStdDev);


    // Calculate the number of valid wheels.
    int validWheelNum = upperLeftWheelValid + upperRightWheelValid + lowerLeftWheelValid
      + lowerRightWheelValid;

    int wheelSize = metersToSteps(description_->wheelD);
    int barrelSize = metersToSteps(description_->barrelD);
    int robotSize = metersToSteps(description_->totalD);
    int robotDSize = metersToSteps(description_->robotD);

    double wheelCenterDist = description_->robotD + 2 * description_->barrelD + description_->wheelD;
    // Get the mask for the left side of the robot
    MatPtr updatedMaskPtr(new cv::Mat(robotGeometryMask_->size(), CV_64FC1));
    // Initialize the transformed map by creating a deep copy of the original.
    robotGeometryMask_->copyTo(*updatedMaskPtr);

    int validAreaTopLeftX = 0, validAreaTopLeftY = 0;
    int validAreaWidth = 0, validAreaHeight = 0;

    // TODO(Vassilis Choutas): Check return values
    // If no wheel position is know then mark the current point as unknown.
    if (validWheelNum == 0 || validWheelNum == 1 || validWheelNum == 2 || validWheelNum == 3)
    {
        return unknownArea;
    }
    /* else if (validWheelNum == 2) */
    // {
      // if (upperLeftWheelValid && upperRightWheelValid)
      // {
        // MatPtr tempMapPtr(new cv::Mat(*updatedMaskPtr,
              // cv::Rect(0, 0, updatedMaskPtr->cols - 1, wheelSize)));
        // // Get the mask for the top side of the robot.
        // bool traversabilityFlag = findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight,
            // upperRightWheelMeanHeight, wheelCenterDist);
        // // If the return value is false then it that point is not traversible.
        // if (!traversabilityFlag)
          // return occupiedArea;
        // validAreaWidth += updatedMaskPtr->cols - 1;
        // validAreaHeight += wheelSize;
        // // Get the Position of the the barrels that are located below the forward wheels.
        // cv::Point leftBarrelPos(
            // metersToSteps(center_.x - description_->robotD / 2 - description_->barrelD
              // - description_->wheelD),
            // metersToSteps(center_.y - description_->robotD / 2 - description_->barrelD));
        // cv::Point rightBarrelPos(
            // metersToSteps(center_.x + description_->robotD / 2 + description_->barrelD),
            // metersToSteps(center_.y - description_->robotD / 2 - description_->barrelD));

        // double upperLeftBarrelMeanHeight, upperLeftBarrelStdDev;
        // bool upperLeftBarrelValid = findHeightOnWheel(leftBarrelPos, &upperLeftBarrelMeanHeight,
            // &upperLeftBarrelStdDev);

        // double upperRightBarrelMeanHeight, upperRightBarrelStdDev;
        // bool upperRightBarrelValid = findHeightOnWheel(rightBarrelPos, &upperRightBarrelMeanHeight,
            // &upperRightBarrelStdDev);
        // if (upperLeftBarrelValid && upperRightBarrelValid)
        // {
          // // Update the values for the elevated mask for the barrel area.
          // tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
                // cv::Rect(0, wheelSize, updatedMaskPtr->cols - 1, barrelSize)));
          // traversabilityFlag = findElevatedTopBottomBody(tempMapPtr, upperLeftBarrelMeanHeight,
              // upperRightBarrelMeanHeight, wheelCenterDist);
          // if (!traversabilityFlag)
            // return occupiedArea;
          // validAreaHeight += barrelSize;
          // // Get the Position of the upper half of the robot body located below .
          // cv::Point leftBodyPartPos(
              // metersToSteps(center_.x - description_->robotD / 2 - description_->barrelD
                // - description_->wheelD),
              // metersToSteps(center_.y - description_->robotD / 2));
          // cv::Point rightBodyPartPos(
              // metersToSteps(center_.x + description_->robotD / 2 + description_->barrelD),
              // metersToSteps(center_.y - description_->robotD));

          // double upperLeftBodyPartMeanHeight, upperLeftBodyPartStdDev;
          // bool upperLeftBodyPartValid = findHeightOnWheel(leftBodyPartPos, &upperLeftBodyPartMeanHeight,
              // &upperLeftBodyPartStdDev);

          // double upperRightBodyPartMeanHeight, upperRightBodyPartStdDev;
          // bool upperRightBodyPartValid = findHeightOnWheel(rightBodyPartPos, &upperRightBodyPartMeanHeight,
              // &upperRightBodyPartStdDev);
          // // Check that the upper half parts of the robot are within a known area of the elevation map
          // if (upperLeftBodyPartValid && upperRightBodyPartValid)
          // {
            // tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
                  // cv::Rect(0, wheelSize + barrelSize, updatedMaskPtr->cols - 1, robotDSize / 2)));
            // traversabilityFlag = findElevatedTopBottomBody(tempMapPtr, upperRightWheelMeanHeight,
                // lowerRightWheelMeanHeight, wheelCenterDist);
            // // If the upper half of the main robot body is not located in a valid area
            // // then mark the point as occupied.
            // if (!traversabilityFlag)
              // return occupiedArea;
            // validAreaHeight += robotDSize / 2;
          // }
        // }
        // // Extract the elevation map area that corresponds to the valid part of the mask.
        // cv::Mat validElevationMapOverlap =
          // (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - barrelSize - wheelSize,
                // center_.y -  - robotDSize / 2 - barrelSize - wheelSize, validAreaWidth, validAreaHeight));
        // // Create a shallow copy of the valid region of the transformed robot height mask.
        // cv::Mat validMask = (*updatedMaskPtr)(cv::Rect(validAreaTopLeftX, validAreaTopLeftY,
              // validAreaWidth, validAreaHeight));
        // cv::Mat diff = validMask - validElevationMapOverlap;
        // cv::Mat result = diff <= 0 & validElevationMapOverlap != unknownArea;
        // if (cv::countNonZero(result) > 0)
          // return occupiedArea;
        // else
        // {
          // if (cv::countNonZero(validElevationMapOverlap == - std::numeric_limits<double>::max()) == 0)
            // return freeArea;
          // else
            // return unknownArea;
        // }
      // }  // End_if : Only top wheels on known elevation.
      // if (lowerLeftWheelValid && lowerRightWheelValid)
      // {
      // }
      // if (upperLeftWheelValid && lowerLeftWheelValid)
      // {
      // }
      // if (upperRightWheelValid && lowerRightWheelValid)
      // {
      // }

    // }
    // else if (validWheelNum == 3)
    // {
    /* } */
    // If all the wheels are on a known area of the elevation Map.
    else if (validWheelNum == 4)
    {
      MatPtr tempMapPtr(new cv::Mat(*updatedMaskPtr,
            cv::Rect(0, 0, wheelSize, updatedMaskPtr->rows)));
      // Calculate the mask for the left side of the robot.
      findElevatedLeftRight(tempMapPtr, upperLeftWheelMeanHeight, lowerLeftWheelMeanHeight, wheelCenterDist);

      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(updatedMaskPtr->cols - wheelSize, 0, wheelSize, updatedMaskPtr->rows)));
      // Get the mask for the right side of the robot.
      findElevatedLeftRight(tempMapPtr, upperRightWheelMeanHeight, lowerRightWheelMeanHeight, wheelCenterDist);

      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(0, 0, updatedMaskPtr->cols, wheelSize)));
      // Get the mask for the top side of the robot.
      findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight, wheelCenterDist);

      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(updatedMaskPtr->rows - wheelSize, 0, updatedMaskPtr->cols, wheelSize)));
      // Get the mask for the bottom side of the robot.
      findElevatedTopBottom(tempMapPtr, lowerLeftWheelMeanHeight, lowerRightWheelMeanHeight,
          wheelCenterDist);

      interpolateElevationMap(updatedMaskPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight,
          lowerRightWheelMeanHeight, lowerLeftWheelMeanHeight);
      validAreaWidth = updatedMaskPtr->cols;
      validAreaHeight = updatedMaskPtr->rows;
      // Decide about binary traversability
      // Extract the elevation map area that corresponds to the valid part of the mask.
      cv::Mat validElevationMapOverlap =
        (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - barrelSize - wheelSize,
              center_.y -  - robotDSize / 2 - barrelSize - wheelSize, validAreaWidth, validAreaHeight));
      // Create a shallow copy of the valid region of the transformed robot height mask.
      cv::Mat validMask = (*updatedMaskPtr)(cv::Rect(validAreaTopLeftX, validAreaTopLeftY,
            validAreaWidth, validAreaHeight));
      cv::Mat diff = validMask - validElevationMapOverlap;
      cv::Mat result = diff <= - description_->eps & validElevationMapOverlap != unknownArea;
      if (cv::countNonZero(result) > 0)
        return occupiedArea;
      else
        return freeArea;
    }
    return unknownArea;
  }  // End of findTraversability

  /**
   * @brief Implements the bilinear interpolation method
   * @description This method takes as input a set of four known and one unknown point and
   * applies bilinear interpolation to find the function value on the unknown Point.
  */
  double TraversabilityMask::bilinearInterpolation(const cv::Point& P, const cv::Point& Q11, const cv::Point& Q21,
      const cv::Point& Q22, const cv::Point& Q12, double fQ11, double fQ21, double fQ22, double fQ12)
  {
    double R1 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ11 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ21;
    double R2 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ12 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ22;

    return (Q22.y - P.y) / (Q22.y - Q21.y) * R1 + (P.y - Q11.y) / (Q22.y - Q11.y) * R2;
  }

  /**
   * @brief Applies barycentric Interpolation to calculate unknown values in a triangle.
   * @description Given the three vertices of the triangles and their corresponding values
   * we calculate the barycentric coordinates of the query point and using them the approximate value
   * of the function at this point
   * @param P[const cv::Point&] The query point where the interpolation will be performed.
   * @param A[const cv::Point&] A vertice of the triangle
   * @param B[const cv::Point&] The second vertice of the triangle.
   * @param C[const cv::Point&] The third vertice of the triangle.
   * @param fA[double] The value of the function at the first vertice.
   * @param fB[double] The value of the function at the second vertice.
   * @param fC[double] The value of the function at the final vertice.
   * @return double The interpolated value at the query point.
   */
  double TraversabilityMask::barycentricInterpolation(const cv::Point& P, const cv::Point& A,
      const cv::Point& B, const cv::Point&C, double fA, double fB, double fC)
  {
  }

  /**
   * @brief Performs interpolation on the elevation Map
   * @description Applies bilinear interpolation on the elevation map to fill the
   * unknown values in the middle of the robot's mask.
   * @param inputOutputMap[const MatPtr&] The input mask that will be updated to
   * produce the filled mask.
   * @param upperLeftWheelMeanHeight[double] The mean estimated height of the upper left wheel.
   * @param upperRightWheelMeanHeight[double] The mean estimated height of the upper right wheel.
   * @param lowerRightWheelMeanHeight[double] The mean estimated height of the lower right wheel.
   * @param lowerLeftWheelMeanHeight[double] The mean estimated height of the lower left wheel.
   */
  void TraversabilityMask::interpolateElevationMap(const MatPtr& inputOutputMap, double upperLeftCornerMeanHeight,
      double upperRightCornerMeanHeight, double lowerRightCornerMeanHeight,
      double lowerLeftCornerMeanHeight)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    double robotSize = metersToSteps(description_->totalD);

    // Interpolate the mask values to get the robot's local estimated elevation.
    for (int i = wheelSize + 1; i < robotSize - wheelSize; ++i)
    {
      for (int j = wheelSize + 1; j < robotSize - wheelSize; ++j)
      {
        inputOutputMap->at<double>(i, j) +=
          bilinearInterpolation(cv::Point(j, i),
              cv::Point(0, 0), cv::Point(inputOutputMap->cols - 1, 0),
              cv::Point(inputOutputMap->cols - 1, inputOutputMap->rows - 1), cv::Point(0, inputOutputMap->rows - 1),
              upperLeftCornerMeanHeight, upperRightCornerMeanHeight, lowerRightCornerMeanHeight,
              lowerLeftCornerMeanHeight);
      }
    }
  }

  void
  TraversabilityMask::setElevationMap(const boost::shared_ptr<cv::Mat const>& map)
  {
    elevationMapPtr_ = map;
  }

  /**
   * @brief Loads a robot description and create a 2D mask to describe it.
   * @description The robot's dimensions are loaded using the provided nodehandle and
   * then an approximate 2D elevation mask is created.
   * @param nh[const ros::NodeHandle& nh] A nodehandle used to access the paramater server
   * @return void
  */
  void
  TraversabilityMask::loadGeometryMask(const ros::NodeHandle& nh)
  {
    description_.reset(new RobotGeometryMaskDescription);

    if (!nh.getParam("robot_description/wheel_height", description_->wheelH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the wheels!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->wheelH = 0;
    }
    if (!nh.getParam("robot_description/barrel_height", description_->barrelH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the motor cylinder!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->barrelH = 0.06;
    }
    if (!nh.getParam("robot_description/robot_height", description_->robotH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the body of the robot!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->robotH = 0.08;
    }
    if (!nh.getParam("robot_description/wheel_width", description_->wheelD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's wheel!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->wheelD = 0.07;
    }
    if (!nh.getParam("robot_description/barrel_width", description_->barrelD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's motor cylinder!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->barrelD = 0.07;
    }
    if (!nh.getParam("robot_description/robot_width", description_->robotD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's body!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->robotD = 0.09;
    }

    description_->totalD = description_->robotD + 2 * (description_->barrelD + description_->wheelD);

    if (!nh.getParam("robot_description/epsilon", description_->eps))
    {
      ROS_WARN("[Traversability Mask]: Could not read the tolerance parameter!");
      ROS_INFO("[Traversability Mask]: Setting it to 1 cm!");
      description_->eps = 0.01;
    }
    if (!nh.getParam("robot_description/max_slope", description_->eps))
    {
      ROS_WARN("[Traversability Mask]: Could not read the maximum possible slope!");
      ROS_INFO("[Traversability Mask]: Setting it to 20 degrees!");
      description_->maxPossibleAngle = 20;
    }
    if (!nh.getParam("robot_description/cellResolution", description_->RESOLUTION))
    {
      ROS_WARN("[Traversability Mask]: Could not read the mask resolution!");
      ROS_INFO("[Traversability Mask]: Setting it to 2 cm per cell!");
      description_->RESOLUTION = 0.02;
    }
    if (!nh.getParam("elevation_difference/elevation_difference_low_free_threshold",
                     elevationDifferenceLowFreeThreshold_))
    {
      ROS_WARN("[Traversability Mask]: Could not read the elevation difference low free threshold!");
      ROS_INFO("[Traversability Mask]: Setting it to 10 centimeters!");
      elevationDifferenceLowFreeThreshold_ = 0.10;
    }
    if (!nh.getParam("elevation_difference/elevation_difference_high_free_threshold",
                     elevationDifferenceHighFreeThreshold_))
    {
      ROS_WARN("[Traversability Mask]: Could not read the elevation difference high free threshold!");
      ROS_INFO("[Traversability Mask]: Setting it to 19 centimeters!");
      elevationDifferenceHighFreeThreshold_ = 0.19;
    }
    if (!nh.getParam("elevation_difference/elevation_difference_low_occupied_threshold",
                     elevationDifferenceLowOccupiedThreshold_))
    {
      ROS_WARN("[Traversability Mask]: Could not read the elevation difference low occupied threshold!");
      ROS_INFO("[Traversability Mask]: Setting it to 14 centimeters!");
      elevationDifferenceLowOccupiedThreshold_ = 0.14;
    }
    if (!nh.getParam("elevation_difference/elevation_difference_high_occupied_threshold",
                     elevationDifferenceHighOccupiedThreshold_))
    {
      ROS_WARN("[Traversability Mask]: Could not read the elevation difference high occupied threshold!");
      ROS_INFO("[Traversability Mask]: Setting it to 23 centimeters!");
      elevationDifferenceHighOccupiedThreshold_ = 0.23;
    }

    ROS_INFO("[Traversability Mask]: Mask Loading finished successfully!");
    return;
  }

  bool
  TraversabilityMask::findElevatedLeftRight(MatPtr aLeftRight, double hForward, double hBack, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->totalD);

    // Iterate over the section of the mask for the current pair of wheels
    // and update their corresponding values.
    double slope = fabs(hForward - hBack) / d;
    double angle = asin(slope);
    // Reject slopes greater than the maximum possible angle.
    if (angle * 180 / CV_PI > description_->maxPossibleAngle)
      return false;
    // Find the wheel that is located higher
    if (hForward > hBack)
    {
      double slopeResolution = hBack;
      for (int j = wheelSize; j < aLeftRight->rows - wheelSize; ++j)
      {
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(robotSize - j, i) += j * fabs(hForward - hBack) / aLeftRight->rows;
        }
      }
    }
    else
    {
      double slopeResolution = hForward;
      for (int j = wheelSize; j < aLeftRight->rows - wheelSize; ++j)
      {
        slopeResolution += slope;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(j, i) += j * fabs(hForward - hBack) / aLeftRight->rows;
        }
      }
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedLeftRightBody(MatPtr aLeftRight, double hForward, double hBack, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    double angle;
    double slope;
    // Find the wheel that is located higher
    if (hForward > hBack)
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hForward - hBack) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;

      slope =  tan(angle);
      for (int j = 0; j < aLeftRight->rows; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hBack;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(robotSize - j, i) += val;
        }
      }
      return true;
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hBack - hForward) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aLeftRight->rows; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hForward;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(j, i) += val;
        }
      }
      return true;
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedTopBottom(MatPtr aTopBottom, double hLeft, double hRight, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->totalD);

    // Iterate over the section of the mask for the current pair of wheels
    // and update their corresponding values.
    double slope = fabs(hRight - hLeft) / d;
    double angle = asin(slope);
    // Reject slopes greater than the maximum possible angle.
    if (angle * 180 / CV_PI > description_->maxPossibleAngle)
      return false;
    // Find the wheel that is located higher
    if (hLeft < hRight)
    {
      double slopeResolution = hLeft;
      for (int j = wheelSize; j < aTopBottom->cols - wheelSize; ++j)
      {
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, j) += j * fabs(hRight - hLeft) / aTopBottom->cols;
        }
      }
    }
    else
    {
      double slopeResolution = hRight;
      for (int j = wheelSize; j < aTopBottom->cols - wheelSize; ++j)
      {
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, robotSize - j) += j * fabs(hRight - hLeft) / aTopBottom->cols;
        }
      }
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedTopBottomBody(MatPtr aTopBottom, double hLeft, double hRight, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    double angle;
    double slope;
    // Find the wheel that is located higher
    if (hLeft < hRight)
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hRight - hLeft) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aTopBottom->cols; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hLeft;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, j) += val;
        }
      }
      return true;
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hLeft - hRight) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aTopBottom->cols; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hRight;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, robotSize - j) += val;
        }
      }
      return true;
    }

    return true;
  }

  bool
  TraversabilityMask::findHeightOnWheel(const cv::Point& wheelPos, double* meanHeight, double* stdDevHeight)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    MatPtr wheelElevation( new cv::Mat(wheelSize, wheelSize, CV_64FC1, cv::Scalar(0)));
    bool known = cropToWheel(wheelPos, wheelElevation);
    if (!known)
      return false;

    cv::Mat validityMask = *wheelElevation == - std::numeric_limits<double>::max();
    if (cv::countNonZero(validityMask) > 0)
      return false;

    cv::Scalar mean, std_dev;
    cv::meanStdDev(*wheelElevation, mean, std_dev);
    *meanHeight = mean[0];
    *stdDevHeight = std_dev[0];

    return true;
  }

  /**
   * @brief Creates a mask for the given wheel on the elevation map.
  */
  bool
  TraversabilityMask::cropToWheel(const cv::Point& wheelPos, const MatPtr& wheel)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    // Copy the region of the elevation map that corresponds to the current wheel.
    cv::Mat roi = (*elevationMapPtr_)(cv::Rect(wheelPos.x, wheelPos.x, wheelSize, wheelSize));

    roi.copyTo(*wheel);
    return true;
  }

  cv::Mat TraversabilityMask::findRowTraversability(const cv::Mat& inputImage)
  {
    cv::Mat nonTraversableRowPoints = cv::Mat(inputImage.rows - 1, inputImage.cols, CV_8UC1);
    for (int jj = 0; jj < nonTraversableRowPoints.cols; ++jj)
    {
      for (int ii = 0; ii < nonTraversableRowPoints.rows; ++ii)
      {
        if (inputImage.at<double>(ii, jj) == -std::numeric_limits<double>::max() ||
            inputImage.at<double>(ii + 1, jj) == -std::numeric_limits<double>::max())
        {
          nonTraversableRowPoints.at<uchar>(ii, jj) = unknownArea;
        }
        else
        {
          double grad = fabs(inputImage.at<double>(ii + 1, jj) -
                             inputImage.at<double>(ii, jj));
          /*
          if (detectRamps_)
          {
            if (grad <= elevationDifferenceLowFreeThreshold_ ||
                (grad > elevationDifferenceLowOccupiedThreshold_ &&
                grad <= elevationDifferenceHighFreeThreshold_))
            {
              nonTraversableRowPoints.at<uchar>(ii, jj) = freeArea;
            }
            else if (grad > elevationDifferenceHighFreeThreshold_ &&
                    grad <= elevationDifferenceHighOccupiedThreshold_)
            {
              nonTraversableRowPoints.at<uchar>(ii, jj) = rampArea;
            }
            else
            {
              nonTraversableRowPoints.at<uchar>(ii, jj) = occupiedArea;
            }
          } */
          //if ()
          //{
            if (grad <= elevationDifferenceLowFreeThreshold_)
            {
              nonTraversableRowPoints.at<uchar>(ii, jj) = freeArea;
            }
            else
            {
              nonTraversableRowPoints.at<uchar>(ii, jj) = occupiedArea;
            }
        //  }
        }
      }
    }
    return nonTraversableRowPoints;
  }

  cv::Mat TraversabilityMask::findColTraversability(const cv::Mat& inputImage)
  {
    cv::Mat nonTraversableColPoints = cv::Mat(inputImage.rows, inputImage.cols - 1, CV_8UC1);
    for (int ii = 0; ii < nonTraversableColPoints.rows; ++ii)
    {
      for (int jj = 0; jj < nonTraversableColPoints.cols; ++jj)
      {
        if (inputImage.at<double>(ii, jj) == -std::numeric_limits<double>::max() ||
            inputImage.at<double>(ii, jj + 1) == -std::numeric_limits<double>::max())
        {
          nonTraversableColPoints.at<uchar>(ii, jj) = unknownArea;
        }
        else
        {
          double grad = fabs(inputImage.at<double>(ii, jj + 1) -
                             inputImage.at<double>(ii, jj));
   /*       if (detectRamps_)
          {
            if (grad <= elevationDifferenceLowFreeThreshold_ ||
                (grad > elevationDifferenceLowOccupiedThreshold_ &&
                grad <= elevationDifferenceHighFreeThreshold_))
            {
              nonTraversableColPoints.at<uchar>(ii, jj) = freeArea;
            }
            else if (grad > elevationDifferenceHighFreeThreshold_ &&
                    grad <= elevationDifferenceHighOccupiedThreshold_)
            {
              nonTraversableColPoints.at<uchar>(ii, jj) = rampArea;
            }
            else
            {
              nonTraversableColPoints.at<uchar>(ii, jj) = occupiedArea;
            }
          } */
          //else
          //{
            if (grad <= elevationDifferenceLowFreeThreshold_)
            {
              nonTraversableColPoints.at<uchar>(ii, jj) = freeArea;
            }
            else
            {
              nonTraversableColPoints.at<uchar>(ii, jj) = occupiedArea;
            }
          //}
        }
      }
    }
    return nonTraversableColPoints;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
