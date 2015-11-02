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
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H

#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pandora_vision_obstacle/hard_obstacle_detection/RobotGeometryMaskDescription.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  /**
   * @class TraversabilityMask TODO
   */
  class TraversabilityMask
  {
   public:
    typedef boost::shared_ptr<TraversabilityMask> Ptr;
    typedef boost::shared_ptr<TraversabilityMask const> ConstPtr;

    typedef boost::shared_ptr<cv::Mat> MatPtr;
    typedef boost::shared_ptr<cv::Mat const> MatConstPtr;

   public:
    TraversabilityMask();

    explicit TraversabilityMask(const RobotGeometryMaskDescriptionPtr& descriptionPtr);

    virtual ~TraversabilityMask();

    /**
     * Find trinary traversability of point in elevation map
     * Values can be free, obstacle or unknown
     */
    int8_t
    findTraversability(const cv::Point& center);

    cv::Mat findRowTraversability(const cv::Mat& inputImage);

    cv::Mat findColTraversability(const cv::Mat& inputImage);

    /**
     * Sets elevation map to be used
     */
    void
    setElevationMap(const boost::shared_ptr<cv::Mat const>& map);

    /**
     * load robot's geometry mask from yaml (matrix A)
     */
    void
    loadGeometryMask(const ros::NodeHandle& nh);

    double bilinearInterpolation(const cv::Point& P, const cv::Point& Q11, const cv::Point& Q21,
      const cv::Point& Q22, const cv::Point& Q12, double fQ11, double fQ21, double fQ22, double fQ12);

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
    double barycentricInterpolation(const cv::Point& P, const cv::Point& A,
      const cv::Point& B, const cv::Point&C, double fA, double fB, double fC);


    void setRobotMaskDescriptionPtr(const RobotGeometryMaskDescriptionPtr& robotGeometryMaskDescriptionPtr)
    {
      description_ = robotGeometryMaskDescriptionPtr;
    }


    void createMaskFromDesc();
    /**
     * @brief Creates the Robot Height Mask
     * @description Creates the height mask for the robot according to the description file.
     * @param inputOutputMap[const MatPtr&] A pointer to the matrix that contains the mask.
     * @param description[RobotGeometryMaskDescriptionPtr] A pointer to the parameter structure
     * that approximately describes the robot.
     * @return void
     */
    void createMaskFromDesc(const RobotGeometryMaskDescriptionPtr& description);

    MatConstPtr getRobotMaskPtr()
    {
      return robotGeometryMask_;
    }

    void setElevationDifferenceHighFreeThreshold(double elevationDifferenceHighFreeThreshold)
    {
      elevationDifferenceHighFreeThreshold_ = elevationDifferenceHighFreeThreshold;
    }

    void setElevationDifferenceLowFreeThreshold(double elevationDifferenceLowFreeThreshold)
    {
      elevationDifferenceLowFreeThreshold_ = elevationDifferenceLowFreeThreshold;
    }

    void setElevationDifferenceHighOccupiedThreshold(double elevationDifferenceHighOccupiedThreshold)
    {
      elevationDifferenceHighOccupiedThreshold_ = elevationDifferenceHighOccupiedThreshold;
    }

    void setElevationDifferenceLowOccupiedThreshold(double elevationDifferenceLowOccupiedThreshold)
    {
      elevationDifferenceLowOccupiedThreshold_ = elevationDifferenceLowOccupiedThreshold;
    }

    void setDetectRamps(bool detectRamps)
    {
      detectRamps_ = detectRamps;
    }

   private:
    /**
     * Finds B matrices, input is respective A matrix, heights and wheel
     * distance. Should compare wheel heights and make the proper
     * transformations
     */
    bool
    findElevatedLeftRight(MatPtr aLeftRight, double hForward, double hBack, double d);
    bool
    findElevatedTopBottom(MatPtr aTopBottom, double hLeft, double hRight, double d);

    bool
    findElevatedLeftRightBody(MatPtr aLeftRight, double hForward, double hBack, double d);

    bool
    findElevatedTopBottomBody(MatPtr aTopBottom, double hLeft, double hRight, double d);

    /**
     * @brief: find mean and stdev of height on wheel, false if wheel has
     * unknown values underneath it
     */
    bool
    findHeightOnWheel(const cv::Point& wheelPos, double* meanHeight, double* stdDevHeight);

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
    void interpolateElevationMap(const MatPtr& inputOutputMap, double upperLeftCornerMeanHeight,
      double upperRightCornerMeanHeight, double lowerRightCornerMeanHeight,
      double lowerLeftCornerMeanHeight);

    /**
     * Fills up wheel with a mask which corresponds to the wheel, performs a
     * check whether wheel is on unknown terrain. Returns false if it is
     */
    bool
    cropToWheel(const cv::Point& wheelPos, const MatPtr& wheel);

    /**
     * Crops elevationMap_ to Ar according to center point and geometric params
     */
    MatPtr
    cropToRight();

    /**
     * Crops elevationMap_ to Al according to center point and geometric params
     */
    MatPtr
    cropToLeft();

    /**
     * Crops elevationMap_ to At according to center point and geometric params
     */
    MatPtr
    cropToTop();

    /**
     * Crops elevationMap_ to Ab according to center point and geometric params
     */
    MatPtr
    cropToBottom();

    inline int metersToSteps(double meters)
    {
      return static_cast<int>(meters / description_->RESOLUTION);
    }

   private:
    /// This is A matrix
    MatPtr robotGeometryMask_;
    /// This is geometrical description of robot's base
    RobotGeometryMaskDescriptionPtr description_;
    /// Local elevation map created
    MatConstPtr elevationMapPtr_;

    double elevationDifferenceLowOccupiedThreshold_;
    double elevationDifferenceHighOccupiedThreshold_;
    double elevationDifferenceLowFreeThreshold_;
    double elevationDifferenceHighFreeThreshold_;

    bool detectRamps_;

    /// Center of matrix A
    cv::Point center_;

    friend class TraversabilityMaskTest;
  };

  const int8_t occupiedArea = 70;
  const int8_t freeArea = 0;
  const int8_t unknownArea = 51;
  const int8_t rampArea = 50;

  typedef TraversabilityMask::Ptr TraversabilityMaskPtr;
  typedef TraversabilityMask::ConstPtr TraversabilityMaskConstPtr;
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_TRAVERSABILITY_MASK_H
