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

#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <utility>
#include "pandora_vision_obstacle/barrel_detection/barrel_detector.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  BarrelDetector::BarrelDetector(const std::string& name, const ros::NodeHandle& nh)
  {
    nodeName_ = name;
  }

  BarrelDetector::BarrelDetector() {}

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
  void BarrelDetector::getSymmetryObject(
      const cv::Mat& inputImage,
      cv::Rect* roi,
      cv::Point* symmetricStartPoint,
      cv::Point* symmetricEndPoint)
  {
    cv::Point accumIndex(-1, -1);

    if (!inputImage.data)
      return;

    /* Determine the shape of Hough accumulationmatrix */
    float rhoDivs = hypotf(inputImage.rows, inputImage.cols) + 1;
    float thetaDivs = 180.0;

    FastSymmetryDetector detector(inputImage.size(), cv::Size(rhoDivs, thetaDivs), 1);

    cv::Rect region(0, inputImage.rows, thetaDivs * 2.0, rhoDivs * 0.5);
    cv::Mat temp, edge, depth8UC3;

    int cannyThresh1 = BarrelDetection::fsd_canny_thresh_1;
    int cannyThresh2 = BarrelDetection::fsd_canny_thresh_2;
    int minPairDist  = BarrelDetection::fsd_min_pair_dist;
    int maxPairDist  = BarrelDetection::fsd_max_pair_dist;
    int noOfPeaks    = BarrelDetection::fsd_no_of_peaks;

    temp = inputImage.clone();

    double minVal, maxVal;
    cv::minMaxLoc(temp, &minVal, &maxVal);
    cv::Mat(temp - minVal).convertTo(edge, CV_8UC1, 255.0 / (maxVal - minVal));
    edge = cv::abs(edge);
    edge.copyTo(depth8UC3);
    // temp.convertTo(depth8UC3, CV_8UC3, 255);
    /* Find the edges */
    if (edge.channels() == 3)
      cvtColor(edge, edge, CV_BGR2GRAY);
    cv::Canny(edge, edge, cannyThresh1, cannyThresh2);

    /* Vote for the accumulation matrix */
    detector.vote(edge, minPairDist, maxPairDist);

    /* Draw the symmetrical line */
    std::vector<std::pair<cv::Point, cv::Point> > result = detector.getResult(noOfPeaks);
    float maxDist;
    float maxY;
    float minY;
    detector.getMaxDistance(&maxDist);
    detector.getYCoords(&maxY, &minY);

    for (int i = 0; i < result.size(); i ++)
    {
      // float len1 = std::sqrt(result[i].first.x * result[i].first.x + result[i].first.y * result[i].first.y);
      // float len2 = std::sqrt(result[i].second.x * result[i].second.x + result[i].second.y * result[i].second.y);

      // float dot = result[i].first.x * result[i].second.x + result[i].first.y * result[i].second.y;

      // float a = dot / (len1 * len2);

      // float angle;
      // if (a >= 1.0)
      //   angle = 0.0;
      // else if (a <= -1.0)
      //   angle = 3.14;
      // else
      //   angle = std::acos(a); //  0..PI
      // angle = angle * 180 / 3.14;
      cv::Point minPoint, maxPoint;
      minPoint.x = std::min(result[i].first.x, result[i].second.x);
      minPoint.y = std::min(result[i].first.y, result[i].second.y);
      maxPoint.x = std::max(result[i].first.x, result[i].second.x);
      maxPoint.y = std::max(result[i].first.y, result[i].second.y);
      float startROIX = minPoint.x - maxDist / 2;
      float startROIY = minPoint.y;
      float widthROI = maxDist;
      float heightROI = maxPoint.y - minPoint.y;
      if (startROIX < 0)
        startROIX = 0;
      if (startROIY < 0)
        startROIY = 0;
      if (startROIX + widthROI > inputImage.cols)
        widthROI = inputImage.cols - startROIX;
      if (startROIY + heightROI > inputImage.rows)
        heightROI = inputImage.rows - startROIY;
      (*roi) = cv::Rect(
          startROIX,
          startROIY,
          widthROI,
          heightROI);
      (*symmetricStartPoint) = result[i].second;
      (*symmetricEndPoint) = result[i].first;


      if (BarrelDetection::show_respective_barrel)
      {
        // cv::line(depth8UC3, result[i].first, result[i].second, cv::Scalar(0, 0, 255), 2);
        // cv::rectangle(depth8UC3,
        //     cv::Point(result[i].second.x - maxDist / 2, result[i].second.y),
        //     cv::Point(result[i].first.x + maxDist / 2, result[i].first.y),
        //     cv::Scalar(255, 0, 0),
        //     2);
        //  cv::line(temp, s3, s2, cv::Scalar(0, 255, 0), 2);

        /* Visualize the Hough accum matrix */
        // cv::Mat accum = detector.getAccumulationMatrix();
        // accum.convertTo(accum, CV_8UC3);
        // cv::applyColorMap(accum, accum, cv::COLORMAP_JET);
        // cv::resize(accum, accum, cv::Size(), 2.0, 0.5);

        // /* Draw lines based on cursor position */
        // if (accumIndex.x != -1 && accumIndex.y != -1)
        // {
        //   std::pair<cv::Point, cv::Point> pointPair = detector.getLine(accumIndex.y, accumIndex.x);
        //   cv::line(depth8UC3, pointPair.first, pointPair.second, CV_RGB(0, 255, 0), 2);
        // }

        /* Show the original and edge images */
        debugShow(depth8UC3, (*roi), 1);
        //  cv::Mat appended = cv::Mat::zeros(depth8UC3.rows + accum.rows, depth8UC3.cols * 2, CV_8UC3);
        //  depth8UC3.copyTo(cv::Mat(appended, cv::Rect(0, 0, depth8UC3.cols, depth8UC3.rows)));
        //  cv::cvtColor(edge, cv::Mat(appended, cv::Rect(depth8UC3.cols, 0, edge.cols, edge.rows)), CV_GRAY2BGR);
        //  accum.copyTo(cv::Mat(appended, Rect(0, depth8UC3.rows, accum.cols, accum.rows)));
        //  cv::imshow("Candidate Barrel", appended);
      }
    }
  }

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
  bool BarrelDetector::validateRoi(
      const cv::Mat& rgbImage,
      const cv::Mat& depthImage,
      const cv::Rect& rectRoi,
      const cv::Point& symmetricStartPoint,
      const cv::Point& symmetricEndPoint)
  {
    //  Validate based on variance in RGB ROI
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::Mat roiRgb = rgbImage(rectRoi);
    cv::Mat roiDepth = depthImage(rectRoi);
    cv::meanStdDev(roiRgb, mean, stddev);
    if (stddev.val[0] > BarrelDetection::roi_variance_thresh)
    {
      // (*valid) = false;
      return false;
    }

    //  Validate that through the symmetry line we have almost
    //  constant depth
    cv::LineIterator itSym(depthImage, symmetricStartPoint, symmetricEndPoint, 8);
    std::vector<cv::Vec3b> bufSym(itSym.count);
    std::vector<cv::Point> pointsSym(itSym.count);
    float sumDiffs = 0.0;

    for (int linePoint = 0; linePoint < itSym.count; linePoint ++, ++itSym)
    {
      bufSym[linePoint] = (const cv::Vec3b)* itSym;
      pointsSym[linePoint] = itSym.pos();
      if (depthImage.at<float>(pointsSym[linePoint].y, pointsSym[linePoint].x) != 0.0)
        sumDiffs +=
          std::abs(depthImage.at<float>(pointsSym[linePoint].y, pointsSym[linePoint].x)
              - depthImage.at<float>(pointsSym[linePoint - 1].y, pointsSym[linePoint - 1].x));
    }

    if (sumDiffs > BarrelDetection::symmetry_line_depth_difference_thresh)
    {
      // (*valid) = false;
      return false;
    }
    cv::Point2f slope;
    int length = std::abs(rectRoi.width / 2);
    slope.x = symmetricEndPoint.x - symmetricStartPoint.x;
    slope.y = symmetricEndPoint.y - symmetricStartPoint.y;
    float magnitude = std::sqrt(slope.x * slope.x + slope.y * slope.y);
    slope.x /= magnitude;
    slope.y /= magnitude;
    //  Rotate vector 90 degrees clockwisely
    float temp1 = slope.x;
    slope.x = -slope.y;
    slope.y = temp1;
    //  A point on the symmetry line
    cv::Point s1 =
      cv::Point(rectRoi.x + rectRoi.width / 2,
          rectRoi.y + rectRoi.height / 2);
    // ROS_INFO("%f, %f", slope.x, slope.y);
    cv::Point s2;
    s2.x = s1.x + slope.x * length;
    s2.y = s1.y + slope.y * length;

    cv::Point s3;
    s3.x = s1.x - slope.x * length;
    s3.y = s1.y - slope.y * length;

    //  In order to calculate circularity place points of the perpendicular
    //  line into a vector

    cv::LineIterator it(depthImage, s3, s2, 8);
    std::vector<cv::Vec3b> buf(it.count);
    std::vector<cv::Point> points(it.count);
    std::vector<float> differentialPoints(it.count);
    float maxDifferential = std::numeric_limits<float>::min();

    int firstBorder = it.count > 10 ? 10 : it.count;
    int secondBorder = it.count - firstBorder > 10 ? it.count - 10 : it.count;
    //  Ignore values at the border
    for (int linePoint = 0; linePoint < firstBorder; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      differentialPoints[linePoint] = 0.0;
    }


    // Used to spot extreme values, in order to remove them from calculations
    float avgDifferentials = 0.0;
    for (int linePoint = firstBorder; linePoint < secondBorder; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      if (depthImage.at<float>(points[linePoint].y, points[linePoint].x) != 0.0
          && std::abs(
            depthImage.at<float>(points[linePoint].y, points[linePoint].x) -
            depthImage.at<float>(points[linePoint - 1].y, points[linePoint - 1].x)
            - avgDifferentials) <= 0.3)
      {
        differentialPoints[linePoint] =
          depthImage.at<float>(points[linePoint].y, points[linePoint].x)
          - depthImage.at<float>(points[linePoint - 1].y, points[linePoint - 1].x);
        avgDifferentials =
          (avgDifferentials * (linePoint - 1 + 1 - firstBorder)
           + differentialPoints[linePoint]) / (linePoint + 1 - firstBorder);
      }
      else
        differentialPoints[linePoint] = 0;
      if (std::abs(differentialPoints[linePoint]) > maxDifferential)
        maxDifferential = std::abs(differentialPoints[linePoint]);
    }

    for (int linePoint = secondBorder; linePoint < it.count; linePoint ++, ++it)
    {
      buf[linePoint] = (const cv::Vec3b)* it;
      points[linePoint] = it.pos();
      differentialPoints[linePoint] = 0.0;
    }

    //  calculate differentiation of depth values, through the perpendicular
    //  line(s) for the left side of the barrel and the right side of the
    //  barrel. Calculate avg of differentiation for left and right side
    //  separately.
    int leftRightBorder = static_cast<int>(points.size() / 2);
    float sumLeftLine = 0.0;

    for (int leftLinePoint = 0; leftLinePoint <= leftRightBorder; leftLinePoint ++)
    {
      sumLeftLine += differentialPoints[leftLinePoint];
    }

    float avgLeftLinePoint = 0.0;
    if (leftRightBorder + 1 > 0)
      avgLeftLinePoint = sumLeftLine / (leftRightBorder + 1);
    else
      avgLeftLinePoint = 0.0;
    // avgLeftLinePoint = sumLeftLine;


    float sumRightLine = 0.0;
    for (int rightLinePoint = leftRightBorder + 1; rightLinePoint < points.size(); rightLinePoint ++)
    {
      sumRightLine += differentialPoints[rightLinePoint];
    }

    float avgRightLinePoint = 0.0;
    if (points.size() - leftRightBorder > 0)
      avgRightLinePoint = sumRightLine / (points.size() - leftRightBorder);
    else
      avgRightLinePoint = 0.0;
    // avgRightLinePoint = sumRightLine;

    //  We expect that starting from the left side of the barrel,
    //  we have a decreasing depth up to the peak of the barrel
    //  (negative differential avg), and then increasing depth.
    if (avgLeftLinePoint >= 0 || avgRightLinePoint <= 0)
    {
      // (*valid) = false;
      return false;
    }

    //  We must have a symmetry between the differential avgs of the
    //  left and right sides of the barrel.
    if (std::abs(avgLeftLinePoint + avgRightLinePoint) > BarrelDetection::differential_depth_unsymmetry_thresh)
    {
      // (*valid) = false;
      return false;
    }

    // Check if some of the points belong to a circle curve

    // Firstly get an approximation of the center of the curve, thus
    // approximate radius

    // s1 is the coordinates of the center (this point is located on the symmetry line)
    // so approximate radius as the difference of this point (on a barrel this point
    // will have the smallest depth, same as all points on the symmetry line) and the
    // point of the biggest depth inside the roi, hopefully it will be a point on the
    // wall, but there is no problem if it is on the barrel.
    float maxDepth = std::numeric_limits<float>::min();
    for (int row = rectRoi.y; row < rectRoi.y + rectRoi.height; row ++)
      for (int col = rectRoi.x; col < rectRoi.x + rectRoi.width; col ++)
        if (depthImage.at<float>(row, col) != 0.0)
          if (depthImage.at<float>(row, col) > maxDepth)
            maxDepth = depthImage.at<float>(row, col);
    float radius = 0.0;
    if (depthImage.at<float>(s1.y, s1.x) != 0.0)
      radius =
        std::sqrt(std::pow(maxDepth - depthImage.at<float>(s1.y, s1.x), 2));
    else
      return false;
    int sumNonZero = 0;
    int sumOnCurve = 0;
    for (int linePoint = leftRightBorder; linePoint >= firstBorder; linePoint --)
      if (depthImage.at<float>(points[linePoint].y, points[linePoint].x) != 0.0)
      {
        sumNonZero++;
        float circleLeftEq =
          std::pow(
              std::sqrt(std::pow(points[linePoint].x, 2) + std::pow(points[linePoint].y, 2))
              - std::sqrt(std::pow(s1.x, 2) + std::pow(s1.y, 2)), 2) +
          std::pow(depthImage.at<float>(points[linePoint].y, points[linePoint].x) - maxDepth, 2);
        if (std::abs(circleLeftEq - std::pow(radius, 2)) < BarrelDetection::curve_approximation_max_epsilon)
          sumOnCurve++;
      }
    for (int linePoint = leftRightBorder; linePoint < secondBorder; linePoint ++)
      if (depthImage.at<float>(points[linePoint].y, points[linePoint].x) != 0.0)
      {
        sumNonZero++;
        float circleLeftEq =
          std::pow(points[linePoint].x - s1.x, 2) + std::pow(points[linePoint].y - s1.y, 2) +
          std::pow(depthImage.at<float>(points[linePoint].y, points[linePoint].x) - maxDepth, 2);
        if (std::abs(circleLeftEq - std::pow(radius, 2)) < BarrelDetection::curve_approximation_max_epsilon)
          sumOnCurve++;
      }
    float curveProbability = static_cast<float>(sumOnCurve) / static_cast<float>(sumNonZero);
    if (curveProbability < BarrelDetection::min_circle_overlapping)
    {
      // (*valid) = false;
      return false;
    }

    // Eliminate corners with Harris Corner Detector
    int thresh = 100;
    cv::Mat dst, dst_norm, dst_norm_scaled, gray;
    // cv::cvtColor(rgbImage, gray, CV_BGR2GRAY);
    // dst = cv::Mat::zeros(rgbImage.size(), CV_32FC1);
    depthImage.copyTo(gray);
    dst = cv::Mat::zeros(depthImage.size(), CV_32FC1);

    // Detecting corners
    cv::cornerHarris(gray, dst, 7, 5, 0.05, cv::BORDER_DEFAULT);

    // Normalizing
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);
    if (cv::mean(dst_norm_scaled).val[0] > BarrelDetection::max_corner_thresh)
    {
      // (*valid) = false;
      return false;
    }

    // Validation based on specific barrels' color
    if (BarrelDetection::color_validation)
    {
      cv::Mat rgbTemp;
      rgbImage.copyTo(rgbTemp);
      cv::Mat hsvImage = rgbTemp(rectRoi);
      cv::Mat binary, binaryTemp;
      // convert RGB image into HSV image
      cv::cvtColor(hsvImage, hsvImage, CV_BGR2HSV);
      std::vector<cv::Mat> hsvChannels;
      cv::split(hsvImage, hsvChannels);
      cv::Mat hImage = hsvChannels[0];

      int iLowH = BarrelDetection::hue_lowest_thresh, iHighH = BarrelDetection::hue_highest_thresh;
      int iLowS = BarrelDetection::saturation_lowest_thresh, iHighS = BarrelDetection::saturation_highest_thresh;
      int iLowV = BarrelDetection::value_lowest_thresh, iHighV = BarrelDetection::value_highest_thresh;
      // cv::inRange(hImage, iLowH, iHighH, binary);
      cv::inRange(
          hsvImage, 
          cv::Scalar(iLowH, iLowS, iLowV), 
          cv::Scalar(iHighH, iHighS, iHighV), 
          binary);
      // For RED define one second threshold
      if (BarrelDetection::color_selection_R_1_G_2_B_3 == 1)
      {
        iLowH = 0;
        iHighH = 3;
        // cv::inRange(hImage, iLowH, iHighH, binaryTemp);
        cv::inRange(
            hsvImage, 
            cv::Scalar(iLowH, iLowS, iLowV), 
            cv::Scalar(iHighH, iHighS, iHighV), 
            binaryTemp);
        binary += binaryTemp;
      }
    

      int whitesCounter = cv::countNonZero(binary);
      float whitesOverlap =
        static_cast<float>(whitesCounter) / static_cast<float>(rectRoi.area());

      if (whitesOverlap < BarrelDetection::specific_color_min_overlap)
      {
        // (*valid) = false;
        return false;
      }

      // cv::inRange(hsvImage, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), binary);  // red*/
    }

    return true;
  }

  float BarrelDetector::findDepthDistance(const cv::Mat& depthImage,
      const cv::Rect& roi)
  {
    float depth = 0.0f;

    for (size_t row = 1; row < roi.height; row ++)
    {
      for (size_t col = 1; col < roi.width; col ++)
      {
        /// Find depth distance
        depth += depthImage.at<float>(row, col);
      }
    }
    depth /= static_cast<float>(roi.area());
    return depth;
  }

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
  void BarrelDetector::debugShow(
      const cv::Mat& image,
      const cv::Rect& rectRoi,
      int visualizationMode)
  {
    cv::Mat visualizableFrame;
    image.copyTo(visualizableFrame);

    cv::rectangle(visualizableFrame,
        cv::Point(rectRoi.x, rectRoi.y),
        cv::Point(rectRoi.x + rectRoi.width, rectRoi.y + rectRoi.height),
        cv::Scalar(255, 0, 0),
        2);
    if (visualizationMode == 1 && BarrelDetection::show_respective_barrel)
      cv::imshow("Respective Barrel", visualizableFrame);
    else
      cv::imshow("Valid Barrel", visualizableFrame);
    cv::waitKey(10);
  }

  /**
   * @brief Detect barrel by performing a number of
   * operations to the rgb and depth image
   * @param rgbImage [const cv::Mat&] The input rgb image
   * @param depthImage [const cv::Mat&] The input depth image
   * @return [std::vector<POIPtr>] The Point Of Interest that
   * includes the barrel
   **/
  std::vector<POIPtr> BarrelDetector::detectBarrel(const cv::Mat& rgbImage,
      const cv::Mat& depthImage)
  {
    cv::Rect roi;
    cv::Point symmetricStartPoint;
    cv::Point symmetricEndPoint;
    getSymmetryObject(depthImage, &roi, &symmetricStartPoint, &symmetricEndPoint);
    bool valid;
    // Find the depth distance of the soft obstacle
    float depthDistance;
    std::vector<POIPtr> pois;
    if (roi.area() > 0.1)
    {
      valid = validateRoi(rgbImage, depthImage,
          roi, symmetricStartPoint, symmetricEndPoint);
      if (valid)
      {
        if (BarrelDetection::show_valid_barrel)
          debugShow(rgbImage, roi, 2);
        depthDistance = findDepthDistance(depthImage, roi);
        ObstaclePOIPtr poi(new ObstaclePOI);
        poi->setPoint(cv::Point((roi.x + roi.width / 2),
              (roi.y + roi.height / 2)));

        poi->setProbability(1.0);
        poi->setType(pandora_vision_msgs::ObstacleAlert::BARREL);

        poi->setDepth(depthDistance);
        pois.push_back(poi);
        ROS_INFO("barrel found");
      }
      else
      {
        if (BarrelDetection::show_valid_barrel)
          debugShow(rgbImage, cv::Rect(0, 0, 0, 0), 2);
      }
    }

    return pois;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

