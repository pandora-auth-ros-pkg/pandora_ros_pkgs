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
 * Authors: Angelos Triantafyllidis<aggelostriadafillidis@gmail.com>,
 * Manos Tsardoulias
 *********************************************************************/

#include "depth_node/utils/image_matching.h"


/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace depth
{

  /**
    @brief Converts Conveyors which represent each hole and include all
    necessary information about it to match with rgb and depth images.
    After transformation checks if the point of interest is outside of rgb image
    borders. If so that hole will be rejected.
    @param[out] conveyor [HolesConveyor*]
    The input conveyor to be converted and sent back as output.
    @param[in] x_th [double] The x coordinate of thermal image in rgb image
    @param[in] y_th [double] The y coordinate of thermal image in rgb image
    @param[in] c_x [double] The c factor on x-axis.
    Found as matchedthermal_x/rgb_x
    @param[in] c_y [double] The c factor on y-axis.
    Found as matchedthermal_y/rgb_y
    @param[in] angle [double] The rotation of thermal image in rads.
    Clockwise is positive. Point of reference is he center of thermal image.
    @return void
   **/
  void ImageMatching::conveyorMatching(HolesConveyor* conveyor, double x_th,
    double y_th, double c_x, double c_y, double angle)
  {
    // Counter of holes deleted
    unsigned int deleted = 0 , i = 0;

    for (; (i - deleted) < conveyor->size(); i++)
    {
      bool skip = false;
      cv::Point2f keypoint;
      keypoint.x = conveyor->holes[i - deleted].keypoint.pt.x;
      keypoint.y = conveyor->holes[i - deleted].keypoint.pt.y;

      // Match the keypoint of each Hole found
      keypoint = matchingFunction(
        keypoint, x_th, y_th, c_x, c_y, angle);

      // If the keypoint is out of or on rbg image borders reject
      // that hole immediately.
      if (keypoint.x < 1 || keypoint.x > (Parameters::Image::WIDTH - 1)
        || keypoint.y < 1 || keypoint.y > (Parameters::Image::HEIGHT -1))
      {
        conveyor->holes.erase(conveyor->holes.begin() + i - deleted);
        deleted++;
      }
      else
      {
        // Continue the process
        conveyor->holes[i - deleted].keypoint.pt.x = keypoint.x;
        conveyor->holes[i - deleted].keypoint.pt.y = keypoint.y;

        // Match the bounding box and check if it is outside rgb image, if so
        // reject that hole. Bounding box check is enough, we dont need to check
        // outlines, because they reside entirely inside it.
        for (unsigned int j = 0; j < conveyor->holes[i - deleted].rectangle.size(); j++)
        {
          conveyor->holes[i - deleted].rectangle[j] = matchingFunction(
            conveyor->holes[i - deleted].rectangle[j], x_th, y_th, c_x, c_y, angle);

          if (conveyor->holes[i - deleted].rectangle[j].x < 0
            || conveyor->holes[i - deleted].rectangle[j].x > (Parameters::Image::WIDTH)
            || conveyor->holes[i - deleted].rectangle[j].y < 0
            || conveyor->holes[i - deleted].rectangle[j].y > (Parameters::Image::HEIGHT))
          {
            conveyor->holes.erase(conveyor->holes.begin() + i - deleted);
            skip = true;
            deleted++;
            break;
          }
        }

        // If hole rejected skip next process step
        if (!skip)
        {
          // Match the blobs outline points
          for (unsigned int o = 0; o < conveyor->holes[i - deleted].outline.size(); o++)
          {
            conveyor->holes[i - deleted].outline[o] = matchingFunction(
              conveyor->holes[i - deleted].outline[o], x_th, y_th, c_x, c_y, angle);
          }


          // The easiest and most efficient way to obtain the new outline is to
          // apply the raycast algorithm
          // cv::Mat canvas = cv::Mat::zeros(Parameters::Image::HEIGHT,
            // Parameters::Image::WIDTH, CV_8UC1);

          // unsigned char* ptr = canvas.ptr();
          // for (unsigned int a = 0; a < conveyor->holes[i].outline.size(); a++)
          // {
            // unsigned int ind =
              // conveyor->holes[i].outline[a].x + canvas.cols * conveyor->holes[i].outline[a].y;
            // ptr[ind] = 255;
          // }

           // float area = 0.0;
           // OutlineDiscovery::raycastKeypoint(conveyor->holes[i].keypoint,
           // &canvas,
           // Parameters::Outline::raycast_keypoint_partitions,
           // false,
           // &conveyor->holes[i].outline,
           // &area);

          // Second method to connect the new outlines
        }
      }
    }
    // Put the outline points in order for each hole
    outlinePointsInOrder(conveyor);

    // Connect the points in order to have a coherent set of points
    // as the hole's outline. For each hole.
    outlinePointsConnector(conveyor);
  }

  /**
    @brief The function that is responsible for the linear and rotational
    transformation of the points given as arguments.
    @param[in] point [const cv::Point2f&].
    The input point coordinates to be converted.
    @return float. The final point in Rgb or Depth image.
    @param[in] xInit [double] The initial x coordinate of thermal image in rgb image
    @param[in] yInit [double] The initial y coordinate of thermal image in rgb image
    @param[in] c_x [double] The c factor on x-axis.
    @param[in] c_y [double] The c factor on y-axis.
    Found as matchedthermal_x/rgb_x same for y.
    @param[in] angle [double] The rotation of thermal image in rads.
    return cv::Point2f. The final point coordinates in Rgb or Depth image.
  **/
  cv::Point2f ImageMatching::matchingFunction(const cv::Point2f& point,
    double xInit, double yInit, double c_x, double c_y, double angle)
  {
    cv::Point2f linear;
    // The value of thermal image point.x or point.y in Rgb image after
    // linear transformation.
    linear.x = xInit + c_x * point.x;
    linear.y = yInit + c_y * point.y;

    cv::Point2f rotation;
    cv::Point2f newCenter;
    // Rotational transformation
    newCenter.x = c_x * Parameters::ThermalImage::WIDTH / 2 + xInit;
    newCenter.y = c_y * Parameters::ThermalImage::HEIGHT / 2 + yInit;

    rotation.x = newCenter.x + cos(angle) * (linear.x - newCenter.x) -
      sin(angle) * (linear.y - newCenter.y);

    rotation.y = newCenter.y + sin(angle) * (linear.x - newCenter.x) +
      cos(angle) * (linear.y - newCenter.y);

    return rotation;
  }


  /**
    @brief When the outline points of the thermal image are matched on
    the rgb image they are not connected anymore. So this function connects
    all the matched outline points and extracts the new matched outline vector.
    @param[out] conveyor[HolesConveyor*]. The final struct with
    the connected points.
    @return void
  **/
  void ImageMatching::outlinePointsConnector(HolesConveyor* conveyor)
  {
    // Draw the outline of the new ordered points for each hole
    for (unsigned int i = 0; i < conveyor->size(); i++)
    {
      cv::Mat outlineImage = cv::Mat::zeros(Parameters::Image::HEIGHT,
        Parameters::Image::WIDTH, CV_8UC1);

      for (unsigned int j = 0; j < conveyor->holes[i].outline.size(); j++)
      {
        cv::line(outlineImage, conveyor->holes[i].outline[j],
          conveyor->holes[i].outline[(j+1) % conveyor->holes[i].outline.size()],
          cv::Scalar(255, 0, 0), 1, 8);
      }
      // Clear the outline vector so it can be filled with the new outline
      // points from the outlineImage image.
      conveyor->holes[i].outline.clear();

      // Every non-zero point is a point drawn so pass it to the vector.
      for (unsigned int rows = 0; rows < outlineImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < outlineImage.cols; cols++)
        {
          if (outlineImage.at<unsigned char>(rows, cols) != 0)
          {
            conveyor->holes[i].outline.push_back(cv::Point2f(cols, rows));
          }
        }
      }
    }
  }


  /**
    @brief The vector of the outline points must have the points in
    order so they can be connected in the next step. The order is produced
    based on the distance between them. For that reason checks one
    point with all the others. In the next loop that point is beeing
    taken out of consideration.
    @param[out] conveyor[HolesConveyor*].
    The new conveyor with the points in order.
    @return void
  **/
  void ImageMatching::outlinePointsInOrder(HolesConveyor* conveyor)
  {
    // Vector of points that will contain the points in order.
    std::vector<cv::Point2f> newVector;

    // For each hole found.
    for (unsigned int i = 0; i < conveyor->size(); i++)
    {
      // Push the first point in the new vector and erase it from the old vector
      newVector.push_back(conveyor->holes[i].outline[0]);
      cv::Point2f currp = newVector[0];
      conveyor->holes[i].outline.erase(conveyor->holes[i].outline.begin());

      while (conveyor->holes[i].outline.size() > 0)
      {
        // Set as starting minimum value width*2, a value that can never
        // been surpassed by any distance between points
        float minimum = Parameters::Image::WIDTH * Parameters::Image::WIDTH;

        // Index that shows the place of the new closest point found in
        // the old vector.
        int minIndex = -1;

        for (unsigned int j = 0; j < conveyor->holes[i].outline.size(); j++)
        {
          // Find the x distance between two points and square it.
          double dx = std::pow(conveyor->holes[i].outline[j].x - currp.x, 2);

          // Find the y distance between two points and square it.
          double dy = std::pow(conveyor->holes[i].outline[j].y - currp.y, 2);

          // Find the euclidean distance between two points.
          double distance = std::sqrt(dx + dy);

          if (distance < minimum)
          {
            minimum = distance;
            minIndex = j;
          }
        }

        // Pass the point found to the new vector.
        newVector.push_back(conveyor->holes[i].outline[minIndex]);

        // The new initial point that we check distances changes.
        currp = conveyor->holes[i].outline[minIndex];

        // Erase that point from the starting vector.
        conveyor->holes[i].outline.erase(
          conveyor->holes[i].outline.begin() + minIndex);
      }
      // Pass the points found in order to the holesconveyor struct
      conveyor->holes[i].outline.clear();
      conveyor->holes[i].outline = newVector;

      // Clear the newVector so it can be used for the next hole
      newVector.clear();
    }
  }

  /**
    @brief Set up the variables needed to convert the points.Takes variables
    from yaml file so that if one (or both) camera's position is changed
    on the robot they can be changed too.
    @param[in] nh [const ros::Nodehandle&] The nodehandle of the class
    that calls the function.
    @param[out] x_th [double] The x coordinate of thermal image in rgb image
    @param[out] y_th [double] The y coordinate of thermal image in rgb image
    @param[out] c_x [double] The c factor on x-axis.
    Found as matchedthermal_x/rgb_x
    @param[out] c_y [double] The c factor on y-axis.
    Found as matchedthermal_y/rgb_y
    @param[out] angle [double] The angle in rads that thermal image been rotated
    Point of reference is the center of thermal image.
    @return void
  **/
  void ImageMatching::variableSetUp(const ros::NodeHandle& nh, double* x_th,
    double* y_th, double* c_x, double* c_y, double* angle)
  {
    // Read values of each variable.
    // x_thermal variable.
    if (!nh.getParam("matching_values/x_therm", *x_th))
    {
      ROS_FATAL("[Thermal_node], Could not find x_th variable");
    }

    // y_thermal variable.
    if (!nh.getParam("matching_values/y_therm", *y_th))
    {
      ROS_FATAL("[Thermal_node], Could not find y_th variable");
    }

    // C factor on x directions.
    if (!nh.getParam("matching_values/c_x", *c_x))
    {
      ROS_FATAL("[Thermal_node], Could not find c_x variable");
    }

    // C factor on y directions.
    if (!nh.getParam("matching_values/c_y", *c_y))
    {
      ROS_FATAL("[Thermal_node], Could not find c_y variable");
    }

    // The angle that thermal image been rotated.
    if (!nh.getParam("matching_values/angle", *angle))
    {
      ROS_FATAL("[Thermal_node], Could not find angle variable");
    }
    *angle = *angle * CV_PI / 180;
  }


}  // namespace depth
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
