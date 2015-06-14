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


#ifndef UTILS_IMAGE_MATCHING_H
#define UTILS_IMAGE_MATCHING_H

#include "utils/holes_conveyor.h"
#include "utils/parameters.h"
#include "utils/outline_discovery.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class ImageMatching
    @brief Matches the thermal image points of interest
    in the rgb image. For instance 80x60 to 640x480.
   **/
  class ImageMatching
  {
    public:

      /**
        @brief Converts Conveyors which represent each hole and include all
        necessary information about it to match with rgb and depth images.
        After transformation checks if the point of interest is outside of rgb 
        image borders. If so that hole will be rejected.
        @param[out] conveyor [HolesConveyor* conveyor] 
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
      static void conveyorMatching(HolesConveyor* conveyor, double x_th,
        double y_th, double c_x, double c_y, double angle);

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
      static cv::Point2f matchingFunction(const cv::Point2f& point,
        double xInit, double yInit, double c_x, double c_y, double angle);

      /**
        @brief Set up the variables needed to convert the points.Takes variables
        from yaml file so that if one (or both) camera's position is changed
        on the robot they can be changed too.
        @param[in] nh [ros::Nodehandle&] The nodehandle of the class that
        calls this function.
        @param[in] x_th [double] The x coordinate of thermal image in rgb image
        @param[in] y_th [double] The y coordinate of thermal image in rgb image
        @param[in] c_x [double] The c factor on x-axis.
        Found as matchedthermal_x/rgb_x
        @param[in] c_y [double] The c factor on y-axis.
        Found as matchedthermal_y/rgb_y
        @param[out] angle [double] The angle in rads that thermal image been 
        rotated.Point of reference is the center of thermal image.
        @return void
        **/
      static void variableSetUp(ros::NodeHandle& nh, double* x_th, double* y_th,
        double* c_x, double* c_y, double* angle);

      /**
        @brief When the outline points of the thermal image are matched on 
        the rgb image they are not connected anymore. So this function connects
        all the matched outline points and extracts the new matched 
        outline vector.
        @param[out] conveyor [const HolesConveyor&]. The final struct with 
        the connected points.
        @return void
       **/
      static void outlinePointsConnector(HolesConveyor* conveyor);

      /**
        @brief The vector of the outline points must have the points in
        order so they can be connected in the next step. The order is produced 
        based on the distance between them. For that reason checks one 
        point with all the others. In the next loop that point is beeing 
        taken out of consideration.
        @param[out] conveyor [HolesConveyor*].
        The new conveyor with the points in order.
        @return void
       **/
      static void outlinePointsInOrder(HolesConveyor* conveyor);
  };

} // namespace pandora_vision

#endif  // UTILS_IMAGE_MATCHING_H
