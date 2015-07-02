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
 * Authors:
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include <vector>
#include <string>
#include <ros/package.h>
#include "gtest/gtest.h"

#include "pandora_testing_tools/map_loader/map_loader.h"
#include "pandora_geotiff/creator.h"


namespace pandora_geotiff
{
  class GeotiffCreatorTest : public ::testing::Test
  {
    protected:
      GeotiffCreatorTest()
      {
        ros::Time::init();
        gc = Creator();
        std::string packagePath = ros::package::getPath("pandora_geotiff");
        map = map_loader::loadMap(packagePath + "/test/test_maps/map1.yaml");

        points.resize(250);

        for (int i = 0; i < 251; i++) {
          points[i] = Eigen::Vector2f(i / 10, i / 10);
        }
      }

      std::vector<Eigen::Vector2f> points;
      Creator gc;
      nav_msgs::OccupancyGrid map;
  };

  TEST_F(GeotiffCreatorTest, createBackgroundIm)
  {
    gc.drawMap(map, "WHITE_MAX", -5, 5, 1);
    gc.drawMap(map, "MAGENTA", 80, 110, 0);
    gc.drawPath(points, "SOLID_ORANGE", 3);
    gc.drawPOI(Eigen::Vector2f(2, 5), "PINK", "WHITE_MAX", "CIRCLE", "5", 20);
    gc.drawPOI(Eigen::Vector2f(4, 3), "SOLID_RED", "WHITE_MAX", "DIAMOND", "5", 50);
    gc.drawPOI(Eigen::Vector2f(7, 6), "SOLID_BLUE", "WHITE_MAX", "CIRCLE", "5", 30);
    gc.drawPOI(Eigen::Vector2f(6, 7), "SOLID_BLUE", "WHITE_MAX", "CIRCLE", "5", 30);
    gc.drawPOI(Eigen::Vector2f(6, 6), "MAGENTA", "WHITE_MAX", "CIRCLE", "5", 30);
    gc.drawPOI(Eigen::Vector2f(0, 0), "YELLOW", "WHITE", "ARROW", "5", 30);
    gc.drawPOI(Eigen::Vector2f(1, 1), "YELLOW", "WHITE", "ARROW", "5", 10);
    gc.drawPOI(Eigen::Vector2f(10, 10), "YELLOW", "WHITE", "ARROW", "5", 10);
    gc.createBackgroundImage();

    // Save to the home directory.
    gc.saveGeotiff("");
  }
}  // namespace pandora_geotiff
