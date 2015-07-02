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
 *   Sideris Konstantinos <siderisk@auth.gr>
 *********************************************************************/

#ifndef PANDORA_GEOTIFF_CREATOR_H
#define PANDORA_GEOTIFF_CREATOR_H

#include <vector>
#include <map>
#include <string>
#include <sys/types.h>

#include <unistd.h>
#include <pwd.h>

#include <QtGui>
#include <QAction>

#include <Eigen/Core>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


namespace pandora_geotiff
{
  class Creator
  {
    public:
      /**
       * @brief Creator constructor
       */

      Creator();

      /**
       * @brief Creator destructor
       */

      ~Creator();

      void setMissionName(const std::string &name);

      /**
       * @brief Create the background image.
       *
       * @details It includes the checkers, orientation, map scale and the mission name.
       *
       * @warning This function must be called after the MapIm has been created or its size is known.
       */

      void createBackgroundImage();

      /**
       * @brief Save the final geotiff image to a specified path.
       */

      void saveGeotiff(const std::string &path);

      /**
       * @brief Create the geotiff.
       */

      void createGeotiff(const std::string &fileName);

      /**
       * @brief Draw the map.
       */

      void drawMap(const nav_msgs::OccupancyGrid &map, const std::string &color, const int &bottomThr,
                   const int &topThr, const int &grid_space = 0);

      /**
       * @brief Draw a point of interest on the map.
       *
       * @details Could be a QR, Hazmat, Victim etc.
       *
       * @param coords   [&Eigen::Vector2f] The coordinates of the object.
       * @param txt      [&int]             A text describing the object
       * @param size     [&int]             The size of the object.
       * @param shape    [&std::string]     The shape of the object.
       * @param color    [&std::string]     The color of the object.
       * @param txtColor [&std::string]     The color of the text for the object.
       *
       * @return void
       */

      void drawPOI(const Eigen::Vector2f &coords, const std::string &color, const std::string &txtColor,
                   const std::string &shape, const std::string &txt, const int &size);

      /**
       * @brief Draw a path.
       *
       * @param points [&std::vector<Eigen::Vector2f>] A series of points that describe the path.
       * @param color  [&std::string]                  The color of the path.
       * @param width  [&int]                          The width of the path.
       *
       * @return void
       */

      void drawPath(const std::vector<Eigen::Vector2f> &points, const std::string &color, const int &width);

      /**
       * @brief Draw the name of the mission.
       *
       * @param coords [&Eigen::Vector2f] The coordinates of the text.
       * @param color  [&std::string]     The color of the text.
       * @param width  [&int]             The width of the text.
       *
       * @return void
       */

      void drawMissionName(const Eigen::Vector2f &coords, const std::string &color, const int &width, QPainter *pen);

      /**
       * @brief Draw the scale of the map.
       *
       * @param coords [&Eigen::Vector2f] The coordinates of the map scale.
       * @param color  [&std::string]     The color of the map scale.
       * @param width  [&int]             The width of the map scale.
       * @param size   [&int]             The size of the map scale.
       *
       * @return void
       */

      void drawMapScale(const Eigen::Vector2f &coords, const std::string &color, const int &width, QPainter *pen);

      /**
       * @brief Draw the scale of the map.
       *
       * @param coords [&Eigen::Vector2f] The coordinates of the map scale.
       * @param color  [&std::string]     The color of the map scale.
       * @param width  [&int]             The width of the map scale.
       * @param size   [&int]             The size of the map scale.
       *
       * @return void
       */

      void drawMapOrientation(const Eigen::Vector2f &coords, const std::string &color, const int &width, QPainter *pen);

      /**
       * @brief Draw the checkers on the background.
       *
       * @param darkColor  [&std::string] The dark color of the cherckers.
       * @param lightColor [&std::string] The light color of the cherckers.
       * @param size       [&int]         The size of the checkers.
       *
       * @return void
       */

      void drawCheckers(const int &size, const std::string &darkColor, const std::string &lightColor, QPainter *pen);

    private:
      Eigen::Vector2i metersToGeotiffPosition(const Eigen::Vector2f point);

      //!< The name of the mission.
      std::string missionName_;

      //!< The mission prefix. e.g: "/RRL_2015_PANDORA_"
      std::string missionNamePrefix_;

      //!< The file extension e.g: ".tiff"
      std::string missionNameExtention_;

      /**
       * Qt/Geotiff specific parameters.
       */

      //!< A hash table with the color names.
      std::map<std::string, QColor> colorMap;

      //!< Qt Application.
      QApplication *app_;

      int fake_argc_;
      char **fake_argv_;

      //!< The background image.
      QImage *backgroundImage_;

      //!< The Map image.
      QImage *mapImage_;

      //!< The final image.
      QImage *finalImage_;

      /**
       * Map specific parameters.
       */

      int mapXoffset_;
      int mapYoffset_;

      int trimmingXoffset_;
      int trimmingYoffset_;

      float mapResolution_;
      bool mapInitialized_;

      int CHECKER_SIZE;
      int MAP_OFFSET;
      int MISSION_NAME_WIDTH;
      int MAP_SCALE_WIDTH;
      int MAP_ORIENTATION_WIDTH;
      int MAP_ORIENTATION_LENGTH;

      std::string CHECKER_COLOR_LIGHT;
      std::string CHECKER_COLOR_DARK;
      std::string MISSION_NAME_COLOR;
      std::string MAP_SCALE_COLOR;
      std::string MAP_ORIENTATION_COLOR;

      Eigen::Vector2f MISSION_NAME_COORDS;
      Eigen::Vector2f MAP_SCALE_COORDS;
      Eigen::Vector2f MAP_ORIENTATION_COORDS;
  };
}  // namespace pandora_geotiff

#endif  // PANDORA_GEOTIFF_CREATOR_H
