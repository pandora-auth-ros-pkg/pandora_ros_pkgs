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

#include <string>
#include <vector>

#include "pandora_geotiff/creator.h"


namespace pandora_geotiff
{
  Creator::Creator()
  {
    fake_argc_ = 0;

    // Create a QApplication cause otherwise drawing text will crash.
    app_ = new QApplication(fake_argc_, fake_argv_, false);

    backgroundImage_ = NULL;
    CHECKER_SIZE = 50;
    finalImage_ = NULL;

    // These parameters should be moved in a yaml file in a later version.
    missionName_ = std::string("TestMission");
    missionNamePrefix_ = std::string("/RRL_2015_PANDORA_");
    missionNameExtention_ = std::string(".tiff");
    CHECKER_COLOR_LIGHT = "LIGHT_GREY";
    CHECKER_COLOR_DARK = "DARK_GREY";
    MISSION_NAME_COLOR = "DARK_BLUE_F";
    MAP_SCALE_COLOR = "DARK_BLUE_M";
    MAP_ORIENTATION_COLOR = "DARK_BLUE_M";
    MISSION_NAME_WIDTH = 2;
    MAP_SCALE_WIDTH = 2;
    MAP_ORIENTATION_WIDTH = 2;

    MAP_OFFSET = 4 * CHECKER_SIZE;
    MISSION_NAME_COORDS = Eigen::Vector2f(CHECKER_SIZE / 2, CHECKER_SIZE / 2);
    MAP_SCALE_COORDS = Eigen::Vector2f(CHECKER_SIZE / 2, CHECKER_SIZE * 2);
    MAP_ORIENTATION_COORDS = Eigen::Vector2f(CHECKER_SIZE * 2, CHECKER_SIZE * 2);
    MAP_ORIENTATION_LENGTH = CHECKER_SIZE;

    // These parameters are given theris real values in a different state.
    mapXoffset_ = 0;
    mapYoffset_ = 0;
    trimmingXoffset_ = 0;
    trimmingYoffset_ = 0;
    mapResolution_ = 0.02;
    mapImage_= new QImage(200, 200, QImage::Format_ARGB32);
    mapInitialized_ = false;

    colorMap["DARK_BLUE_F"]     = QColor(0, 44, 207);
    colorMap["DARK_BLUE_M"]     = QColor(0, 50, 120);
    colorMap["DARK_BLUE_W"]     = QColor(0, 40, 120);
    colorMap["LIGHT_GREY"]      = QColor(226, 226, 227);
    colorMap["DARK_GREY"]       = QColor(237, 237, 238);
    colorMap["BLACK"]           = QColor(190, 190, 191);
    colorMap["YELLOW"]          = QColor(255, 200, 0);
    colorMap["WHITE_MAX"]       = QColor(255, 255, 255);
    colorMap["WHITE_MIN"]       = QColor(128, 128, 128, 150);
    colorMap["LIGHT_GREEN_MAX"] = QColor(180, 230, 180, 100);
    colorMap["LIGHT_GREEN_MIN"] = QColor(130, 230, 130, 100);
    colorMap["SOLID_RED"]       = QColor(240, 10, 10);
    colorMap["YELLOW"]          = QColor(255, 200, 0);
    colorMap["SOLID_ORANGE"]    = QColor(255, 100, 30);
    colorMap["SOLID_BLUE"]      = QColor(10, 10, 240);
    colorMap["MAGENTA"]         = QColor(120, 0, 140, 100);
    colorMap["PINK"]            = QColor(180, 0, 200);
  }

  Creator::~Creator() {}

  // This function must only be called after geotiffMapIm is initialized!
  void Creator::createBackgroundImage()
  {
    ROS_INFO("Creating BackGroundIm...");

    QPainter geotiffPainter;
    int width = mapImage_->width() + MAP_OFFSET;
    int height = mapImage_->height() + MAP_OFFSET;

    backgroundImage_= new QImage(width, height, QImage::Format_RGB32);
    geotiffPainter.begin(backgroundImage_);

    drawCheckers(CHECKER_SIZE, CHECKER_COLOR_LIGHT, CHECKER_COLOR_DARK, &geotiffPainter);
    drawMissionName(MISSION_NAME_COORDS, MISSION_NAME_COLOR, MISSION_NAME_WIDTH, &geotiffPainter);
    drawMapScale(MAP_SCALE_COORDS, MAP_SCALE_COLOR, MAP_SCALE_WIDTH, &geotiffPainter);
    drawMapOrientation(MAP_ORIENTATION_COORDS, MAP_ORIENTATION_COLOR, MAP_ORIENTATION_WIDTH, &geotiffPainter);

    QPoint point(2 * CHECKER_SIZE, 2 * CHECKER_SIZE);
    geotiffPainter.drawImage(point, *mapImage_);

    ROS_INFO("BackGroundIm was created succesfully...");
  }

  void Creator::saveGeotiff(const std::string &directory)
  {
    ROS_INFO("Saving Geotiff...");

    passwd* pw = getpwuid(getuid());
    std::string path(pw->pw_dir);

    path.append(directory);
    path.append(missionNamePrefix_);
    path.append(missionName_);
    path.append(missionNameExtention_);
    mapInitialized_ = false;

    if ((*backgroundImage_).save(path.c_str()))
    {
      ROS_INFO("Geotiff saved at %s", path.c_str());
    }
    else
    {
      ROS_ERROR("Couldn't save geotiff at %s.", path.c_str());
      ROS_INFO("Make sure that the directory exists and you have write access.");
    }
  }

  void Creator::setMissionName(const std::string &missionName)
  {
    missionName_= missionName;
  }

  void Creator::drawCheckers(const int& checkerSize, const std::string& colorD, const std::string& colorL,
                                    QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing Checkers...");
    QBrush gridBrush(colorMap[colorD]);

    // Draw (checkerboard) grid.
    for (int i = 0; i < backgroundImage_->width() / CHECKER_SIZE; i++)
    {
      for (int j = 0; j < backgroundImage_ ->height() / CHECKER_SIZE; j++)
      {
        if ((j + i) % 2)
        {
          geotiffPainter->setPen(colorMap[colorD]);
          gridBrush.setColor(colorMap[colorD]);
        }
        else
        {
          geotiffPainter->setPen(colorMap[colorL]);
          gridBrush.setColor(colorMap[colorL]);
        }
        geotiffPainter->drawRect(i * CHECKER_SIZE, j * CHECKER_SIZE, CHECKER_SIZE, CHECKER_SIZE);
        geotiffPainter->fillRect(i * CHECKER_SIZE, j * CHECKER_SIZE, CHECKER_SIZE, CHECKER_SIZE, gridBrush);
      }
    }
    ROS_INFO("Checkers were drawn succesfully!");
  }

  void Creator::drawMissionName(const Eigen::Vector2f& coords, const std::string& color, const int& width,
                                       QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing MissionName...");

    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(width);
    geotiffPainter->setPen(Pen);

    QPointF filenamePoint(coords.x(), coords.y());

    QString filenameString(QString(missionNamePrefix_.c_str()));
    filenameString.append(QString(missionName_.c_str()));
    filenameString.append(QString(missionNameExtention_.c_str()));

    geotiffPainter->drawText(filenamePoint, filenameString);

    ROS_INFO("MissionName was drawed succesfully");
  }

  void Creator::drawMapScale(const Eigen::Vector2f& coords, const std::string& color, const int& width,
                                    QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing MapScale...");

    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(width);
    geotiffPainter->setPen(Pen);

    // Drawing the main length unit side.
    QPoint Point1(coords.x(), coords.y());
    QPoint Point2(coords.x(), coords.y() - CHECKER_SIZE);
    geotiffPainter->drawLine(Point1, Point2);

    // Drawing the length unit up side (1/12 of the checker size).
    QPoint Point1Up(Point1.x() - CHECKER_SIZE / 12, Point1.y());
    QPoint Point2Up(Point1.x() + CHECKER_SIZE / 12, Point1.y());
    geotiffPainter->drawLine(Point1Up, Point2Up);

    // Drawing the length unit down side (1/12 of the checker size).
    QPoint Point1Down(Point2.x() - CHECKER_SIZE / 12, Point2.y());
    QPoint Point2Down(Point2.x() + CHECKER_SIZE / 12, Point2.y());
    geotiffPainter->drawLine(Point1Down, Point2Down);

    // Draw the length unit point 1m.
    QPointF TextPoint(Point1.x() + (CHECKER_SIZE * 3) / 50, Point1.y() - (CHECKER_SIZE * 3) / 5);
    QString Text("1m");
    geotiffPainter->drawText(TextPoint, Text);

    ROS_INFO("MapScale was drawed succesfully!");
  }

  void Creator::drawMapOrientation(const Eigen::Vector2f& coords, const std::string& color, const int& width,
                                          QPainter* geotiffPainter)
  {
    ROS_INFO("Drawing MapOrientation...");

    // Drawing the lines.
    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(width);
    geotiffPainter->setPen(Pen);

    QPoint pointY(coords.x() - CHECKER_SIZE, coords.y());
    QPoint pointC(coords.x(), coords.y());
    QPoint pointX(coords.x(), coords.y() - CHECKER_SIZE);

    geotiffPainter->drawLine(pointY, pointC);
    geotiffPainter->drawLine(pointC, pointX);

    // Drawing the Y arrow.
    QPoint pointYup(pointY.x() + CHECKER_SIZE / 10, pointY.y() - CHECKER_SIZE / 10);
    QPoint pointYdown(pointY.x() + CHECKER_SIZE / 10, pointY.y() + CHECKER_SIZE / 10);
    geotiffPainter->drawLine(pointY, pointYup);
    geotiffPainter->drawLine(pointY, pointYdown);

    // Drawing The X arrow.
    QPoint pointXup(pointX.x() - CHECKER_SIZE / 10, pointX.y() + CHECKER_SIZE / 10);
    QPoint pointXdown(pointX.x() + CHECKER_SIZE / 10, pointX.y() + CHECKER_SIZE / 10);
    geotiffPainter->drawLine(pointX, pointXup);
    geotiffPainter->drawLine(pointX, pointXdown);

    // Drawing Y.
    QPoint pointYText(pointY.x() + CHECKER_SIZE / 5, pointY.y() - CHECKER_SIZE / 6);
    QString YString("Y");
    geotiffPainter->drawText(pointYText, YString);

    // Drawing X.
    QPoint pointXText(pointX.x() + CHECKER_SIZE / 5, pointX.y() + CHECKER_SIZE / 6);
    QString XString("X");
    geotiffPainter->drawText(pointXText, XString);

    ROS_INFO("MapOrientation drawn succesfully!");
  }

  Eigen::Vector2i  Creator::metersToGeotiffPosition(const Eigen::Vector2f point)
  {
    Eigen::Vector2i tempPoint((point.y() * CHECKER_SIZE) - mapYoffset_, (point.x() * CHECKER_SIZE) - mapXoffset_);
    Eigen::Vector2i newPoint(mapImage_->width() - tempPoint.x() - trimmingXoffset_,
        mapImage_->height() - tempPoint.y() - trimmingYoffset_);
    return newPoint;
  }

  void Creator::drawMap(const nav_msgs::OccupancyGrid& map, const std::string& color, const int& bottomThres,
                        const int& topThres, const int& grid_space)
  {
    ROS_INFO("A map drawing was requested");

    int xRsize = static_cast<int>((map.info.height / CHECKER_SIZE) * CHECKER_SIZE) + CHECKER_SIZE;
    int yRsize = static_cast<int>((map.info.width / CHECKER_SIZE) * CHECKER_SIZE) + CHECKER_SIZE;

    int xsize = map.info.height;
    int ysize = map.info.width;

    QPainter* mapPainter = new QPainter();

    if (!mapInitialized_)
    {
      mapResolution_ = map.info.resolution;
      CHECKER_SIZE = static_cast<int>(1 / mapResolution_);

      mapXoffset_ = map.info.origin.position.x*CHECKER_SIZE;
      mapYoffset_ = map.info.origin.position.y*CHECKER_SIZE;

      trimmingXoffset_ = xRsize - xsize;
      trimmingYoffset_ = yRsize - ysize;

      mapImage_ = new QImage(xRsize, yRsize, QImage::Format_ARGB32);
      mapInitialized_ = true;

      mapPainter->begin(mapImage_);
      mapPainter->setCompositionMode(QPainter::CompositionMode_Source);
      mapPainter->fillRect(mapImage_->rect(), Qt::transparent);
      mapPainter->setCompositionMode(QPainter::CompositionMode_SourceOver);
    }
    else
    {
      mapPainter->begin(mapImage_);
    }

    QColor MapColor(colorMap[color]);
    QPen Pen = QPen(MapColor);
    mapPainter->setPen(Pen);

    for (int i = 0; i < xsize; i++)
    {
      for (int j = 0; j < ysize; j++)
      {
        if (bottomThres < map.data[xsize * ysize - j - i * ysize] &&
            topThres > map.data[xsize * ysize - j - i * ysize])
        {
          bool yGrid = !(j % (CHECKER_SIZE / 2));
          bool xGrid = !(i % (CHECKER_SIZE / 2));

          if (grid_space && (xGrid || yGrid))
          {
            mapPainter->setPen(QPen(colorMap["WHITE_MIN"]));
          }
          // i,j MUST BE xsize and J must be y size if you want to swap y with x u must change a lot of things
          mapPainter->drawPoint(i, j);
          mapPainter->setPen(Pen);
        }
      }
    }
    mapPainter->end();
    ROS_INFO("Map was drawn succesfully.");
  }

  void Creator::drawPath(const std::vector<Eigen::Vector2f>& points, const std::string& color, const int& width)
  {
    ROS_INFO("A path drawing was requested.");

    QPainter* pathPainter= new QPainter();
    pathPainter->begin(mapImage_);

    int rWidth = width*(CHECKER_SIZE/100);

    QPen Pen = QPen(colorMap[color]);
    Pen.setWidth(rWidth);
    pathPainter->setPen(Pen);

    Eigen::Vector2i point1;
    Eigen::Vector2i point2;

    for (int i = 0; i < points.size() - 1; i++)
    {
      point1 = metersToGeotiffPosition(points[i]);
      point2 = metersToGeotiffPosition(points[i + 1]);
      pathPainter->drawLine(point1.x(), point1.y(), point2.x(), point2.y());
    }
    pathPainter->end();
    ROS_INFO("Path was drawn succesfully.");
  }

  void Creator::drawPOI(const Eigen::Vector2f& meterCoords, const std::string& color, const std::string& txtcolor,
                        const std::string& shape, const std::string& txt, const int& size)
  {
    ROS_INFO("Object of shape %s requested", shape.c_str());

    QPainter* objectPainter = new QPainter();
    int rSize = size/(mapResolution_ * 100);

    objectPainter->begin(mapImage_);
    objectPainter->setBrush(colorMap[color]);
    objectPainter->setPen(colorMap[color]);

    Eigen::Vector2i pixelCoords = metersToGeotiffPosition(meterCoords);
    QPen Pen(colorMap[txtcolor]);
    QFont font;
    font.setPixelSize(rSize / 2);

    if (shape == "DIAMOND")
    {
      QPoint points[4];

      points[0] = QPoint(pixelCoords.x(), pixelCoords.y() + (rSize / 2));
      points[1] = QPoint(pixelCoords.x() + (rSize / 2), pixelCoords.y());
      points[2] = QPoint(pixelCoords.x(), pixelCoords.y() - (rSize / 2));
      points[3] = QPoint(pixelCoords.x() - (rSize / 2), pixelCoords.y());

      objectPainter->drawPolygon(points, 4);
      objectPainter->setPen(Pen);
      objectPainter->setFont(font);
      objectPainter->drawText(pixelCoords.x() - (rSize / 2), pixelCoords.y() - (rSize / 2), rSize, rSize,
                              Qt::AlignCenter, QString::fromStdString(txt));
    }
    else if (shape == "CIRCLE")
    {
      objectPainter->drawEllipse(QPoint(pixelCoords.x(), pixelCoords.y()), rSize / 2, rSize / 2);
      objectPainter->setFont(font);
      objectPainter->setPen(Pen);
      objectPainter->drawText(pixelCoords.x() - (rSize / 2), pixelCoords.y() - (rSize / 2), rSize, rSize,
                              Qt::AlignCenter, QString::fromStdString(txt));
    }
    else if (shape == "ARROW")
    {
      QPoint points[4];

      points[0] = QPoint(pixelCoords.x(), pixelCoords.y() - (3 * rSize));
      points[1] = QPoint(pixelCoords.x() + rSize, pixelCoords.y() + (2 * rSize));
      points[2] = QPoint(pixelCoords.x(), pixelCoords.y() + rSize);
      points[3] = QPoint(pixelCoords.x() - rSize, pixelCoords.y() + (2 * rSize));

      objectPainter->drawPolygon(points, 4);
    }
    else
    {
      ROS_WARN("This shape is not supported.");
    }
    objectPainter->end();
    ROS_INFO("Drawing Object of shape %s completed.", shape.c_str());
  }
}  // namespace pandora_geotiff

