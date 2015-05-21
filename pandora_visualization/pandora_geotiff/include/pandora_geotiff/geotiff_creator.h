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

#ifndef GeotiffCreator_H
#define GeotiffCreator_H
#include <QtGui>
#include <QAction>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include "ros/ros.h"
#include "pandora_geotiff/map_creator_interface.h"



namespace pandora_geotiff{
    
  class GeotiffCreator : public MapWriterInterface
  {
  
  public:
    /**
    *@brief GeotiffCreator constructor
    **/
    GeotiffCreator();
    /**
    *@brief GeotiffCreator destructor
    **/
    ~GeotiffCreator() {  };
    /**
    *@brief creates All the GeotiffBackgroundIm
    * @details It includes TheCheckers,The Orientation,The Map Scale,The MissionName
    *@warning This function Must be Called After the MapIm has been created or its size is known.
    **/
    void createBackgroundIm();
    /**
    *@brief save the GeotiffFinalGeotiffImg to the specified path
    **/
    void saveGeotiff(std::string homeFolderString = "/Desktop");
    /**
    *@brief sets the MissionName
    **/
    void setMissionName(std::string missionName);
  
    
    virtual void drawMap(const nav_msgs::OccupancyGrid& map, const std::string& color,
      const int& bottomThres,const int& topThres, const int& grid_space = 0);
    virtual void drawObjectOfInterest(const Eigen::Vector2f& coords,const std::string& color, const std::string& txtcolor,
       const std::string& shape, const std::string& txt, const int& size);
    virtual void drawPath( const std::vector<Eigen::Vector2f>& points, const std::string& color, const int& width);
  
  private:
  
    void geotiffTimerCb(const ros::TimerEvent& event);
    /**
    *@brief drawsTheCheckers of specified size and,color ,
    *@param colorD [&std::string] : The Dark Color of the checkers
    *@param colorL [&std::string] : The Light Color of the Checkers
    *@param checkerSize [&std::string] : The size of the Checkers
    *@return void
    **/
    void drawCheckers (const int& checkerSize,const std::string& colorD,const std::string& colorL, QPainter* geotiffPainter);
    /**
    *@brief draws The missionName with a spesific color in a spesific point
    *@param coords [&Eigen::Vector2f] : The coordinates of the Mission Name
    *@param color [&std::string] : The color the Mission name is painted
    *@param width [&int ] : the width of the pen tha will be used
    *@return void
    **/
    void drawMissionName(const Eigen::Vector2f& coords,const std::string& color, const int& width, QPainter* geotiffPainter);
    /**
    *@brief draws The mapScale with a spesific color in a spesific point
    *@param coords [&Eigen::Vector2f] : The coordinates of the mapscale
    *@param color [&std::string] : The color the mapScale
    *@param width [int] : the width of the pen tha will be used
    *@param size  [int] : the size of the Mapscale
    *@return void
    **/
    void drawMapScale(const Eigen::Vector2f& coords,const std::string& color, const int& width, QPainter* geotiffPainter);
    /**
    *@brief draws The mapOrientation with a spesific color in a spesific point
    *@detail the length of the arrow is decided by the size of the checker
    *@param coords [Eigen::Vector2f] : The coordinates of the The mapOrientation(the point each line meets)
    *@param color [std::string] : The color the mapOrientation
    *@param width [int] : the width of the pen tha will be used
    *@return void
    **/
    void drawMapOrientation(const Eigen::Vector2f& coords,const std::string& color,
      const int& width, QPainter* geotiffPainter);

    Eigen::Vector2i  transformFromMetersToGeotiffPos(const Eigen::Vector2f point);
    
    std::map<std::string,QColor> colorMap; //!< A Map that corelates all the colors name to string Colors
    
  // I declare these As pointers In case the are moved in another Class :)
    QImage* geotiffBackgroundIm_; //!< The background Im
    QImage* geotiffMapIm_; //!< The MapIm
    QImage* geotiffFinalIm_; //!< The MapIm+BackgroundIm
    std::string missionName_; //!< The MissionName
    std::string missionNamePrefix_; //!< The MissionNamePrefix Ex: "/RRL_2015_PANDORA_"
    std::string missionNameExtention_; //!< The MissionNameExtention Ex: ".tiff"
    
    QApplication* app_;
    int fake_argc_;
    char** fake_argv_;
 
    // Map parameters

    int mapXoffset_;
    int mapYoffset_;
    int trimmingXoffset_ ;
    int trimmingYoffset_ ;
    float  geotiffMapRes_;
    bool mapInitialized_;
    
    int CHECKER_SIZE;
    int MAP_OFFSET;
    std::string CHECKER_COLOR_LIGHT;
    std::string CHECKER_COLOR_DARK;
    Eigen::Vector2f MISSION_NAME_COORDS;
    std::string MISSION_NAME_COLOR;
    int MISSION_NAME_WIDTH;
    Eigen::Vector2f MAP_SCALE_COORDS;
    std::string MAP_SCALE_COLOR;
    int MAP_SCALE_WIDTH;
    Eigen::Vector2f MAP_ORIENTATION_COORDS;
    std::string MAP_ORIENTATION_COLOR;
    int MAP_ORIENTATION_WIDTH;
    int MAP_ORIENTATION_LENGTH;

  };

}// namespace pandora_geotiff


#endif
