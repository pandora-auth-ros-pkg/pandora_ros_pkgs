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
#ifndef _MAPCREATORINTERFACE_H__
#define _MAPCREATORINTERFACE_H__

#include <vector>
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

namespace pandora_geotiff
{
  
  //!< Type Definitions
  typedef nav_msgs::OccupancyGrid Map;
  typedef nav_msgs::OccupancyGrid* MapPtr;
  /**
  @class 
  @brief The main Interface  class. Contains the virtual abstract functions used by the plugins for drawing the geotiff .
  **/
  
  class MapWriterInterface {
  public:
  /**
  *@brief Draws the map (occupancy grid ) with a specific color
  *@details each pixel is painted differently 
  *@param map [*nav_msgs::OccupancyGrid] : The map to be painted
  *@param color [&std::string color] : The color to paint each pixel of the occupancy grid
  *@param grid_space [int ] :The space between the lines in the grid .If 0 then no grid is drawed 
  *@return void
  **/
    virtual void drawMap(const nav_msgs::OccupancyGrid& map,const std::string& color,
       const int& bottomThres,const int& topThres,const int& grid_space = 0 ) = 0;
  /**
  *@brief Draws an Object with a specific color and a specific shape
  *@details possible shapes {Diamond,Circle,Initial Arrow}
  *@param coords [&Eigen::Vector2f] : The coordinates of the Object
  *@param color [&std::string] : The color each Object is painted
  *@param shape [&std::string] : The shape each Object is painted
  *@param sequence [int] : The sequence each Object was found
  *@param size [int] : Size of the objects.Diameter for circles,sides for Diamonds and  special_object  
  *@return void
  **/
    virtual void drawObjectOfInterest(const Eigen::Vector2f& coords,const std::string& color,const std::string& txtcolor,
      const std::string& shape,const std::string& txt ,const int& size) = 0;
  /**
  *@brief Draws A trajectory of a specified color 
  *@details each pixel is painted differently
  *@param points [&std::vector<Eigen::Vector2f>]:The points to be painted
  *@param color [&std::string] : The color each Object is painted
  *@return void
  **/
    virtual void drawPath( const std::vector<Eigen::Vector2f>& points, const std::string& color, const int& width) = 0;
    
    virtual ~MapWriterInterface() {};


  };
}//namespace pandora_geotiff


#endif
