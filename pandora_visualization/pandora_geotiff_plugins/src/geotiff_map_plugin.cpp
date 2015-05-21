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

#include <pandora_geotiff/map_creator_interface.h>
#include <pandora_geotiff/map_writer_plugin_interface.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

namespace pandora_geotiff_plugins{

  class MapWriter : public pandora_geotiff::MapWriterPluginInterface{
  public:
    MapWriter();
    virtual ~MapWriter();

    virtual void initialize(const std::string& name);
    virtual void draw(pandora_geotiff::MapWriterInterface *interface);
    void getGeotiffDataMap(nav_msgs::OccupancyGrid map);

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub;

    bool initialized_;
    std::string name_;


  private:

     nav_msgs::OccupancyGrid map;
     bool gotData;
     std::string MAP_COLOR;
     int MAP_BOT_THRES;
     int MAP_TOP_THRES;
     std::string WALL_COLOR;
     int WALL_BOT_THRES;
     int WALL_TOP_THRES;
  };

  MapWriter::MapWriter(): initialized_(false)
  {}

  MapWriter::~MapWriter()
  {}
  void MapWriter::initialize(const std::string& name)
  {
    ros::NodeHandle plugin_nh("~/" + name);
    std::string map_topic_name_;

    plugin_nh.param("/pandora_geotiff_node/published_topic_names/map", map_topic_name_, std::string("/slam/map"));
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/explored_bot_thres",MAP_BOT_THRES,0);
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/explored_top_thres",MAP_TOP_THRES,0);
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/explored_color",MAP_COLOR, std::string("WHITE_MAX"));
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/wall_bot_thres",WALL_BOT_THRES,0);
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/wall_top_thres",WALL_TOP_THRES,0);
    plugin_nh.param("/pandora_geotiff_node/explored_map_params/wall_color",WALL_COLOR, std::string("SOLID_RED"));

    map_sub = plugin_nh.subscribe(map_topic_name_,
         1000,  &MapWriter::getGeotiffDataMap,this);

    initialized_ = true;
    this->name_ = name;
    ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff MapWriter plugin %s.", name_.c_str());
  }

  void MapWriter::getGeotiffDataMap(nav_msgs::OccupancyGrid map){
    this->map = map;
    gotData = true;
  }

  void MapWriter::draw(pandora_geotiff::MapWriterInterface *interface){
       if(!initialized_||!gotData)
    {
      ROS_WARN_NAMED("MapWriter","plugin not initialized or no data has been received /n ABORTING DRAWING..");
      return;
      }

      interface->drawMap(map, MAP_COLOR, MAP_BOT_THRES, MAP_TOP_THRES,1);
      interface->drawMap(map, WALL_COLOR, WALL_BOT_THRES, WALL_TOP_THRES, 0);
  }

}
// namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::MapWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, MapWriter, pandora_geotiff_plugins::MapWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
