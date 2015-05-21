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
  
  class CoverageWriter : public pandora_geotiff::MapWriterPluginInterface{
  public:
    CoverageWriter();
    virtual ~CoverageWriter();
  
    virtual void initialize(const std::string& name);
    virtual void draw(pandora_geotiff::MapWriterInterface *interface);
    void getGeotiffDataCoverage(nav_msgs::OccupancyGrid map);
  
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber sub_coverage;
    std::string coverage_topic_name_;
  
    bool initialized_;
    std::string name_;  
    
  private:
  
     nav_msgs::OccupancyGrid coverage;
     bool gotData;
     std::string COVERAGE_COLOR;
     int COV_BOT_THRES;
     int COV_TOP_THRES;
  };
  
  CoverageWriter::CoverageWriter(): initialized_(false)
  {}
  
  CoverageWriter::~CoverageWriter()
  {}
  void CoverageWriter::initialize(const std::string& name){
  
    ros::NodeHandle plugin_nh("~/" + name);
    plugin_nh.param("pandora_geotiff_node/published_topic_names/coverage_map", coverage_topic_name_,
      std::string("/data_fusion/sensor_coverage/kinect_space"));
    plugin_nh.param("pandora_geotiff_node/coverage_map_params/color",COVERAGE_COLOR, std::string("LIGHT_GREEN_MAX"));
    plugin_nh.param("pandora_geotiff_node/coverage_map_params/bot_thres",COV_BOT_THRES,-5);
    plugin_nh.param("pandora_geotiff_node/coverage_map_params/top_thres",COV_TOP_THRES,100);
  
  
    sub_coverage = plugin_nh.subscribe(coverage_topic_name_,
         1000,  &CoverageWriter::getGeotiffDataCoverage,this);
         
    initialized_ = true;
    this->name_ = name;
    ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff CoverageWriter plugin %s.", name_.c_str());
  }
  
  
  void CoverageWriter::getGeotiffDataCoverage(nav_msgs::OccupancyGrid coverage){
    this->coverage = coverage;
    gotData = true;
  }
  
  void CoverageWriter::draw(pandora_geotiff::MapWriterInterface *interface){
         if(!initialized_||!gotData){
        ROS_WARN_NAMED("CoverageWriter","CoverageWriterplugin not initilized or no data has been received /n ABORTING DRAWING..");
        return;
        }
  
      interface->drawMap(coverage,COVERAGE_COLOR,COV_BOT_THRES,COV_TOP_THRES);
  }

}// namespace pandora_geotiff

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::CoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, CoverageWriter, pandora_geotiff_plugins::CoverageWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
