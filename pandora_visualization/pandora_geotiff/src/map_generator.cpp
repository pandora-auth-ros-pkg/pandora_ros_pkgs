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

#include "pandora_geotiff/map_generator.h"
namespace pandora_geotiff
{
  MapGenerator::MapGenerator(): geotiffCreator(new GeotiffCreator),
    qrCsvCreator(new QrCsvCreator), plugin_loader_(0) ,pn_("~"){
      
      pn_.param("plugins", p_plugin_list_, std::string(""));
  
      std::vector<std::string> plugin_list;
      boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));
  
      //We always have at least one element containing "" in the string list
      if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
        plugin_loader_ = new pluginlib::ClassLoader<MapWriterPluginInterface>(
        "pandora_geotiff", "pandora_geotiff::MapWriterPluginInterface");
  
        for (size_t i = 0; i < plugin_list.size(); ++i){
          try
          { 
            boost::shared_ptr<MapWriterPluginInterface> tmp (plugin_loader_->createInstance(plugin_list[i]));
            tmp->initialize(plugin_loader_->getName(plugin_list[i]));
            plugin_vector_.push_back(tmp);
          }
          catch(pluginlib::PluginlibException& ex)
          {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
          }
        }
      }else{
        ROS_INFO("No plugins loaded for geotiff node");
      }
      
      save_mission_service = pn_.advertiseService("saveMission",&MapGenerator::saveGeotiff,this);
  
      ROS_INFO("Geotiff node started");
    }
   MapGenerator::~MapGenerator()
    {
      if (plugin_loader_){
        delete plugin_loader_;
      }
    }
  
    void MapGenerator::writeGeotiff(const std::string& missionName)
    {
      
      for (size_t i = 0; i < plugin_vector_.size(); ++i){
        plugin_vector_[i]->draw(geotiffCreator);
      }
      geotiffCreator->setMissionName(missionName);
      geotiffCreator->createBackgroundIm();
      //The Default saving target is ~/Desktop    
      geotiffCreator->saveGeotiff();
  }
  
    bool MapGenerator::saveGeotiff(SaveMission::Request& req ,
      SaveMission::Response& res )
    {
      ROS_INFO("SaveMission service was requested");
      
      this->writeGeotiff(std::string(req.SaveMisionFileName.data));
      return true;
    }
}// namespace pandora_geotiff
