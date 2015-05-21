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
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>


namespace pandora_geotiff_plugins
{



class PathWriter : public pandora_geotiff::MapWriterPluginInterface
{
  public:
    PathWriter();
    virtual ~PathWriter();
  
    virtual void initialize(const std::string& name);
    virtual void draw(pandora_geotiff::MapWriterInterface *interface);
    void getRobotTrajectoryData(nav_msgs::Path robotPath);
  
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub;
  
    bool initialized_;
    std::string name_;
  
    private:
      
      bool gotData;
      nav_msgs::Path robotPath;
      std::string PATH_COLOR;
      std::string ARROW_COLOR;
      int ARROW_SIZE;
      int PATH_WIDTH;
  
  };

PathWriter::PathWriter()
    : initialized_(false),gotData(false),PATH_COLOR("MAGENTA"), PATH_WIDTH(2), ARROW_COLOR("YELLOW"), ARROW_SIZE(7)
{}

PathWriter::~PathWriter()
{}

void PathWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string path_topic_name;

  plugin_nh.param("/pandora_geotiff_node/published_topic_names/trajectory", path_topic_name, std::string("/robot_trajectory"));
  plugin_nh.param("/pandora_geotiff_node/arrow_params/color",ARROW_COLOR,std::string("DIAMOND"));
  plugin_nh.param("/pandora_geotiff_node/arrow_params/size",ARROW_SIZE,20);
  plugin_nh.param("/pandora_geotiff_node/path_params/color",PATH_COLOR,std::string("SOLID_ORANGE"));
  plugin_nh.param("/pandora_geotiff_node/path_params/width",PATH_WIDTH,10);
  path_sub = plugin_nh.subscribe(path_topic_name , 1000, &PathWriter::getRobotTrajectoryData,this);
  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff PathWriter plugin %s.", name_.c_str());
}

void PathWriter::getRobotTrajectoryData(nav_msgs::Path robotPath)
{
  this->robotPath = robotPath;
  ROS_INFO("_PATH_DATA_RECEIVED");
  gotData = true;
    }

  
void PathWriter::draw(pandora_geotiff::MapWriterInterface *interface)
{
    if(!initialized_||!gotData)
    {
      ROS_WARN_NAMED("PathWriter","PathWriter plugin not initilized or no data has been received /n ABORTING DRAWING..");
      return;
      }
    
    ROS_INFO("DRAWING THE AWESOME PATH");
    
    std::vector<geometry_msgs::PoseStamped>& path_vector(robotPath.poses);

    size_t size = path_vector.size();

    std::vector<Eigen::Vector2f> pointVec;
    pointVec.resize(size);
    
    ROS_INFO("Robot path Size %ld ", size);
    

    for (size_t i = 0; i < size; ++i){
      const geometry_msgs::PoseStamped& pose (path_vector[i]);

      pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }
    
    if (size > 0){

      interface->drawPath(pointVec,PATH_COLOR,PATH_WIDTH);
      interface->drawObjectOfInterest(pointVec[0],ARROW_COLOR,"irelevant","ARROW","irelavant",ARROW_SIZE);
    }
    ROS_INFO("DRAWED A PATH SUCCESFULLY");
    
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::PathWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, PathWriter, pandora_geotiff_plugins::PathWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
