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
#include <pandora_data_fusion_msgs/GeotiffSrv.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>

namespace pandora_geotiff_plugins
{
  
  class ObjectsWriter : public pandora_geotiff::MapWriterPluginInterface
  {
  public:
    ObjectsWriter();
    virtual ~ObjectsWriter();
  
    virtual void initialize(const std::string& name);
    virtual void draw(pandora_geotiff::MapWriterInterface *interface);
    void getObjectsData();
   
  protected:
    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
    std::string service_name_;
  
    bool initialized_;
    bool gotData_;
    std::string name_;
    bool draw_all_objects_;
    std::string class_id_;
    std::string QRS_COLOR;
    std::string VICTIMS_COLOR;
    std::string TXT_COLOR;
    std::string HAZMATS_COLOR;
    std::string HAZMATS_SHAPE;
    std::string QRS_SHAPE;
    std::string VICTIMS_SHAPE;
    int HAZMATS_SIZE;
    int QRS_SIZE;
    int VICTIMS_SIZE;
    
  private:
  
    std::vector<geometry_msgs::PoseStamped> victims_;
    std::vector<geometry_msgs::PoseStamped> qrs_;
    std::vector<geometry_msgs::PoseStamped> hazmats_;
  
  };
  
  ObjectsWriter::ObjectsWriter()
      : initialized_(false),gotData_(false)
  {}
  
  ObjectsWriter::~ObjectsWriter()
  {}
  
  void ObjectsWriter::initialize(const std::string& name)
  {
    ros::NodeHandle plugin_nh("~" + name);
    
    plugin_nh.param("/pandora_geotiff_node/service_topic_names/data_fusion_objects", service_name_, std::string("data_fusion_geotiff"));
    plugin_nh.param("/pandora_geotiff_node/hazmat_params/shape",HAZMATS_SHAPE,std::string("DIAMOND"));
    plugin_nh.param("/pandora_geotiff_node/hazmat_params/color",HAZMATS_COLOR,std::string("SOLID_ORANGE"));
    plugin_nh.param("/pandora_geotiff_node/hazmat_params/size",HAZMATS_SIZE,1);
    plugin_nh.param("/pandora_geotiff_node/qr_params/shape",QRS_SHAPE,std::string("DIAMOND"));
    plugin_nh.param("/pandora_geotiff_node/qr_params/color",QRS_COLOR,std::string("SOLID_ORANGE"));
    plugin_nh.param("/pandora_geotiff_node/qr_params/size",QRS_SIZE,1);
    plugin_nh.param("/pandora_geotiff_node/victim_params/shape",VICTIMS_SHAPE,std::string("DIAMOND"));
    plugin_nh.param("/pandora_geotiff_node/victim_params/color",VICTIMS_COLOR,std::string("SOLID_ORANGE"));
    plugin_nh.param("/pandora_geotiff_node/victim_params/size",VICTIMS_SIZE,1);
    plugin_nh.param("/pandora_geotiff_node/victim_params/size",VICTIMS_SIZE,1);
    plugin_nh.param("/pandora_geotiff_node/general_objects_params/txt_color",TXT_COLOR,std::string("SOLID_ORANGE"));
  
    service_client_ = nh_.serviceClient<pandora_data_fusion_msgs::GeotiffSrv>(service_name_);
  
    initialized_ = true;
    this->name_ = name;
    ROS_INFO_NAMED(name_, "Successfully initialized pandora_geotiff  ObjectsWriter plugin %s.", name_.c_str());
  }
  
  
  void ObjectsWriter::getObjectsData()
  {
      pandora_data_fusion_msgs::GeotiffSrv dataFusionSrv;
  
      if (!service_client_.call(dataFusionSrv)) {
        ROS_ERROR_NAMED(name_, "Cannot draw Objects, service %s failed", service_client_.getService().c_str());
        return;
      }
       
      victims_ = dataFusionSrv.response.victims;
      ROS_INFO("VICTIMS_SAVED SUCCESEFULLY");
      qrs_ = dataFusionSrv.response.qrs;
      ROS_INFO("QRS_SAVED SUCCESEFULLY");
      hazmats_ = dataFusionSrv.response.hazmats; 
      ROS_INFO("HAZMATS_SAVED SUCCESFULLY");
      gotData_ = true;
    }
  
  
  void ObjectsWriter::draw(pandora_geotiff::MapWriterInterface *interface)
  {
      this->getObjectsData();
  
      if(!initialized_||!gotData_)
      {
        ROS_WARN_NAMED("OBjectsWriter","ObjectWriter plugin not initilized or no data has been received /n ABORTING DRAWING..");
        return;
  
      }
      ROS_INFO("DRAWING THE AWESOME OBJECTS");
      Eigen::Vector2f coords;
      std::string txt;
  
      tf::TransformListener listener;
      tf::StampedTransform transform;
      tfScalar pitch, roll, yaw;
      tf::Vector3 origin ;
      float x,y;
      
      for (int i = 0 ; i< qrs_.size(); i++){
  
        //Get Tf
        try{
    
          listener.waitForTransform("/map", qrs_[i].header.frame_id ,qrs_[i].header.stamp, ros::Duration(1));
          listener.lookupTransform("/map", qrs_[i].header.frame_id , qrs_[i].header.stamp, transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        //Get  origin and roll , pitch, yaw
        transform.getBasis().getRPY(roll, pitch, yaw);
        origin = transform.getOrigin();
        //Add The origin vector
        x  =  qrs_[i].pose.position.x + origin.x();
        y  = qrs_[i].pose.position.y + origin.y();
        //Rotate acording to yaw and  pitch
        coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
        txt = boost::lexical_cast<std::string>(i+1);
       
       interface->drawObjectOfInterest(coords,QRS_COLOR ,TXT_COLOR, QRS_SHAPE,txt,QRS_SIZE);
      }
  
      for (int i = 0 ; i< victims_.size(); i++){
  
         //Get Tf
        try{
    
          listener.waitForTransform("/map", victims_[i].header.frame_id ,victims_[i].header.stamp, ros::Duration(1));
          listener.lookupTransform("/map", victims_[i].header.frame_id , victims_[i].header.stamp, transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        //Get  origin and roll , pitch, yaw
        transform.getBasis().getRPY(roll, pitch, yaw);
        origin = transform.getOrigin();
  
        //Add The origin vector
        x  = victims_[i].pose.position.x + origin.x();
        y  = victims_[i].pose.position.y + origin.y();
        //Rotate acording to yaw and  pitch
        coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
        txt = boost::lexical_cast<std::string>(i+1);
  
        
       interface->drawObjectOfInterest(coords,VICTIMS_COLOR ,TXT_COLOR, VICTIMS_SHAPE,txt,VICTIMS_SIZE);
      }
      
      for (int i = 0 ; i< hazmats_.size(); i++){
      //Get Tf
      try{
    
          listener.waitForTransform("/map", hazmats_[i].header.frame_id ,hazmats_[i].header.stamp, ros::Duration(1));
          listener.lookupTransform("/map", hazmats_[i].header.frame_id , hazmats_[i].header.stamp, transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
       //Get  origin and roll , pitch, yaw
        transform.getBasis().getRPY(roll, pitch, yaw);
        origin = transform.getOrigin();
  
        //Add The origin vector
        x  = hazmats_[i].pose.position.x + origin.x();
        y  = hazmats_[i].pose.position.y + origin.y();
        //Rotate acording to yaw and  pitch
        coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
        txt = boost::lexical_cast<std::string>(i+1);
       
       interface->drawObjectOfInterest(coords,HAZMATS_COLOR ,TXT_COLOR, HAZMATS_SHAPE,txt,HAZMATS_SIZE);
      }
  
      ROS_INFO("DRAWING THE AWESOME OBJECTS SUCCEEDED");
     
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
#ifdef PLUGINLIB_EXPORT_CLASS
  PLUGINLIB_EXPORT_CLASS(pandora_geotiff_plugins::ObjectsWriter, pandora_geotiff::MapWriterPluginInterface)
#else
  PLUGINLIB_DECLARE_CLASS(pandora_geotiff_plugins, ObjectsWriter, pandora_geotiff_plugins::ObjectsWriter, pandora_geotiff::MapWriterPluginInterface)
#endif
