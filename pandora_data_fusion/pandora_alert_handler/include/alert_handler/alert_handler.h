// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_ALERT_HANDLER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_ALERT_HANDLER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int16.h>

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/QRAlertsVectorMsg.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"
#include "vision_communications/HolesPositionsVectorMsg.h"
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"
#include "data_fusion_communications/VictimVerificationMsg.h"
#include "data_fusion_communications/GetVictimsAction.h"
#include "data_fusion_communications/VictimInfoMsg.h"
#include "data_fusion_communications/DeleteCurrentVictimAction.h"
#include "data_fusion_communications/ValidateCurrentHoleAction.h"
#include "data_fusion_communications/GetObjectsSrv.h"
#include "data_fusion_communications/DatafusionGeotiffSrv.h"
#include "data_fusion_communications/GetMarkersSrv.h"

#include "state_manager/state_client.h"

#include "pandora_alert_handler/AlertHandlerConfig.h"
#include "alert_handler/defines.h"
#include "alert_handler/object_list.h"
#include "alert_handler/pose_finder.h"
#include "alert_handler/object_handler.h"
#include "alert_handler/victim_handler.h"


typedef actionlib::SimpleActionServer
    <data_fusion_communications::GetVictimsAction> GetVictimsServer;
typedef actionlib::SimpleActionServer
    <data_fusion_communications::DeleteCurrentVictimAction> DeleteVictimServer;
typedef actionlib::SimpleActionServer
    <data_fusion_communications::ValidateCurrentHoleAction>
                                                    ValidateCurrentHoleServer;


typedef unsigned char** Map;

class AlertHandler : public StateClient {

 public:

  AlertHandler();

  void holeDirectionAlertCallback(
    const vision_communications::HolesDirectionsVectorMsg& msg);
  void holePositionAlertCallback(
    const vision_communications::HolesPositionsVectorMsg& msg);
  void faceDirectionAlertCallback(
    const vision_communications::FaceDirectionMsg& msg);
  void tpaDirectionAlertCallback(
    const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  void mlxDirectionAlert(
    const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  void victimVerificationCallback(
    const data_fusion_communications::VictimVerificationMsg& msg);
  void hazmatAlertCallback(
    const vision_communications::HazmatAlertsVectorMsg& msg);
  void qrAlertCallback(const vision_communications::QRAlertsVectorMsg& msg);
  void selectedVictimCallback(const std_msgs::Int16& msg);

  void getVictimsCallback();
  void deleteVictimCallback();
  void validateCurrentHoleCallback();

  void currentVictimTimerCb(const ros::TimerEvent& event);

 private:

  void initRosInterfaces();

  void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);
  
  void initializeMap();

  void dynamicReconfigCallback(
      pandora_alert_handler::AlertHandlerConfig &config,
        uint32_t level);

  bool flushQueues(
    std_srvs::Empty::Request& rq,
      std_srvs::Empty::Response &rs);

  bool getObjectsServiceCb(
    data_fusion_communications::GetObjectsSrv::Request& rq,
      data_fusion_communications::GetObjectsSrv::Response &rs);

  bool geotiffSericeCb(
    data_fusion_communications::DatafusionGeotiffSrv::Request &req,
      data_fusion_communications::DatafusionGeotiffSrv::Response &res);

  bool getMarkersServiceCb(
    data_fusion_communications::GetMarkersSrv::Request& rq,
      data_fusion_communications::GetMarkersSrv::Response &rs);
 
  virtual void startTransition(int newState);
 
 private:

  ros::NodeHandle nh_;
  ros::Subscriber holeDirectionSubscrider_;
  ros::Subscriber holePositionSubscrider_;
  ros::Subscriber faceDirectionSubscrider_;
  ros::Subscriber tpaDirectionSubscrider_;
  ros::Subscriber mlxDirectionSubscrider_;
  ros::Subscriber victimVerificationSubscrider_;
  ros::Subscriber qrSubscrider_;
  ros::Subscriber hazmatSubscrider_;
  ros::Subscriber currentVictimSubscrider_;
  
  ros::Subscriber mapSubscriber_;

  ros::ServiceServer flushService_;
  ros::ServiceServer getObjectsService_;
  ros::ServiceServer geotiffService_;
  ros::ServiceServer getMarkersService_;

  tf::TransformBroadcaster currentVictimBroadcaster_;

  ros::Timer currentVictimTimer_;

  boost::shared_ptr<GetVictimsServer> victimsServer_;
  boost::shared_ptr<DeleteVictimServer> deleteVictimServer_;
  boost::shared_ptr<ValidateCurrentHoleServer> validateCurrentHoleServer_;

  Map map_;

  // save for geotiff
  int prevxMin;
  int prevyMin;

  boost::scoped_ptr<PoseFinder> poseFinderPtr_;

  HoleListPtr holes_;
  QrListPtr qrs_;
  HazmatListPtr hazmats_;
  TpaListPtr tpas_;

  ObjectHandlerPtr objectHandler_;
  VictimHandlerPtr victimHandler_;

  int currentVictimId;

  dynamic_reconfigure::Server<pandora_alert_handler::AlertHandlerConfig> 
    dynReconfserver_;

  int curState;
  bool eraseHolesQrs;

};

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_ALERT_HANDLER_H_
