// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_ALERT_HANDLER_H
#define ALERT_HANDLER_ALERT_HANDLER_H

#include <string>
#include <boost/utility.hpp>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int16.h>

#include "data_fusion_communications/VictimVerificationMsg.h"
#include "data_fusion_communications/GetVictimsAction.h"
#include "data_fusion_communications/VictimInfoMsg.h"
#include "data_fusion_communications/DeleteCurrentVictimAction.h"
#include "data_fusion_communications/ValidateCurrentHoleAction.h"
#include "data_fusion_communications/GetObjectsSrv.h"
#include "data_fusion_communications/DatafusionGeotiffSrv.h"
#include "data_fusion_communications/GetMarkersSrv.h"

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/QRAlertsVectorMsg.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"
#include "vision_communications/HolesPositionsVectorMsg.h"
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"

#include "state_manager/state_client.h"

#include "pandora_alert_handler/AlertHandlerConfig.h"
#include "alert_handler/defines.h"
#include "alert_handler/object_list.h"
#include "alert_handler/object_factory.h"
#include "alert_handler/pose_finder.h"
#include "alert_handler/object_handler.h"
#include "alert_handler/victim_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

typedef actionlib::SimpleActionServer
    <data_fusion_communications::GetVictimsAction> GetVictimsServer;
typedef actionlib::SimpleActionServer
    <data_fusion_communications::DeleteCurrentVictimAction> DeleteVictimServer;
typedef actionlib::SimpleActionServer
    <data_fusion_communications::ValidateCurrentHoleAction>
                                                    ValidateCurrentHoleServer;

class AlertHandler : public StateClient, private boost::noncopyable
{
 public:

  /**
   * @brief Constructor
   * @param ns [std::string const&] Has the namespace of the node.
   */
  explicit AlertHandler(const std::string& ns);

  /* Alert-concerned Subscribers */
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
  void hazmatAlertCallback(
    const vision_communications::HazmatAlertsVectorMsg& msg);
  void qrAlertCallback(const vision_communications::QRAlertsVectorMsg& msg);

  /* Victim-concerned Subscribers */
  /**
   * @brief Communication with VictimFusion (possibly needs to change).
   * @param msg [const data_fusion_communications::VictimVerificationMsg&] Msg
   * @return void
   */
  void victimVerificationCallback(
    const data_fusion_communications::VictimVerificationMsg& msg);
  /**
   * @brief Communication with Navigation (possibly needs to change).
   * @param msg [const std_msgs::Int16&] Msg
   * @return void
   */
  void selectedVictimCallback(const std_msgs::Int16& msg);

  /* MapSubsriber Callback - Communication with SLAM */
  /**
   * @brief Communication with SLAM. Gets current global map.
   * @param msg [const nav_msgs::OccupancyGridConstPtr&] Contains map info.
   * @return void
   */
  void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);

  /* Victim-concerned Goal Callbacks */
  /**
   * @brief Client is Navigation. Sends Victims (possibly needs to change).
   * @return void
   */
  void getVictimsCallback();
  /**
   * @brief Client is FSM. Order to delete Victim.
   * @return void
   */
  void deleteVictimCallback();
  /**
   * @brief Client is FSM. Orded to validate current hole (identification mode).
   * @param msg [const nav_msgs::OccupancyGridConstPtr&] Contains map info.
   * @return void
   */
  void validateCurrentHoleCallback();

  /* Dynamic Reconfiguration Callback */
  void dynamicReconfigCallback(
      const ::pandora_alert_handler::AlertHandlerConfig &config,
        uint32_t level);

  /* Services Callbacks */
  bool flushQueues(
    std_srvs::Empty::Request& rq,
      std_srvs::Empty::Response &rs);
  //!< Map Visualization Callbacks
  bool getObjectsServiceCb(
    data_fusion_communications::GetObjectsSrv::Request& rq,
      data_fusion_communications::GetObjectsSrv::Response &rs);

  bool geotiffServiceCb(
    data_fusion_communications::DatafusionGeotiffSrv::Request &req,
      data_fusion_communications::DatafusionGeotiffSrv::Response &res);

  bool getMarkersServiceCb(
    data_fusion_communications::GetMarkersSrv::Request& rq,
      data_fusion_communications::GetMarkersSrv::Response &rs);

  /* Current Victim Timer Callback */
  void currentVictimTimerCb(const ros::TimerEvent& event);

 private:

  void initRosInterfaces();

  virtual void startTransition(int newState);

 private:

  ros::NodeHandle nh_;
  ros::Subscriber holeDirectionSubscriber_;
  ros::Subscriber holePositionSubscriber_;
  ros::Subscriber faceDirectionSubscriber_;
  ros::Subscriber tpaDirectionSubscriber_;
  ros::Subscriber mlxDirectionSubscriber_;
  ros::Subscriber qrSubscriber_;
  ros::Subscriber hazmatSubscriber_;
  
  ros::Subscriber victimVerificationSubscriber_;
  ros::Subscriber currentVictimSubscriber_;

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

  dynamic_reconfigure::Server< ::pandora_alert_handler::AlertHandlerConfig >
    dynReconfserver_;

  MapPtr map_;

  //!< save for geotiff
  int prevxMin;
  int prevyMin;

  HoleListPtr holes_;
  QrListPtr qrs_;
  HazmatListPtr hazmats_;
  TpaListPtr tpas_;

  ObjectFactoryPtr objectFactory_;
  ObjectHandlerPtr objectHandler_;
  VictimHandlerPtr victimHandler_;

  int currentVictimId;

  int curState;
  bool eraseHolesQrs;

};

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_ALERT_HANDLER_H
