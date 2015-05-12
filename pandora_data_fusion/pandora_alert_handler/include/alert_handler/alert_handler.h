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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef ALERT_HANDLER_ALERT_HANDLER_H
#define ALERT_HANDLER_ALERT_HANDLER_H

#include <string>
#include <boost/utility.hpp>
#include <map>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int16.h>

#include "pandora_data_fusion_msgs/WorldModelMsg.h"
#include "pandora_data_fusion_msgs/VictimInfoMsg.h"
#include "pandora_data_fusion_msgs/DeleteVictimAction.h"
#include "pandora_data_fusion_msgs/ValidateVictimAction.h"
#include "pandora_data_fusion_msgs/GetObjectsSrv.h"
#include "pandora_data_fusion_msgs/GeotiffSrv.h"
#include "pandora_data_fusion_msgs/GetMarkersSrv.h"

#include "pandora_vision_msgs/HoleDirectionAlertVector.h"
#include "pandora_vision_msgs/QRAlertVector.h"
#include "pandora_vision_msgs/HazmatAlertVector.h"
#include "pandora_vision_msgs/DataMatrixAlertVector.h"
#include "pandora_vision_msgs/LandoltcAlertVector.h"
#include "pandora_common_msgs/GeneralAlertVector.h"

#include "pandora_alert_handler/AlertHandlerConfig.h"
#include "alert_handler/defines.h"
#include "alert_handler/object_list.h"
#include "alert_handler/victim_list.h"
#include "alert_handler/object_factory.h"
#include "alert_handler/object_handler.h"
#include "alert_handler/victim_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    //!< Type Definitions
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;
    typedef actionlib::SimpleActionServer
      <pandora_data_fusion_msgs::DeleteVictimAction> DeleteVictimServer;
    typedef actionlib::SimpleActionServer
      <pandora_data_fusion_msgs::ValidateVictimAction>
      ValidateVictimServer;
    typedef boost::shared_ptr<const ValidateVictimServer::Goal> GoalConstPtr;

    class AlertHandler : private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         */
        explicit AlertHandler(const std::string& ns);

        /* Victim-concerned Goal Callbacks */

        /**
         * @brief Client is Agent. Order to delete Victim.
         * @return void
         */
        void deleteVictimCallback();

        /**
         * @brief Client is Agent. Order to validate current victim (identification mode).
         * @return void
         */
        void validateVictimCallback();

        /*  Dynamic Reconfiguration Callback  */
        void dynamicReconfigCallback(
            const ::pandora_alert_handler::AlertHandlerConfig &config,
            uint32_t level);

      private:
        /**
         * @brief Templated subscriber for all objects
         */
        template <class MsgType, class ClassType>
          void setSubscriber(const std::string name,
            void (ClassType::*callback)(MsgType));

        /*  Alert-concerned Subscribers  */

        void hazmatAlertCallback(
            const pandora_vision_msgs::HazmatAlertVector& msg);
        void qrAlertCallback(
            const pandora_vision_msgs::QRAlertVector& msg);
        void landoltcAlertCallback(
            const pandora_vision_msgs::LandoltcAlertVector& msg);
        void dataMatrixAlertCallback(
            const pandora_vision_msgs::DataMatrixAlertVector& msg);

        /*  Victim-concerned Subscribers  */

        void holeAlertCallback(
            const pandora_vision_msgs::HoleDirectionAlertVector& msg);
        void thermalAlertCallback(
            const pandora_common_msgs::GeneralAlertVector& msg);
        template <class ObjectType>
          void victimAlertCallback(
              const pandora_common_msgs::GeneralAlertVector& msg);

        /*  Map Subsriber Callback - Communication with SLAM  */

        /**
         * @brief Communication with SLAM. Gets current global map.
         * @param msg [const nav_msgs::OccupancyGridConstPtr&] Contains map info.
         * @return void
         */
        void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);

        /*  Map Visualization Callbacks  */

        bool geotiffServiceCb(
            pandora_data_fusion_msgs::GeotiffSrv::Request &req,
            pandora_data_fusion_msgs::GeotiffSrv::Response &res);

        bool getMarkersServiceCb(
            pandora_data_fusion_msgs::GetMarkersSrv::Request& rq,
            pandora_data_fusion_msgs::GetMarkersSrv::Response &rs);

        /*  Services Callbacks  */

        bool getObjectsServiceCb(
            pandora_data_fusion_msgs::GetObjectsSrv::Request& rq,
            pandora_data_fusion_msgs::GetObjectsSrv::Response &rs);

        bool flushQueues(
            std_srvs::Empty::Request& rq,
            std_srvs::Empty::Response& rs);

        /**
         * @brief Function that posts objects' transformations periodically.
         * Triggered by a timer.
         * @param event [ros::TimerEvent const&]
         * @return void
         */
        void tfPublisherCallback(const ros::TimerEvent& event);

        /**
         * @brief Broadcasts transformations from /world according to
         * stamped poses given.
         * @param poseVector [PoseStampedVector const&] vector containing pose info
         * @return void
         */
        void broadcastPoseVector(const PoseStampedVector& poseVector);

        /**
         * @brief Takes info from VictimsToGo_ and publishes it to the Agent.
         * @return void
         */
        void publishVictims();

        void initRosInterfaces();

      private:
        NodeHandlePtr nh_;

        //!< Holds the subscribers using a key
        std::map<std::string, ros::Subscriber> subscribers_;

        ros::ServiceServer getMarkersService_;
        ros::ServiceServer geotiffService_;

        ros::ServiceServer flushService_;
        ros::ServiceServer getObjectsService_;

        ros::Publisher worldModelPublisher_;

        tf::TransformBroadcaster objectsBroadcaster_;
        ros::Timer tfPublisherTimer_;

        boost::shared_ptr<DeleteVictimServer> deleteVictimServer_;
        boost::shared_ptr<ValidateVictimServer> validateVictimServer_;

        dynamic_reconfigure::Server< ::pandora_alert_handler::AlertHandlerConfig >
          dynReconfServer_;

        MapPtr map_;

        //!< save for geotiff
        int prevxMin;
        int prevyMin;

        //!< The alerts list
        QrListPtr qrs_;
        HazmatListPtr hazmats_;
        LandoltcListPtr landoltcs_;
        DataMatrixListPtr dataMatrices_;

        HoleListPtr holes_;
        ThermalListPtr thermals_;
        MotionListPtr motions_;
        VictimImageListPtr victimImages_;
        SoundListPtr sounds_;
        Co2ListPtr co2s_;

        //!< The unvisited victims list
        VictimListPtr victimsToGo_;
        //!< The visited victims list
        VictimListPtr victimsVisited_;

        ObjectFactoryPtr objectFactory_;
        ObjectHandlerPtr objectHandler_;
        VictimHandlerPtr victimHandler_;

    };

      template <class ObjectType>
        void AlertHandler::victimAlertCallback(
            const pandora_common_msgs::GeneralAlertVector& msg)
      {
        if (map_->data.size() == 0)
          return;

        ROS_INFO_STREAM_NAMED("ALERT_HANDLER_ALERT_CALLBACK",
            ObjectType::getObjectType() << " ALERT ARRIVED!");

        typename ObjectType::PtrVectorPtr objectsVectorPtr;
        try
        {
          objectsVectorPtr = objectFactory_->makeObjects<ObjectType>(msg);
        }
        catch (TfException ex)
        {
          ROS_ERROR("[ALERT_HANDLER %d] %s",  __LINE__, ex.what());
          return;
        }

        objectHandler_->handleObjects<ObjectType>(objectsVectorPtr);

        victimHandler_->inspect();

        publishVictims();
      }

      /**
       * @brief tamplated function responsible for initialising each one of the
       * node's subscribers.
       * @param name [std::string] string with the name of the subscriber and
       * of the yaml param
       * @param callback [void] pointer to the Callback of the subscriber
       * @return void
       */
      template <class MsgType, class ClassType>
        void AlertHandler::setSubscriber(const std::string name,
          void (ClassType::*callback)(MsgType))
        {
          std::string param;
          if (nh_->getParam("subscribed_topic_names/" + name, param))
          {
            ros::Subscriber sub;
            sub = nh_->subscribe(param, 1, callback, this);
            //!< Store the subscriber to the std::map
            subscribers_[name] = sub;
          }
          else
          {
            ROS_FATAL("[ALERT_HANDLER] %s topic name param not found", name.c_str());
            ROS_BREAK();
          }
        }

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_ALERT_HANDLER_H
