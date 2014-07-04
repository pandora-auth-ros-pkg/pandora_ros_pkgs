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
#include "pandora_data_fusion_msgs/DatafusionGeotiffSrv.h"
#include "pandora_data_fusion_msgs/GetMarkersSrv.h"

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/QRAlertsVectorMsg.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"
#include "vision_communications/DataMatrixAlertsVectorMsg.h"
#include "vision_communications/LandoltcAlertsVectorMsg.h"
#include "pandora_common_msgs/GeneralAlertMsg.h"

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
        /*  Alert-concerned Subscribers  */

        void holeDirectionAlertCallback(
            const vision_communications::HolesDirectionsVectorMsg& msg);
        void thermalDirectionAlertCallback(
            const pandora_common_msgs::GeneralAlertMsg& msg);
        void hazmatAlertCallback(
            const vision_communications::HazmatAlertsVectorMsg& msg);
        void qrAlertCallback(const vision_communications::QRAlertsVectorMsg& msg);
        void landoltcAlertCallback(
            const vision_communications::LandoltcAlertsVectorMsg& msg);
        void dataMatrixAlertCallback(
            const vision_communications::DataMatrixAlertsVectorMsg& msg);
        template <class ObjectType>
          void objectDirectionAlertCallback(
              const pandora_common_msgs::GeneralAlertMsg& msg);

        /*  Victim-concerned Subscribers  */

        /*  Map Subsriber Callback - Communication with SLAM  */

        /**
         * @brief Communication with SLAM. Gets current global map.
         * @param msg [const nav_msgs::OccupancyGridConstPtr&] Contains map info.
         * @return void
         */
        void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);

        /*  Map Visualization Callbacks  */

        bool getObjectsServiceCb(
            pandora_data_fusion_msgs::GetObjectsSrv::Request& rq,
            pandora_data_fusion_msgs::GetObjectsSrv::Response &rs);

        bool geotiffServiceCb(
            pandora_data_fusion_msgs::DatafusionGeotiffSrv::Request &req,
            pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response &res);

        bool getMarkersServiceCb(
            pandora_data_fusion_msgs::GetMarkersSrv::Request& rq,
            pandora_data_fusion_msgs::GetMarkersSrv::Response &rs);

        /*  Services Callbacks  */

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

        ros::Subscriber holeDirectionSubscriber_;
        ros::Subscriber faceDirectionSubscriber_;
        ros::Subscriber co2DirectionSubscriber_;
        ros::Subscriber motionDirectionSubscriber_;
        ros::Subscriber thermalDirectionSubscriber_;
        ros::Subscriber soundDirectionSubscriber_;
        ros::Subscriber qrSubscriber_;
        ros::Subscriber hazmatSubscriber_;
        ros::Subscriber landoltcSubscriber_;
        ros::Subscriber dataMatrixSubscriber_;

        ros::Subscriber mapSubscriber_;

        ros::ServiceServer flushService_;
        ros::ServiceServer getObjectsService_;
        ros::ServiceServer geotiffService_;
        ros::ServiceServer getMarkersService_;

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
        HoleListPtr holes_;
        QrListPtr qrs_;
        MotionListPtr motions_;
        FaceListPtr faces_;
        SoundListPtr sounds_;
        Co2ListPtr co2s_;
        HazmatListPtr hazmats_;
        ThermalListPtr thermals_;
        LandoltcListPtr landoltcs_;
        DataMatrixListPtr dataMatrices_;

        //!< The unvisited victims list
        VictimListPtr victimsToGo_;
        //!< The visited victims list
        VictimListPtr victimsVisited_;

        ObjectFactoryPtr objectFactory_;
        ObjectHandlerPtr objectHandler_;
        VictimHandlerPtr victimHandler_;
    };

    template <class ObjectType>
      void AlertHandler::objectDirectionAlertCallback(
          const pandora_common_msgs::GeneralAlertMsg& msg)
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

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_ALERT_HANDLER_H
