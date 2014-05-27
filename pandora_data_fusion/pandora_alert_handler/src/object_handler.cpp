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

#include "alert_handler/object_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectHandler::ObjectHandler(const VictimListConstPtr& victimsToGoList,
        const VictimListConstPtr& victimsVisitedList) : 
      victimsToGoList_(victimsToGoList),
      victimsVisitedList_(victimsVisitedList)
    {
      Hole::setObjectType("HOLE");
      Hazmat::setObjectType("HAZMAT");
      Qr::setObjectType("QR");
      Thermal::setObjectType("THERMAL");
      Face::setObjectType("FACE");
      Motion::setObjectType("MOTION");
      Sound::setObjectType("SOUND");
      Co2::setObjectType("CO2");
      Landoltc::setObjectType("LANDOLTC");
      DataMatrix::setObjectType("DATA_MATRIX");

      roboCupScore_ = 0;

      std::string param;

      if (ros::param::get("published_topic_names/qr_notification", param))
      {
        qrPublisher_ = ros::NodeHandle().
          advertise<pandora_data_fusion_msgs::QrNotificationMsg>(param, 10);
      }
      else
      {
        ROS_FATAL("qr_notification topic name param not found");
        ROS_BREAK();
      }

      if (ros::param::get("published_topic_names/robocup_score", param))
      {
        scorePublisher_ = ros::NodeHandle().
          advertise<std_msgs::Int32>(param, 10);
      }
      else
      {
        ROS_FATAL("robocup_score topic name param not found");
        ROS_BREAK();
      }
    }

    void ObjectHandler::handleHoles(const HolePtrVectorPtr& newHoles,
        const tf::Transform& transform)
    {
      keepValidHoles(newHoles, transform);

      for(int ii = 0; ii < newHoles->size(); ++ii)
      {
        Hole::getList()->add( newHoles->at(ii) );
      }
    }

    void ObjectHandler::handleQrs(const QrPtrVectorPtr& newQrs) 
    {
      for(int ii = 0; ii < newQrs->size(); ++ii)
      {
        int qrScore = Qr::getList()->add(newQrs->at(ii));
        if(qrScore)
        {
          pandora_data_fusion_msgs::QrNotificationMsg newQrNofifyMsg;
          newQrNofifyMsg.header.stamp = newQrs->at(ii)->getTimeFound();
          newQrNofifyMsg.x = newQrs->at(ii)->getPose().position.x;
          newQrNofifyMsg.y = newQrs->at(ii)->getPose().position.y;
          newQrNofifyMsg.content = newQrs->at(ii)->getContent();
          qrPublisher_.publish(newQrNofifyMsg);
          std_msgs::Int32 updateScoreMsg;
          roboCupScore_ += qrScore;
          updateScoreMsg.data = roboCupScore_;
          scorePublisher_.publish(updateScoreMsg);
        }
      }
    }

    void ObjectHandler::keepValidHoles(const HolePtrVectorPtr& holesPtr,
        const tf::Transform& transform)
    {
      tf::Vector3 origin = transform.getOrigin();
      geometry_msgs::Point framePosition = Utils::vector3ToPoint(origin);

      HolePtrVector::iterator iter = holesPtr->begin();

      while(iter != holesPtr->end())
      {
        bool invalid = !Utils::arePointsInRange((*iter)->getPose().position,
            framePosition, SENSOR_RANGE );

        if(invalid)
        {
          ROS_DEBUG_NAMED("object_handler",
              "[OBJECT_HANDLER %d] Deleting not valid hole...", __LINE__);
          iter = holesPtr->erase(iter);
        }
        else
        {
          ++iter;
        }
      }
    }

    void ObjectHandler::updateParams(float sensor_range, float victim_cluster_radius)
    {
      SENSOR_RANGE = sensor_range;
      VICTIM_CLUSTER_RADIUS = victim_cluster_radius;
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

