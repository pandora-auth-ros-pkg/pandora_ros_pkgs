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

#include "alert_handler/alert_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    AlertHandler::AlertHandler(const std::string& ns)
    {
      nh_.reset( new ros::NodeHandle(ns) );
      map_.reset( new Map );

      holes_.reset( new ObjectList<Hole> );
      qrs_.reset( new ObjectList<Qr> );
      hazmats_.reset( new ObjectList<Hazmat> );
      thermals_.reset( new ObjectList<Thermal> );
      faces_.reset( new ObjectList<Face> );
      motions_.reset( new ObjectList<Motion> );
      sounds_.reset( new ObjectList<Sound> );
      co2s_.reset( new ObjectList<Co2> );
      landoltcs_.reset( new ObjectList<Landoltc> );
      dataMatrices_.reset( new ObjectList<DataMatrix> );

      Hole::setList(holes_);
      Qr::setList(qrs_);
      Hazmat::setList(hazmats_);
      Thermal::setList(thermals_);
      Face::setList(faces_);
      Motion::setList(motions_);
      Sound::setList(sounds_);
      Co2::setList(co2s_);
      Landoltc::setList(landoltcs_);
      DataMatrix::setList(dataMatrices_);

      victimsToGo_.reset( new VictimList );
      victimsVisited_.reset( new VictimList );

      std::string mapType;
      nh_->getParam("map_type", mapType);
      objectFactory_.reset( new ObjectFactory(map_, mapType) );
      objectHandler_.reset( new ObjectHandler(nh_, victimsToGo_, victimsVisited_) );
      victimHandler_.reset( new VictimHandler(victimsToGo_, victimsVisited_) );

      initRosInterfaces();
    }

    void AlertHandler::publishVictims()
    {
      pandora_data_fusion_msgs::WorldModelMsg worldModelMsg;
      victimHandler_->getVictimsInfo(&worldModelMsg);
      worldModelPublisher_.publish(worldModelMsg);
    }

    void AlertHandler::initRosInterfaces()
    {
      std::string param; 

      //!< alert-concerned subscribers

      if (nh_->getParam("subscribed_topic_names/holeDirection", param))
      {
        holeDirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::holeDirectionAlertCallback, this);
      }
      else
      {
        ROS_FATAL("holeDirection topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/thermalDirection", param))
      {
        thermalDirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::objectDirectionAlertCallback< Thermal >, this);
      }
      else
      {
        ROS_FATAL("thermalDirection topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/qr", param))
      {
        qrSubscriber_ = nh_->subscribe(param, 1, &AlertHandler::qrAlertCallback, this);
      }
      else
      {
        ROS_FATAL("qr topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/hazmat", param))
      {
        hazmatSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::hazmatAlertCallback, this);
      }
      else
      {
        ROS_FATAL("hazmat topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/faceDirection", param))
      {
        faceDirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::objectDirectionAlertCallback< Face >, this);
      }
      else
      {
        ROS_FATAL("faceDirection topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/co2Direction", param))
      {
        co2DirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::objectDirectionAlertCallback< Co2 >, this);
      }
      else
      {
        ROS_FATAL("co2Direction topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/motionDirection", param))
      {
        motionDirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::objectDirectionAlertCallback< Motion >, this);
      }
      else
      {
        ROS_FATAL("motionDirection topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/soundDirection", param))
      {
        soundDirectionSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::objectDirectionAlertCallback< Sound >, this);
      }
      else
      {
        ROS_FATAL("soundDirection topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/landoltc", param))
      {
        landoltcSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::landoltcAlertCallback, this);
      }
      else
      {
        ROS_FATAL("landoltc topic name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("subscribed_topic_names/dataMatrix", param))
      {
        dataMatrixSubscriber_ = nh_->subscribe(param, 
            1, &AlertHandler::dataMatrixAlertCallback, this);
      }
      else
      {
        ROS_FATAL("dataMatrix topic name param not found");
        ROS_BREAK();
      }

      //!< map subscriber

      if (nh_->getParam("subscribed_topic_names/map", param))
      {
        mapSubscriber_ = nh_->subscribe(param, 1, &AlertHandler::updateMap, this);
      }
      else
      {
        ROS_FATAL("map topic name param not found");
        ROS_BREAK();
      }

      //!< publishers

      if (nh_->getParam("published_topic_names/world_model", param))
      {
        worldModelPublisher_ = nh_->
          advertise<pandora_data_fusion_msgs::WorldModelMsg>(param, 10);
      }
      else
      {
        ROS_FATAL("victims topic name param not found");
        ROS_BREAK();
      }

      //!< action servers

      if (nh_->getParam("action_server_names/delete_victim", param))
      {
        deleteVictimServer_.reset(new DeleteVictimServer(*nh_, param, false));
      } 
      else
      {
        ROS_FATAL("delete_victim action name param not found");
        ROS_BREAK();
      }
      deleteVictimServer_->registerGoalCallback(
          boost::bind( &AlertHandler::deleteVictimCallback, this) );
      deleteVictimServer_->start();

      if (nh_->getParam("action_server_names/validate_victim", param))
      {
        validateVictimServer_.reset(
            new ValidateVictimServer(*nh_, param, false));
      }
      else
      {
        ROS_FATAL("validate_victim action name param not found");
        ROS_BREAK();
      }
      validateVictimServer_->registerGoalCallback(
          boost::bind( &AlertHandler::validateVictimCallback, this) );
      validateVictimServer_->start();

      //!< service servers

      if (nh_->getParam("service_server_names/flush_queues", param))
      {
        flushService_ = nh_->advertiseService(param, 
            &AlertHandler::flushQueues, this);
      }
      else
      {
        ROS_FATAL("flush_queues service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/get_objects", param))
      {
        getObjectsService_ = nh_->advertiseService(param, 
            &AlertHandler::getObjectsServiceCb, this);
      }
      else 
      {
        ROS_FATAL("get_objects service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/get_markers", param))
      {
        getMarkersService_ = nh_->advertiseService(param, 
            &AlertHandler::getMarkersServiceCb, this);
      }
      else
      {
        ROS_FATAL("get_markers service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/geotiff", param))
      {
        geotiffService_ = nh_->advertiseService(param, 
            &AlertHandler::geotiffServiceCb, this);
      }
      else
      {
        ROS_FATAL("geotiffSrv service name param not found");
        ROS_BREAK();
      }

      //!< dynamic reconfigure server

      dynReconfServer_.setCallback(boost::bind(
            &AlertHandler::dynamicReconfigCallback, this, _1, _2));

      //!< timers

      tfPublisherTimer_ = nh_->createTimer(ros::Duration(0.1), 
          &AlertHandler::tfPublisherCallback, this);
    }  

    //!< Alert-concerned callbacks

    void AlertHandler::holeDirectionAlertCallback(
        const vision_communications::HolesDirectionsVectorMsg& msg)
    {
      if (map_->data.size() == 0)
        return;

      ROS_DEBUG_STREAM_NAMED("ALERT_HANDLER_ALERT_CALLBACK", 
          Hole::getObjectType() << " ALERT ARRIVED!");

      HolePtrVectorPtr holesVectorPtr;
      try
      {
        holesVectorPtr = objectFactory_->makeHoles(msg);
      }
      catch (TfException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
        return;
      }

      objectHandler_->handleHoles(holesVectorPtr, objectFactory_->getTransform());

      victimHandler_->notify();

      publishVictims();
    }

    void AlertHandler::hazmatAlertCallback(
        const vision_communications::HazmatAlertsVectorMsg& msg)
    {
      if (map_->data.size() == 0)
        return;

      ROS_DEBUG_STREAM_NAMED("ALERT_HANDLER_ALERT_CALLBACK", 
          Hazmat::getObjectType() << " ALERT ARRIVED!");

      HazmatPtrVectorPtr hazmatsVectorPtr;
      try
      {
        hazmatsVectorPtr = objectFactory_->makeHazmats(msg);
      }
      catch (TfException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
        return;
      }

      objectHandler_->handleObjects<Hazmat>(hazmatsVectorPtr);
    }

    void AlertHandler::qrAlertCallback(
        const vision_communications::QRAlertsVectorMsg& msg)
    {
      if (map_->data.size() == 0)
        return;

      ROS_DEBUG_STREAM_NAMED("ALERT_HANDLER_ALERT_CALLBACK", 
          Qr::getObjectType() << " ALERT ARRIVED!");  

      QrPtrVectorPtr qrsVectorPtr;
      try
      {
        qrsVectorPtr = objectFactory_->makeQrs(msg);
      }
      catch (TfException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
        return;
      }

      objectHandler_->handleQrs(qrsVectorPtr);
    }

    void AlertHandler::landoltcAlertCallback(
        const vision_communications::LandoltcAlertsVectorMsg& msg)
    {
      if (map_->data.size() == 0)
        return;

      ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "LANDOLTC ALERT ARRIVED!");

      LandoltcPtrVectorPtr landoltcsVectorPtr;
      try
      {
        landoltcsVectorPtr = objectFactory_->makeLandoltcs(msg);
      }
      catch (TfException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
        return;
      }

      objectHandler_->handleObjects<Landoltc>(landoltcsVectorPtr);
    }

    void AlertHandler::dataMatrixAlertCallback(
        const vision_communications::DataMatrixAlertsVectorMsg& msg)
    {
      if (map_->data.size() == 0)
        return;

      ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "DATA MATRIX ALERT ARRIVED!");

      DataMatrixPtrVectorPtr dataMatricesVectorPtr;
      try
      {
        dataMatricesVectorPtr = objectFactory_->makeDataMatrices(msg);
      }
      catch (TfException ex)
      {
        ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
        return;
      }

      objectHandler_->handleObjects<DataMatrix>(dataMatricesVectorPtr);
    }

    //!< Other Callbacks

    void AlertHandler::tfPublisherCallback(const ros::TimerEvent& event)
    {
      PoseStampedVector objectsPosesStamped; 
      qrs_->getObjectsPosesStamped(&objectsPosesStamped);
      hazmats_->getObjectsPosesStamped(&objectsPosesStamped);
      thermals_->getObjectsPosesStamped(&objectsPosesStamped);
      faces_->getObjectsPosesStamped(&objectsPosesStamped);
      motions_->getObjectsPosesStamped(&objectsPosesStamped);
      sounds_->getObjectsPosesStamped(&objectsPosesStamped);
      co2s_->getObjectsPosesStamped(&objectsPosesStamped);
      landoltcs_->getObjectsPosesStamped(&objectsPosesStamped);
      dataMatrices_->getObjectsPosesStamped(&objectsPosesStamped);
      victimsToGo_->getObjectsPosesStamped(&objectsPosesStamped);

      broadcastPoseVector(objectsPosesStamped); 
    }

    void AlertHandler::broadcastPoseVector(const PoseStampedVector& poseVector)
    {
      for(PoseStampedVector::const_iterator it = poseVector.begin(); 
          it != poseVector.end(); ++it)
      {
        tf::Quaternion tfQuaternion(it->pose.orientation.x, 
            it->pose.orientation.y, 
            it->pose.orientation.z, 
            it->pose.orientation.w);
        tf::Vector3 vec(it->pose.position.x, 
            it->pose.position.y, 
            it->pose.position.z);
        tf::Transform tfObject(tfQuaternion, vec);

        ROS_DEBUG_NAMED("ALERT_HANDLER_TF_PUBLISHER",
            "Publishing tf : %f , %f , %f , world to %s ", vec[0], vec[1], vec[2],
            it->header.frame_id.c_str());

        objectsBroadcaster_.sendTransform( 
            tf::StampedTransform(tfObject, it->header.stamp,
              "/world", it->header.frame_id ) );
      }
    }

    void AlertHandler::deleteVictimCallback()
    {
      int victimId = deleteVictimServer_->acceptNewGoal()->victimId;
      bool deleted = victimHandler_->deleteVictim(victimId);
      publishVictims();
      if(!deleted)
        deleteVictimServer_->setAborted();
      deleteVictimServer_->setSucceeded();
    }

    void AlertHandler::validateVictimCallback()
    {
      GoalConstPtr goal = validateVictimServer_->acceptNewGoal();
      bool validated = victimHandler_->validateVictim(goal->victimId, goal->victimValid);
      publishVictims();
      if(!validated)
        validateVictimServer_->setAborted();
      validateVictimServer_->setSucceeded();
    }

    void AlertHandler::updateMap(const nav_msgs::OccupancyGridConstPtr& msg)
    {
      prevxMin = msg->info.origin.position.x / msg->info.resolution 
        + msg->info.width / 2;
      prevyMin = msg->info.origin.position.y / msg->info.resolution 
        + msg->info.height / 2;

      *map_ = *msg;
    }

    void AlertHandler::dynamicReconfigCallback(
        const ::pandora_alert_handler::AlertHandlerConfig& config, uint32_t level)
    {
      objectFactory_->dynamicReconfigForward( config.occupiedCellThres,
          config.highThres, config.lowThres, 
          config.orientationCircle, config.orientationDist); 

      Hole::setObjectScore(-1);
      Hole::setProbabilityThres(config.holeMinProbability);
      Hole::setDistanceThres(config.holeMinDistance);
      Hole::getFilterModel()->initializeSystemModel(config.holeSystemNoiseSD);
      Hole::getFilterModel()->initializeMeasurementModel(config.holeMeasurementSD);

      Hazmat::setObjectScore(config.hazmatScore);
      Hazmat::setProbabilityThres(config.hazmatMinProbability);
      Hazmat::setDistanceThres(config.hazmatMinDistance);
      Hazmat::getFilterModel()->initializeSystemModel(config.hazmatSystemNoiseSD);
      Hazmat::getFilterModel()->initializeMeasurementModel(config.hazmatMeasurementSD);

      Qr::setObjectScore(config.qrScore);
      Qr::setProbabilityThres(config.qrMinProbability);
      Qr::setDistanceThres(config.qrMinDistance);
      Qr::getFilterModel()->initializeSystemModel(config.qrSystemNoiseSD);
      Qr::getFilterModel()->initializeMeasurementModel(config.qrMeasurementSD);

      DataMatrix::setObjectScore(config.dataMatrixScore);
      DataMatrix::setProbabilityThres(config.dataMatrixMinProbability);
      DataMatrix::setDistanceThres(config.dataMatrixMinDistance);
      DataMatrix::getFilterModel()->initializeSystemModel(config.dataMatrixSystemNoiseSD);
      DataMatrix::getFilterModel()->initializeMeasurementModel(config.dataMatrixMeasurementSD);

      Landoltc::setObjectScore(config.landoltcScore);
      Landoltc::setProbabilityThres(config.landoltcMinProbability);
      Landoltc::setDistanceThres(config.landoltcMinDistance);
      Landoltc::getFilterModel()->initializeSystemModel(config.landoltcSystemNoiseSD);
      Landoltc::getFilterModel()->initializeMeasurementModel(config.landoltcMeasurementSD);

      Thermal::setObjectScore(config.thermalScore);
      Thermal::setProbabilityThres(config.thermalMinProbability);
      Thermal::setDistanceThres(config.thermalMinDistance);
      Thermal::getFilterModel()->initializeSystemModel(config.thermalSystemNoiseSD);
      Thermal::getFilterModel()->initializeMeasurementModel(config.thermalMeasurementSD);

      Face::setObjectScore(config.faceScore);
      Face::setProbabilityThres(config.faceMinProbability);
      Face::setDistanceThres(config.faceMinDistance);
      Face::getFilterModel()->initializeSystemModel(config.faceSystemNoiseSD);
      Face::getFilterModel()->initializeMeasurementModel(config.faceMeasurementSD);

      Motion::setObjectScore(config.motionScore);
      Motion::setProbabilityThres(config.motionMinProbability);
      Motion::setDistanceThres(config.motionMinDistance);
      Motion::getFilterModel()->initializeSystemModel(config.motionSystemNoiseSD);
      Motion::getFilterModel()->initializeMeasurementModel(config.motionMeasurementSD);

      Sound::setObjectScore(config.soundScore);
      Sound::setProbabilityThres(config.soundMinProbability);
      Sound::setDistanceThres(config.soundMinDistance);
      Sound::getFilterModel()->initializeSystemModel(config.soundSystemNoiseSD);
      Sound::getFilterModel()->initializeMeasurementModel(config.soundMeasurementSD);

      Co2::setObjectScore(config.co2Score);
      Co2::setProbabilityThres(config.co2MinProbability);
      Co2::setDistanceThres(config.co2MinDistance);
      Co2::getFilterModel()->initializeSystemModel(config.co2SystemNoiseSD);
      Co2::getFilterModel()->initializeMeasurementModel(config.co2MeasurementSD);

      objectHandler_->updateParams(config.sensorRange, config.clusterRadius);

      victimHandler_->updateParams(config.clusterRadius , config.sameVictimRadius);
    }

    ///////////////////////////////////////////////////////

    bool AlertHandler::getObjectsServiceCb(
        pandora_data_fusion_msgs::GetObjectsSrv::Request& rq,
        pandora_data_fusion_msgs::GetObjectsSrv::Response &rs)
    {
      holes_->getObjectsPosesStamped(&rs.holes);
      qrs_->getObjectsPosesStamped(&rs.qrs);
      hazmats_->getObjectsPosesStamped(&rs.hazmats);
      thermals_->getObjectsPosesStamped(&rs.thermals);
      faces_->getObjectsPosesStamped(&rs.faces);
      motions_->getObjectsPosesStamped(&rs.motions);
      sounds_->getObjectsPosesStamped(&rs.sounds);
      co2s_->getObjectsPosesStamped(&rs.co2s);
      landoltcs_->getObjectsPosesStamped(&rs.landoltcs);
      dataMatrices_->getObjectsPosesStamped(&rs.dataMatrices);

      victimHandler_->getVictimsPosesStamped(&rs.victimsToGo, &rs.victimsVisited);

      return true;
    }

    bool AlertHandler::getMarkersServiceCb(
        pandora_data_fusion_msgs::GetMarkersSrv::Request& rq,
        pandora_data_fusion_msgs::GetMarkersSrv::Response &rs)
    {
      holes_->getVisualization(&rs.holes);
      qrs_->getVisualization(&rs.hazmats);
      hazmats_->getVisualization(&rs.qrs);
      thermals_->getVisualization(&rs.thermals);
      faces_->getVisualization(&rs.faces);
      motions_->getVisualization(&rs.motions);
      sounds_->getVisualization(&rs.sounds);
      co2s_->getVisualization(&rs.co2s);
      landoltcs_->getVisualization(&rs.landoltcs);
      dataMatrices_->getVisualization(&rs.dataMatrices);

      victimHandler_->getVisualization(&rs.victimsVisited, &rs.victimsToGo);

      return true;
    }

    bool AlertHandler::geotiffServiceCb(
        pandora_data_fusion_msgs::DatafusionGeotiffSrv::Request &req,
        pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response &res)
    {
      qrs_->fillGeotiff(&res);
      hazmats_->fillGeotiff(&res);
      victimHandler_->fillGeotiff(&res);

      for (int i = 0; i < res.victimsx.size(); i++)
      {
        res.victimsx[i] -= prevxMin;
        res.victimsy[i] -= prevyMin;
      }

      for (int i = 0; i < res.qrx.size(); i++)
      {
        res.qrx[i] -=  prevxMin;
        res.qry[i] -=  prevyMin;
      }

      for (int i = 0; i < res.hazmatx.size(); i++)
      {
        res.hazmatx[i] -=  prevxMin;
        res.hazmaty[i] -=  prevyMin;
      }

      return true;
    }

    bool AlertHandler::flushQueues(
        std_srvs::Empty::Request& rq,
        std_srvs::Empty::Response &rs)
    {
      ROS_INFO_NAMED("ALERT_HANDLER_FLUSH_SERVICE", "Flushing lists!");
      holes_->clear();
      qrs_->clear();
      hazmats_->clear();
      thermals_->clear();
      faces_->clear();
      motions_->clear();
      sounds_->clear();
      co2s_->clear();
      landoltcs_->clear();
      dataMatrices_->clear();
      victimHandler_->flush();
      return true;
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

