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

#include <string>

#include "alert_handler/alert_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    AlertHandler::AlertHandler(const std::string& ns)
    {
      nh_.reset( new ros::NodeHandle(ns) );
      map_.reset( new Map );

      holes_.reset( new HoleList );
      qrs_.reset( new QrList );
      hazmats_.reset( new HazmatList );
      thermals_.reset( new ThermalList );
      victimImages_.reset( new VictimImageList );
      motions_.reset( new MotionList );
      sounds_.reset( new SoundList );
      co2s_.reset( new Co2List );
      landoltcs_.reset( new LandoltcList );
      dataMatrices_.reset( new DataMatrixList );

      Hole::setList(holes_);
      Qr::setList(qrs_);
      Hazmat::setList(hazmats_);
      Thermal::setList(thermals_);
      VictimImage::setList(victimImages_);
      Motion::setList(motions_);
      Sound::setList(sounds_);
      Co2::setList(co2s_);
      Landoltc::setList(landoltcs_);
      DataMatrix::setList(dataMatrices_);

      std::string param;

      nh_->param<std::string>("object_names/hazmat", param, "hazmat");
      Hazmat::setObjectType(param);
      nh_->param<std::string>("object_names/qr", param, "qr");
      Qr::setObjectType(param);
      nh_->param<std::string>("object_names/landoltc", param, "landoltc");
      Landoltc::setObjectType(param);
      nh_->param<std::string>("object_names/data_matrix", param, "data_matrix");
      DataMatrix::setObjectType(param);

      nh_->param<std::string>("object_names/hole", param, "hole");
      Hole::setObjectType(param);
      nh_->param<std::string>("object_names/thermal", param, "thermal");
      Thermal::setObjectType(param);
      nh_->param<std::string>("object_names/victim_image", param, "victim_image");
      VictimImage::setObjectType(param);
      nh_->param<std::string>("object_names/motion", param, "motion");
      Motion::setObjectType(param);
      nh_->param<std::string>("object_names/sound", param, "sound");
      Sound::setObjectType(param);
      nh_->param<std::string>("object_names/co2", param, "co2");
      Co2::setObjectType(param);

      Hazmat::is3D = true;
      Qr::is3D = true;
      Landoltc::is3D = true;
      DataMatrix::is3D = true;
      Sound::is3D = false;
      Co2::is3D = false;
      Hole::is3D = true;
      Thermal::is3D = true;
      Motion::is3D = true;
      VictimImage::is3D = true;

      Hazmat::isVictimAlert = false;
      Qr::isVictimAlert = false;
      Landoltc::isVictimAlert = false;
      DataMatrix::isVictimAlert = false;
      Sound::isVictimAlert = true;
      Co2::isVictimAlert = true;
      Hole::isVictimAlert = true;
      Thermal::isVictimAlert = true;
      Motion::isVictimAlert = true;
      VictimImage::isVictimAlert = true;

      victimsToGo_.reset( new VictimList );
      victimsVisited_.reset( new VictimList );

      if (!nh_->getParam("map_type", param))
      {
        ROS_FATAL("[ALERT_HANDLER] map_type param not found");
        ROS_BREAK();
      }

      objectFactory_.reset( new ObjectFactory(map_, param) );
      objectHandler_.reset( new ObjectHandler(nh_, victimsToGo_, victimsVisited_) );
      victimHandler_.reset( new VictimHandler(nh_, victimsToGo_, victimsVisited_) );

      if (!nh_->getParam("global_frame", param))
      {
        ROS_FATAL("[ALERT_HANDLER] global_frame param not found");
        ROS_BREAK();
      }
      BaseObject::setGlobalFrame(param);

      initRosInterfaces();
    }

    /**
     * [AlertHandler::publishVictims description]
     */
    void AlertHandler::publishVictims()
    {
      pandora_data_fusion_msgs::WorldModelMsg worldModelMsg;
      victimHandler_->getVictimsInfo(&worldModelMsg);
      worldModelPublisher_.publish(worldModelMsg);
    }

    void AlertHandler::initRosInterfaces()
    {
      // Alert-concerned Subscribers

      setSubscriber<Qr>();
      setSubscriber<Hazmat>();
      setSubscriber<Landoltc>();
      setSubscriber<DataMatrix>();
      setSubscriber<Hole>();
      setSubscriber<Thermal>();
      setSubscriber<VictimImage>();
      setSubscriber<Co2>();
      setSubscriber<Motion>();
      setSubscriber<Sound>();

      // Map Subscriber
      std::string param;
      if (nh_->getParam("subscribed_topic_names/map", param))
      {
        mapSubscriber_ = nh_->subscribe(param, 1, &AlertHandler::updateMap, this);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] map topic name param not found");
        ROS_BREAK();
      }

      // Publishers
      if (nh_->getParam("published_topic_names/world_model", param))
      {
        worldModelPublisher_ = nh_->
          advertise<pandora_data_fusion_msgs::WorldModelMsg>(param, 10);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] world_model topic name param not found");
        ROS_BREAK();
      }

      // Action Servers

      if (nh_->getParam("action_server_names/delete_victim", param))
      {
        deleteVictimServer_.reset(new DeleteVictimServer(*nh_, param, false));
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] delete_victim action name param not found");
        ROS_BREAK();
      }
      deleteVictimServer_->registerGoalCallback(
          boost::bind(&AlertHandler::deleteVictimCallback, this));
      deleteVictimServer_->start();

      if (nh_->getParam("action_server_names/validate_victim", param))
      {
        validateVictimServer_.reset(
            new ValidateVictimServer(*nh_, param, false));
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] validate_victim action name param not found");
        ROS_BREAK();
      }
      validateVictimServer_->registerGoalCallback(
          boost::bind(&AlertHandler::validateVictimCallback, this));
      validateVictimServer_->start();

      // Service Servers
      if (nh_->getParam("service_server_names/flush_queues", param))
      {
        flushService_ = nh_->advertiseService(param,
            &AlertHandler::flushQueues, this);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] flush_queues service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/get_objects", param))
      {
        getObjectsService_ = nh_->advertiseService(param,
            &AlertHandler::getObjectsServiceCb, this);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] get_objects service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/get_markers", param))
      {
        getMarkersService_ = nh_->advertiseService(param,
            &AlertHandler::getMarkersServiceCb, this);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] get_markers service name param not found");
        ROS_BREAK();
      }

      if (nh_->getParam("service_server_names/geotiff", param))
      {
        geotiffService_ = nh_->advertiseService(param,
            &AlertHandler::geotiffServiceCb, this);
      }
      else
      {
        ROS_FATAL("[ALERT_HANDLER] geotiffSrv service name param not found");
        ROS_BREAK();
      }

      // Dynamic Reconfigure Server
      dynReconfServer_.setCallback(boost::bind(
            &AlertHandler::dynamicReconfigCallback, this, _1, _2));

      // Timers
      tfPublisherTimer_ = nh_->createTimer(ros::Duration(0.1),
          &AlertHandler::tfPublisherCallback, this);
    }

    /*  Other Callbacks  */

    void AlertHandler::tfPublisherCallback(const ros::TimerEvent& event)
    {
      PoseStampedVector objectsPosesStamped;
      qrs_->getObjectsPosesStamped(&objectsPosesStamped);
      hazmats_->getObjectsPosesStamped(&objectsPosesStamped);
      thermals_->getObjectsPosesStamped(&objectsPosesStamped);
      victimImages_->getObjectsPosesStamped(&objectsPosesStamped);
      motions_->getObjectsPosesStamped(&objectsPosesStamped);
      sounds_->getObjectsPosesStamped(&objectsPosesStamped);
      co2s_->getObjectsPosesStamped(&objectsPosesStamped);
      landoltcs_->getObjectsPosesStamped(&objectsPosesStamped);
      dataMatrices_->getObjectsPosesStamped(&objectsPosesStamped);
      victimsToGo_->getObjectsPosesStamped(&objectsPosesStamped);
      victimsVisited_->getObjectsPosesStamped(&objectsPosesStamped);

      broadcastPoseVector(objectsPosesStamped);
    }

    void AlertHandler::broadcastPoseVector(const PoseStampedVector& poseVector)
    {
      for (PoseStampedVector::const_iterator it = poseVector.begin();
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

        objectsBroadcaster_.sendTransform(
            tf::StampedTransform(tfObject, it->header.stamp,
              BaseObject::getGlobalFrame(), it->header.frame_id));
      }
    }

    void AlertHandler::deleteVictimCallback()
    {
      int victimId = deleteVictimServer_->acceptNewGoal()->victimId;
      bool deleted = victimHandler_->deleteVictim(victimId);
      publishVictims();
      if (!deleted)
        deleteVictimServer_->setAborted();
      deleteVictimServer_->setSucceeded();
    }

    void AlertHandler::validateVictimCallback()
    {
      GoalConstPtr goal = validateVictimServer_->acceptNewGoal();
      bool validated = victimHandler_->validateVictim(goal->victimId, goal->victimValid);
      publishVictims();
      if (!validated)
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
      objectFactory_->dynamicReconfigForward(config.occupiedCellThres,
          config.highThres, config.lowThres,
          config.orientationCircle);

      Hole::setObjectScore(-1);
      Hole::setProbabilityThres(config.holeMinProbability);
      Hole::setDistanceThres(config.holeMinDistance);
      Hole::setOrientDiff(config.holeOrientDiff);
      Hole::setMergeDistance(config.objectMergeDistance);
      Hole::getFilterModel()->initializeSystemModel(config.holeSystemNoiseSD);
      Hole::getFilterModel()->initializeMeasurementModel(config.holeMeasurementSD);

      Hazmat::setObjectScore(config.hazmatScore);
      Hazmat::setProbabilityThres(config.hazmatMinProbability);
      Hazmat::setDistanceThres(config.hazmatMinDistance);
      Hazmat::setOrientDiff(config.hazmatOrientDiff);
      Hazmat::setMergeDistance(config.objectMergeDistance);
      Hazmat::getFilterModel()->initializeSystemModel(config.hazmatSystemNoiseSD);
      Hazmat::getFilterModel()->initializeMeasurementModel(config.hazmatMeasurementSD);

      Qr::setObjectScore(config.qrScore);
      Qr::setProbabilityThres(config.qrMinProbability);
      Qr::setDistanceThres(config.qrMinDistance);
      Qr::setOrientDiff(config.qrOrientDiff);
      Qr::setMergeDistance(config.objectMergeDistance);
      Qr::getFilterModel()->initializeSystemModel(config.qrSystemNoiseSD);
      Qr::getFilterModel()->initializeMeasurementModel(config.qrMeasurementSD);

      DataMatrix::setObjectScore(config.dataMatrixScore);
      DataMatrix::setProbabilityThres(config.dataMatrixMinProbability);
      DataMatrix::setDistanceThres(config.dataMatrixMinDistance);
      DataMatrix::setOrientDiff(config.dataMatrixOrientDiff);
      DataMatrix::setMergeDistance(config.objectMergeDistance);
      DataMatrix::getFilterModel()->initializeSystemModel(config.dataMatrixSystemNoiseSD);
      DataMatrix::getFilterModel()->initializeMeasurementModel(config.dataMatrixMeasurementSD);

      Landoltc::setObjectScore(config.landoltcScore);
      Landoltc::setProbabilityThres(config.landoltcMinProbability);
      Landoltc::setDistanceThres(config.landoltcMinDistance);
      Landoltc::setOrientDiff(config.landoltcOrientDiff);
      Landoltc::setMergeDistance(config.objectMergeDistance);
      Landoltc::getFilterModel()->initializeSystemModel(config.landoltcSystemNoiseSD);
      Landoltc::getFilterModel()->initializeMeasurementModel(config.landoltcMeasurementSD);

      Thermal::setObjectScore(config.thermalScore);
      Thermal::setProbabilityThres(config.thermalMinProbability);
      Thermal::setDistanceThres(config.thermalMinDistance);
      Thermal::setOrientDiff(config.thermalOrientDiff);
      Thermal::setMergeDistance(config.objectMergeDistance);
      Thermal::getFilterModel()->initializeSystemModel(config.thermalSystemNoiseSD);
      Thermal::getFilterModel()->initializeMeasurementModel(config.thermalMeasurementSD);

      VictimImage::setObjectScore(config.victimImageScore);
      VictimImage::setProbabilityThres(config.victimImageMinProbability);
      VictimImage::setDistanceThres(config.victimImageMinDistance);
      VictimImage::setOrientDiff(config.victimImageOrientDiff);
      VictimImage::setMergeDistance(config.objectMergeDistance);
      VictimImage::getFilterModel()->initializeSystemModel(config.victimImageSystemNoiseSD);
      VictimImage::getFilterModel()->initializeMeasurementModel(config.victimImageMeasurementSD);

      Motion::setObjectScore(config.motionScore);
      Motion::setProbabilityThres(config.motionMinProbability);
      Motion::setDistanceThres(config.motionMinDistance);
      Motion::setOrientDiff(config.motionOrientDiff);
      Motion::setMergeDistance(config.objectMergeDistance);
      Motion::getFilterModel()->initializeSystemModel(config.motionSystemNoiseSD);
      Motion::getFilterModel()->initializeMeasurementModel(config.motionMeasurementSD);

      Sound::setObjectScore(config.soundScore);
      Sound::setProbabilityThres(config.soundMinProbability);
      Sound::setDistanceThres(config.soundMinDistance);
      Sound::setOrientDiff(config.soundOrientDiff);
      Sound::setMergeDistance(config.objectMergeDistance);
      Sound::getFilterModel()->initializeSystemModel(config.soundSystemNoiseSD);
      Sound::getFilterModel()->initializeMeasurementModel(config.soundMeasurementSD);

      Co2::setObjectScore(config.co2Score);
      Co2::setProbabilityThres(config.co2MinProbability);
      Co2::setDistanceThres(config.co2MinDistance);
      Co2::setOrientDiff(config.co2OrientDiff);
      Co2::setMergeDistance(config.objectMergeDistance);
      Co2::getFilterModel()->initializeSystemModel(config.co2SystemNoiseSD);
      Co2::getFilterModel()->initializeMeasurementModel(config.co2MeasurementSD);

      objectHandler_->updateParams(config.sensorRange, config.clusterRadius);

      victimHandler_->updateParams(config.clusterRadius, config.sameVictimRadius);
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
      victimImages_->getObjectsPosesStamped(&rs.victimImages);
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
      victimImages_->getVisualization(&rs.victimImages);
      motions_->getVisualization(&rs.motions);
      sounds_->getVisualization(&rs.sounds);
      co2s_->getVisualization(&rs.co2s);
      landoltcs_->getVisualization(&rs.landoltcs);
      dataMatrices_->getVisualization(&rs.dataMatrices);

      victimHandler_->getVisualization(&rs.victimsVisited, &rs.victimsToGo);

      return true;
    }

    bool AlertHandler::geotiffServiceCb(
        pandora_data_fusion_msgs::GeotiffSrv::Request &req,
        pandora_data_fusion_msgs::GeotiffSrv::Response &res)
    {
      qrs_->getObjectsPosesStamped(&res.qrs);
      hazmats_->getObjectsPosesStamped(&res.hazmats);

      victimHandler_->fillGeotiff(&res);

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
      victimImages_->clear();
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
