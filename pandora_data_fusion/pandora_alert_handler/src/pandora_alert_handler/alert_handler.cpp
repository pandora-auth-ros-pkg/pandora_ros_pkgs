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

#include "pandora_alert_handler/alert_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  AlertHandler::AlertHandler(const std::string& ns)
  {
    nh_.reset( new ros::NodeHandle(ns) );

    holes_.reset( new HoleList );
    obstacles_.reset( new ObstacleList );
    qrs_.reset( new QrList );
    hazmats_.reset( new HazmatList );
    thermals_.reset( new ThermalList );
    visualVictims_.reset( new VisualVictimList );
    motions_.reset( new MotionList );
    sounds_.reset( new SoundList );
    co2s_.reset( new Co2List );
    landoltcs_.reset( new LandoltcList );
    dataMatrices_.reset( new DataMatrixList );

    Hole::setList(holes_);
    Obstacle::setList(obstacles_);
    Qr::setList(qrs_);
    Hazmat::setList(hazmats_);
    Thermal::setList(thermals_);
    VisualVictim::setList(visualVictims_);
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
    nh_->param<std::string>("object_names/obstacle", param, "obstacle");
    Obstacle::setObjectType(param);
    nh_->param<std::string>("object_names/thermal", param, "thermal");
    Thermal::setObjectType(param);
    nh_->param<std::string>("object_names/visual_victim", param, "visual_victim");
    VisualVictim::setObjectType(param);
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
    Obstacle::is3D = false;
    Thermal::is3D = true;
    Motion::is3D = true;
    VisualVictim::is3D = true;

    Hazmat::isVictimAlert = true;
    Qr::isVictimAlert = false;
    Landoltc::isVictimAlert = false;
    DataMatrix::isVictimAlert = false;
    Sound::isVictimAlert = true;
    Co2::isVictimAlert = true;
    Hole::isVictimAlert = true;
    Obstacle::isVictimAlert = false;
    Thermal::isVictimAlert = true;
    Motion::isVictimAlert = true;
    VisualVictim::isVictimAlert = true;

    victimsToGo_.reset( new VictimList );
    victimsVisited_.reset( new VictimList );

    if (!nh_->getParam("map_type", param))
    {
      ROS_FATAL("[ALERT_HANDLER] map_type param not found");
      ROS_BREAK();
    }

    if (!nh_->getParam("global_frame", globalFrame_))
    {
      ROS_FATAL("[ALERT_HANDLER] global_frame param not found");
      ROS_BREAK();
    }

    poseFinderPtr_.reset( new pose_finder::PoseFinder(param) );
    objectFactory_.reset( new ObjectFactory(poseFinderPtr_, globalFrame_) );
    objectHandler_.reset( new ObjectHandler(nh_, victimsToGo_, victimsVisited_) );
    victimHandler_.reset( new VictimHandler(nh_, globalFrame_, victimsToGo_, victimsVisited_) );

    initRosInterfaces();
  }

  /**
    * [AlertHandler::publishVictims description]
    */
  void AlertHandler::publishVictims()
  {
    pandora_data_fusion_msgs::WorldModel worldModelMsg;
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
    setSubscriber<VisualVictim>();
    setSubscriber<Co2>();
    setSubscriber<Motion>();
    setSubscriber<Sound>();
    setSubscriber<Obstacle>();

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
        advertise<pandora_data_fusion_msgs::WorldModel>(param, 10);
    }
    else
    {
      ROS_FATAL("[ALERT_HANDLER] world_model topic name param not found");
      ROS_BREAK();
    }

    // Action Servers

    if (nh_->getParam("action_server_names/target_victim", param))
    {
      targetVictimServer_.reset(new TargetVictimServer(*nh_, param, false));
    }
    else
    {
      ROS_FATAL("[ALERT_HANDLER] target_victim action name param not found");
      ROS_BREAK();
    }
    targetVictimServer_->registerGoalCallback(
        boost::bind(&AlertHandler::targetVictimCallback, this));
    targetVictimServer_->start();

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
      ROS_FATAL("[ALERT_HANDLER] getObjects service name param not found");
      ROS_BREAK();
    }

    if (nh_->getParam("service_server_names/get_markers", param))
    {
      getMarkersService_ = nh_->advertiseService(param,
          &AlertHandler::getMarkersServiceCb, this);
    }
    else
    {
      ROS_FATAL("[ALERT_HANDLER] getMarkers service name param not found");
      ROS_BREAK();
    }

    if (nh_->getParam("service_server_names/get_geotiff", param))
    {
      getGeotiffService_ = nh_->advertiseService(param,
          &AlertHandler::getGeotiffServiceCb, this);
    }
    else
    {
      ROS_FATAL("[ALERT_HANDLER] getGeotiff service name param not found");
      ROS_BREAK();
    }

    if (nh_->getParam("service_server_names/get_victim_probabilities", param))
    {
      getVictimProbabilitiesService_ = nh_->advertiseService(param,
          &AlertHandler::getVictimProbabilitiesCb, this);
    }
    else
    {
      ROS_FATAL("[ALERT_HANDLER] getVictimProbabilities service name param not found");
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
    PoseStampedVector objectsTfInfo;
    obstacles_->getObjectsTfInfo(&objectsTfInfo);
    qrs_->getObjectsTfInfo(&objectsTfInfo);
    hazmats_->getObjectsTfInfo(&objectsTfInfo);
    thermals_->getObjectsTfInfo(&objectsTfInfo);
    visualVictims_->getObjectsTfInfo(&objectsTfInfo);
    motions_->getObjectsTfInfo(&objectsTfInfo);
    sounds_->getObjectsTfInfo(&objectsTfInfo);
    co2s_->getObjectsTfInfo(&objectsTfInfo);
    landoltcs_->getObjectsTfInfo(&objectsTfInfo);
    dataMatrices_->getObjectsTfInfo(&objectsTfInfo);
    victimsToGo_->getObjectsTfInfo(&objectsTfInfo);
    victimsVisited_->getObjectsTfInfo(&objectsTfInfo);

    broadcastPoseVector(objectsTfInfo);
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
                               globalFrame_, it->header.frame_id));
    }
  }

  void AlertHandler::targetVictimCallback()
  {
    int victimId = targetVictimServer_->acceptNewGoal()->victimId;
    bool targeted = victimHandler_->targetVictim(victimId);
    if (!targeted)
      targetVictimServer_->setAborted();
    targetVictimServer_->setSucceeded();
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
    bool validated = victimHandler_->validateVictim(goal->victimId,
        goal->victimVerified, goal->victimValid);
    publishVictims();
    if (!validated)
      validateVictimServer_->setAborted();
    validateVictimServer_->setSucceeded();
  }

  void AlertHandler::updateMap(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    mapPtr_ = msg;
    poseFinderPtr_->updateMap(msg);
  }

  void AlertHandler::dynamicReconfigCallback(
      const ::pandora_alert_handler::AlertHandlerConfig& config, uint32_t level)
  {
    objectFactory_->dynamicReconfigForward(config.occupiedCellThres,
        config.highThres, config.lowThres,
        config.orientationCircle, config.softObstacleWidth);

    Hole::setObjectScore(-1);
    Hole::setProbabilityThres(config.holeMinProbability);
    Hole::setDistanceThres(config.holeMinDistance);
    Hole::setOrientDiff(config.holeOrientDiff);
    Hole::setMergeDistance(config.objectMergeDistance);
    Hole::getFilterModel()->initializeSystemModel(config.holeSystemNoiseSD);
    Hole::getFilterModel()->initializeMeasurementModel(config.holeMeasurementSD);

    Obstacle::setObjectScore(config.obstacleScore);
    Obstacle::setProbabilityThres(config.obstacleMinProbability);
    Obstacle::setDistanceThres(config.obstacleMinDistance);
    Obstacle::setOrientDiff(config.obstacleOrientDiff);
    Obstacle::setMergeDistance(config.objectMergeDistance);
    Obstacle::getFilterModel()->initializeSystemModel(config.obstacleSystemNoiseSD);
    Obstacle::getFilterModel()->initializeMeasurementModel(config.obstacleMeasurementSD);

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

    VisualVictim::setObjectScore(config.visualVictimScore);
    VisualVictim::setProbabilityThres(config.visualVictimMinProbability);
    VisualVictim::setDistanceThres(config.visualVictimMinDistance);
    VisualVictim::setOrientDiff(config.visualVictimOrientDiff);
    VisualVictim::setMergeDistance(config.objectMergeDistance);
    VisualVictim::getFilterModel()->initializeSystemModel(config.visualVictimSystemNoiseSD);
    VisualVictim::getFilterModel()->initializeMeasurementModel(config.visualVictimMeasurementSD);

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
      pandora_data_fusion_msgs::GetObjects::Request& rq,
      pandora_data_fusion_msgs::GetObjects::Response& rs)
  {
    pandora_data_fusion_msgs::GetObjectsResponsePtr ptr(
        new pandora_data_fusion_msgs::GetObjectsResponse );
    holes_->getObjectsPosesStamped(&ptr->holes);
    obstacles_->getObjectsPosesStamped(&ptr->obstacles);
    qrs_->getObjectsPosesStamped(&ptr->qrs);
    hazmats_->getObjectsPosesStamped(&ptr->hazmats);
    thermals_->getObjectsPosesStamped(&ptr->thermals);
    visualVictims_->getObjectsPosesStamped(&ptr->visualVictims);
    motions_->getObjectsPosesStamped(&ptr->motions);
    sounds_->getObjectsPosesStamped(&ptr->sounds);
    co2s_->getObjectsPosesStamped(&ptr->co2s);
    landoltcs_->getObjectsPosesStamped(&ptr->landoltcs);
    dataMatrices_->getObjectsPosesStamped(&ptr->dataMatrices);

    victimHandler_->getVictimsPosesStamped(&ptr->victimsToGo, &ptr->victimsVisited);
    rs = *ptr;

    return true;
  }

  bool AlertHandler::getMarkersServiceCb(
      pandora_data_fusion_msgs::GetMarkers::Request& rq,
      pandora_data_fusion_msgs::GetMarkers::Response& rs)
  {
    pandora_data_fusion_msgs::GetMarkersResponsePtr ptr(
        new pandora_data_fusion_msgs::GetMarkersResponse );
    holes_->getVisualization(&ptr->holes);
    obstacles_->getVisualization(&ptr->obstacles);
    qrs_->getVisualization(&ptr->hazmats);
    hazmats_->getVisualization(&ptr->qrs);
    thermals_->getVisualization(&ptr->thermals);
    visualVictims_->getVisualization(&ptr->visualVictims);
    motions_->getVisualization(&ptr->motions);
    sounds_->getVisualization(&ptr->sounds);
    co2s_->getVisualization(&ptr->co2s);
    landoltcs_->getVisualization(&ptr->landoltcs);
    dataMatrices_->getVisualization(&ptr->dataMatrices);

    victimHandler_->getVisualization(&ptr->victimsVisited, &ptr->victimsToGo);
    rs = *ptr;

    return true;
  }

  bool AlertHandler::getGeotiffServiceCb(
      pandora_data_fusion_msgs::GetGeotiff::Request& rq,
      pandora_data_fusion_msgs::GetGeotiff::Response& rs)
  {
    pandora_data_fusion_msgs::GetGeotiffResponsePtr ptr(
        new pandora_data_fusion_msgs::GetGeotiffResponse );
    qrs_->fillGeotiff(ptr);
    hazmats_->fillGeotiff(ptr);
    obstacles_->fillGeotiff(ptr);
    victimHandler_->fillGeotiff(ptr);
    rs = *ptr;
    return true;
  }

  bool AlertHandler::getVictimProbabilitiesCb(
      pandora_data_fusion_msgs::GetVictimProbabilities::Request& rq,
      pandora_data_fusion_msgs::GetVictimProbabilities::Response& rs)
  {
    pandora_data_fusion_msgs::GetVictimProbabilitiesResponsePtr ptr(
        new pandora_data_fusion_msgs::GetVictimProbabilitiesResponse );
    bool success = victimHandler_->getVictimProbabilities(rq.victimId, ptr);
    rs = *ptr;
    rs.success = success;

    return true;
  }

  bool AlertHandler::flushQueues(
      std_srvs::Empty::Request& rq,
      std_srvs::Empty::Response& rs)
  {
    ROS_INFO_NAMED("ALERT_HANDLER_FLUSH_SERVICE", "Flushing lists!");
    holes_->clear();
    obstacles_->clear();
    qrs_->clear();
    hazmats_->clear();
    thermals_->clear();
    visualVictims_->clear();
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
