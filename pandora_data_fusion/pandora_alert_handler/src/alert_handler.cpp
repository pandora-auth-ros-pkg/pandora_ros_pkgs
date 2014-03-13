// "Copyright [year] <Copyright Owner>"

#include "alert_handler/alert_handler.h"
#include <string> 

AlertHandler::AlertHandler() {
  initializeMap();

  holes_.reset(new ObjectList<Hole>);
  qrs_.reset(new ObjectList<Qr>);
  hazmats_.reset(new ObjectList<Hazmat>);
  tpas_.reset(new ObjectList<Tpa>);

  poseFinderPtr_.reset( new PoseFinder(map_) );
  objectHandler_.reset( new ObjectHandler( holes_, qrs_, hazmats_, tpas_  ) );
  victimHandler_.reset( new VictimHandler( holes_ , tpas_ ) );

  initRosInterfaces();

  curState = 0;
  eraseHolesQrs = true;

  clientInitialize();
}

void AlertHandler::initRosInterfaces() {

  std::string param; 

  //~ subscribers

  if (nh_.getParam("subscribed_topic_names/holeDirection", param)) {
    holeDirectionSubscrider_ = nh_.subscribe(param, 
      1, &AlertHandler::holeDirectionAlertCallback, this);
  } else {
    ROS_FATAL("holeDirection topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/tpaDirection", param)) {
    tpaDirectionSubscrider_ = nh_.subscribe(param, 
      1, &AlertHandler::tpaDirectionAlertCallback, this);
  } else {
    ROS_FATAL("tpaDirection topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/victimVerification", param)) {
    victimVerificationSubscrider_ = nh_.subscribe(param, 
      1, &AlertHandler::victimVerificationCallback, this);
  } else {
    ROS_FATAL("victimVerification topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/qr", param)) {
  qrSubscrider_ = nh_.subscribe(param, 1, &AlertHandler::qrAlertCallback, this);
  } else {
    ROS_FATAL("qr topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/hazmat", param)) {
    hazmatSubscrider_ = nh_.subscribe(param, 
      1, &AlertHandler::hazmatAlertCallback, this);
  } else {
    ROS_FATAL("hazmat topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/currentVictim", param)) {
    currentVictimSubscrider_ = nh_.subscribe(param, 
      1, &AlertHandler::selectedVictimCallback, this);
  } else {
    ROS_FATAL("currentVictim topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/map", param)) {
    mapSubscriber_ = nh_.subscribe(param, 1, &AlertHandler::updateMap, this);
  } else {
    ROS_FATAL("map topic name param not found");
    ROS_BREAK();
  }
  
  //~ action servers
  
  if (nh_.getParam("action_server_names/get_victims", param)) {
    victimsServer_.reset(new GetVictimsServer(nh_, param,  false));
  } else {
    ROS_FATAL("get_victims action name param not found");
    ROS_BREAK();
  }
  victimsServer_->registerGoalCallback(
    boost::bind(&AlertHandler::getVictimsCallback, this) );
  victimsServer_->start();
  
  if (nh_.getParam("action_server_names/delete_current_victim", param)) {
    deleteVictimServer_.reset(new DeleteVictimServer(nh_, param,  false));
  } else {
    ROS_FATAL("delete_current_victim action name param not found");
    ROS_BREAK();
  }
  deleteVictimServer_->registerGoalCallback(
    boost::bind(&AlertHandler::deleteVictimCallback, this) );
  deleteVictimServer_->start();
  
  if (nh_.getParam("action_server_names/validate_current_hole", param)) {
    validateCurrentHoleServer_.reset(
    new ValidateCurrentHoleServer(nh_, param,  false));
  } else {
    ROS_FATAL("validate_current_hole action name param not found");
    ROS_BREAK();
  }
  validateCurrentHoleServer_->registerGoalCallback( 
    boost::bind(&AlertHandler::validateCurrentHoleCallback, this) );
  validateCurrentHoleServer_->start();

  //~ service servers
  
  if (nh_.getParam("service_server_names/flush_queues", param)) {
  flushService_ = nh_.advertiseService(param,
                                     &AlertHandler::flushQueues, this);
  } else {
    ROS_FATAL("flush_queues service name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("service_server_names/get_objects", param)) {
  getObjectsService_ = nh_.advertiseService(param,
                                     &AlertHandler::getObjectsServiceCb, this);
  } else {
    ROS_FATAL("get_objects service name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("service_server_names/get_markers", param)) {
  getMarkersService_ = nh_.advertiseService(param,
                                     &AlertHandler::getMarkersServiceCb, this);
  } else {
    ROS_FATAL("get_markers service name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("service_server_names/geotiff", param)) {
  geotiffService_ = nh_.advertiseService(param,
                                     &AlertHandler::geotiffSericeCb, this);
  } else {
    ROS_FATAL("geotiffSrv service name param not found");
    ROS_BREAK();
  }

  //~ dynamic reconfigure server
  
  dynReconfserver_.setCallback(boost::bind(
    &AlertHandler::dynamicReconfigCallback, this, _1, _2));

  //~ timers
  
  currentVictimTimer_ = nh_.createTimer(ros::Duration(0.1),
                                     &AlertHandler::currentVictimTimerCb, this);
  
}


void AlertHandler::currentVictimTimerCb(const ros::TimerEvent& event) {
  tf::StampedTransform stampedTransform;

  if ( victimHandler_->getCurrentVictimTransform(&stampedTransform)  ) {
    currentVictimBroadcaster_.sendTransform(stampedTransform);
  }

}

void AlertHandler::holeDirectionAlertCallback(
    const vision_communications::HolesDirectionsVectorMsg& msg) {
    
  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "HOLE ALERT ARRIVED!");  
    
  tf::Transform cameraTransform;
  try {
    cameraTransform = poseFinderPtr_->lookupTransformFromWorld(msg.header);
  } catch (AlertException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  HolePtrStdVector holesVector;
  for (int i = 0; i < msg.holesDirections.size(); i++) {

    try {
      HolePtr newHole(new Hole);
      newHole->setPose(poseFinderPtr_->findAlertPose(msg.holesDirections[i].yaw,
                      msg.holesDirections[i].pitch, cameraTransform));
      newHole->setHoleId(msg.holesDirections[i].holeId);
      newHole->setProbability(msg.holesDirections[i].probability);
      holesVector.push_back(newHole);
    } catch (AlertException ex) {
      ROS_WARN_NAMED("ALERT_HANDLER",
         "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }

  }

  objectHandler_->handleHoles(holesVector, cameraTransform);

  victimHandler_->notify();
}



void AlertHandler::hazmatAlertCallback(
    const vision_communications::HazmatAlertsVectorMsg& msg) {

  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "HAZMAT ALERT ARRIVED!");  

  tf::Transform cameraTransform;
  try {

    cameraTransform = poseFinderPtr_->lookupTransformFromWorld(msg.header);
  } catch (AlertException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  HazmatPtrStdVector hazmatsVector;
  for (int i = 0; i < msg.hazmatAlerts.size(); i++) {

    try {
      HazmatPtr newHazmat(new Hazmat);
      newHazmat->setPose(poseFinderPtr_->findAlertPose(msg.hazmatAlerts[i].yaw,
                        msg.hazmatAlerts[i].pitch, cameraTransform));
      newHazmat->setPattern(msg.hazmatAlerts[i].patternType);
      hazmatsVector.push_back(newHazmat);
    } catch (AlertException ex) {
      ROS_WARN_NAMED("ALERT_HANDLER",
        "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }

  }

  objectHandler_->handleHazmats(hazmatsVector, cameraTransform);

  //~ victimHandler_->fixVictims();
}

void AlertHandler::qrAlertCallback(
    const vision_communications::QRAlertsVectorMsg& msg) {

  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "QR ALERT ARRIVED!");  

  tf::Transform cameraTransform;

  try {
    cameraTransform = poseFinderPtr_->lookupTransformFromWorld(msg.header);
  } catch (AlertException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  QrPtrStdVector qrsVector;
  for (int i = 0; i < msg.qrAlerts.size(); i++) {

    try {
      QrPtr newQr(new Qr);
      newQr->setPose(poseFinderPtr_->findAlertPose(msg.qrAlerts[i].yaw,
                     msg.qrAlerts[i].pitch, cameraTransform));
      newQr->setContent(msg.qrAlerts[i].QRcontent);
      qrsVector.push_back(newQr);
    } catch (AlertException ex) {
      ROS_WARN_NAMED("ALERT_HANDLER", 
        "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }

  }

  objectHandler_->handleQrs(qrsVector, cameraTransform, eraseHolesQrs);

  victimHandler_->fixVictims();
}

void AlertHandler::tpaDirectionAlertCallback(
    const data_fusion_communications::ThermalDirectionAlertMsg& msg) {

  tf::Transform frameTransform;
  try {
    frameTransform = poseFinderPtr_->lookupTransformFromWorld(msg.header);
  } catch (AlertException ex) {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  TpaPtrStdVector tpaVector;
  try {
    TpaPtr newTpa(new Tpa);
    newTpa->setPose(poseFinderPtr_->findAlertPose(msg.yaw,
                   msg.pitch, frameTransform));
    newTpa->setProbability(msg.probability);
    tpaVector.push_back(newTpa);
  } catch (AlertException ex) {
    ROS_WARN_NAMED("ALERT_HANDLER", 
      "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
  }

  objectHandler_->handleTpas(tpaVector, frameTransform);

  victimHandler_->notify();
}

void AlertHandler::victimVerificationCallback(
    const data_fusion_communications::VictimVerificationMsg& msg) {
  victimHandler_->handleVictimVerification(msg);
}

void AlertHandler::selectedVictimCallback(const std_msgs::Int16& msg) {
  victimHandler_->setCurrentVictimIndex(msg.data);
}

void AlertHandler::getVictimsCallback() {
  victimsServer_->acceptNewGoal();

  data_fusion_communications::GetVictimsResult result;

  victimHandler_->getVictimsMsg( & result.victimsArray );

  victimsServer_->setSucceeded(result);
}

void AlertHandler::deleteVictimCallback() {
  deleteVictimServer_->acceptNewGoal();

  victimHandler_->deleteCurrentVictim();

  deleteVictimServer_->setSucceeded();
}

void AlertHandler::validateCurrentHoleCallback() {
  bool victimValid = validateCurrentHoleServer_->acceptNewGoal()->valid;

  victimHandler_->validateCurrentHole(victimValid);

  validateCurrentHoleServer_->setSucceeded();
}

void AlertHandler::updateMap(const nav_msgs::OccupancyGridConstPtr& msg) {

  int width = msg->info.width;
  int height = msg->info.height;

  prevxMin = (msg->info.origin.position.x) / OCGD + MAP_SIZE / 2;
  prevyMin = (msg->info.origin.position.y) / OCGD + MAP_SIZE / 2;

  for(int i = 0; i < width; i++) {
    for(int j = 0; j < height; j++) {
      map_[i + prevxMin][j + prevyMin] =
        (100.0 - msg->data[j * width + i]) / 100.0 * 255.0;
    }
  }
}

void AlertHandler::initializeMap() {
  map_ = new unsigned char *[MAP_SIZE];
  for (unsigned int i = 0; i < MAP_SIZE; i++)
    map_[i] = new unsigned char[MAP_SIZE];

  for (unsigned int i = 0; i < MAP_SIZE; i++)
    for (unsigned int j = 0; j < MAP_SIZE; j++)
      map_[i][j] = 127;
}


void AlertHandler::dynamicReconfigCallback(
    pandora_alert_handler::AlertHandlerConfig &config, uint32_t level) {

  poseFinderPtr_->updateParams(
    config.highThres, config.lowThres, config.approachDist,
    config.orientationDist, config.orientationCircle
  );

  holes_->setParams(config.holeCounterThreshold, config.holeMinimumDist);
  qrs_->setParams(config.qrCounterThreshold, config.qrMinimumDist);
  hazmats_->setParams(config.hazmatCounterThreshold, config.hazmatMinimumDist);

  objectHandler_->updateParams(config.sensorRange,
                               config.qrClosestAlert,
                               config.hazmatClosestAlert );

  victimHandler_->updateParams(
    config.clusterRadius , config.sameVictimRadius,
    config.approachDist, config.victimUpdate , config.verificationProbability);
}


///////////////////////////////////////////////////////



bool AlertHandler::getObjectsServiceCb(
    data_fusion_communications::GetObjectsSrv::Request& rq,
      data_fusion_communications::GetObjectsSrv::Response &rs) {

  holes_->getObjectsPosesStamped(&rs.holes);
  qrs_->getObjectsPosesStamped(&rs.hazmats);
  hazmats_->getObjectsPosesStamped(&rs.qrs);
  tpas_->getObjectsPosesStamped(&rs.tpas);

  //~ victimHandler_->getVictimsPosesStamped(&rs.victimsToGo, &rs.victimsVisited
                                         //~ , &rs.approachPoints);

  return true;

}

bool AlertHandler::getMarkersServiceCb(
    data_fusion_communications::GetMarkersSrv::Request& rq,
      data_fusion_communications::GetMarkersSrv::Response &rs) {
  holes_->getVisualization(&rs.holes);
  qrs_->getVisualization(&rs.hazmats);
  hazmats_->getVisualization(&rs.qrs);
  tpas_->getVisualization(&rs.tpas);

  victimHandler_->getVisualization(&rs.victimsVisited, &rs.victimsToGo);

  return true;

}

bool AlertHandler::geotiffSericeCb(
    data_fusion_communications::DatafusionGeotiffSrv::Request &req,
      data_fusion_communications::DatafusionGeotiffSrv::Response &res) {
  qrs_->fillGeotiff(&res);
  hazmats_->fillGeotiff(&res);
  victimHandler_->fillGeotiff(&res);

  for (int i = 0; i < res.victimsx.size(); i++) {
    res.victimsx[i] -= prevxMin;
    res.victimsy[i] -= prevyMin;
  }

  for (int i = 0; i < res.qrx.size(); i++) {
    res.qrx[i] -=  prevxMin;
    res.qry[i] -=  prevyMin;
  }

  for (int i = 0; i < res.hazmatx.size(); i++) {
    res.hazmatx[i] -=  prevxMin;
    res.hazmaty[i] -=  prevyMin;
  }

  return true;
}



bool AlertHandler::flushQueues(
    std_srvs::Empty::Request& rq,
      std_srvs::Empty::Response &rs) {
  holes_->clear();
  qrs_->clear();
  hazmats_->clear();
  victimHandler_->flush();

}

void AlertHandler::startTransition(int newState) {

  curState = newState;

  //check if face detection algorithm should be running now
  eraseHolesQrs = 
  (curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
  (curState == state_manager_communications::robotModeMsg::MODE_IDENTIFICATION);

  //shutdown if the robot is switched off
  if (curState == 
        state_manager_communications::robotModeMsg::MODE_TERMINATING) {
    ros::shutdown();
    return;
  }

  transitionComplete(curState); 
}
