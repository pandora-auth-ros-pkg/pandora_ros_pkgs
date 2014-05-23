// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_factory.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectFactory::ObjectFactory(const MapPtr& map, const std::string& mapType)
    {
      poseFinder_.reset( new PoseFinder(map, mapType) );
    }

    HolePtrVectorPtr ObjectFactory::makeHoles(
        const vision_communications::HolesDirectionsVectorMsg& msg)
    {
      currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

      HolePtrVectorPtr holesVectorPtr( new HolePtrVector );
      for (int ii = 0; ii < msg.holesDirections.size(); ++ii)
      {
        try
        {
          HolePtr newHole( new Hole );
          setUpHole( newHole, msg.holesDirections[ii] );
          holesVectorPtr->push_back( newHole );
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
        }
      }

      return holesVectorPtr;
    }

    HazmatPtrVectorPtr ObjectFactory::makeHazmats(
        const vision_communications::HazmatAlertsVectorMsg& msg)
    {
      currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

      HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
      for (int ii = 0; ii < msg.hazmatAlerts.size(); ++ii)
      {
        try
        {
          HazmatPtr newHazmat( new Hazmat );
          setUpHazmat( newHazmat, msg.hazmatAlerts[ii] );
          hazmatsVectorPtr->push_back( newHazmat );
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
        }
      }

      return hazmatsVectorPtr;
    }

    QrPtrVectorPtr ObjectFactory::makeQrs(
        const vision_communications::QRAlertsVectorMsg& msg)
    {
      currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

      QrPtrVectorPtr qrsVectorPtr( new QrPtrVector );
      for (int ii = 0; ii < msg.qrAlerts.size(); ++ii)
      {
        try
        {
          QrPtr newQr( new Qr );
          setUpQr( newQr, msg.qrAlerts[ii], msg.header.stamp );
          qrsVectorPtr->push_back( newQr );
        }
        catch (AlertException ex)
        {
          ROS_WARN_NAMED("ALERT_HANDLER",
              "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
        }
      }

      return qrsVectorPtr;
    }

    void ObjectFactory::dynamicReconfigForward(float occupiedCellThres,
        float highThres, float lowThres,
        float orientationCircle, float orientationDist)
    {
      poseFinder_->updateParams( occupiedCellThres, 
          highThres, lowThres,
          orientationDist, orientationCircle
          );
    }

    void ObjectFactory::setUpHole(const HolePtr& holePtr, 
        const vision_communications::HoleDirectionMsg& msg)
    {
      holePtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
            msg.pitch, currentTransform_) );
      holePtr->setProbability( msg.probability );
      holePtr->setHoleId( msg.holeId );
      holePtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpHazmat(const HazmatPtr& hazmatPtr, 
        const vision_communications::HazmatAlertMsg& msg)
    {
      hazmatPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
            msg.pitch, currentTransform_) );
      hazmatPtr->setProbability( 0.5 );
      hazmatPtr->setPattern( msg.patternType );
      hazmatPtr->initializeObjectFilter();
    }

    void ObjectFactory::setUpQr(const QrPtr& qrPtr, 
        const vision_communications::QRAlertMsg& msg,
        ros::Time timeFound)
    {
      qrPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
            msg.pitch, currentTransform_) );
      qrPtr->setProbability( 0.5 );
      qrPtr->setContent( msg.QRcontent );
      qrPtr->initializeObjectFilter();
      qrPtr->setTimeFound(timeFound);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

