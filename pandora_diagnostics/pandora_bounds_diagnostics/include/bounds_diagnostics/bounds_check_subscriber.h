#ifndef BOUNDS_CHECK_SUBSCRIBER_H
#define BOUNDS_CHECK_SUBSCRIBER_H

#include <diagnostic_updater/diagnostic_updater.h>

template <class MessageType>
class BoundsCheckSubscriber {

protected:

  ros::NodeHandle nh;  

  ros::Subscriber subscriber;

  std::string topicName;

  diagnostic_updater::Updater & _updater;

  
public:  
  
  BoundsCheckSubscriber(
    std::string topic, diagnostic_updater::Updater & updater) : 
      _updater(updater) {
    
    topicName = topic;
    subscriber = nh.subscribe(topicName,100,
      &BoundsCheckSubscriber::subscriberActualCallback, this);
    
  };
  
    virtual ~BoundsCheckSubscriber() {};
  
    virtual void subscriberActualCallback(const MessageType & checkerMsg) = 0; 
    
    virtual  bool sensorCheckerDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;

};


#endif
