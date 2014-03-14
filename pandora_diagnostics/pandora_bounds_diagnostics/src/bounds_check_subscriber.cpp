#include "bounds_diagnostics/bounds_check_subscriber.h"

template <class MessageType>
BoundsCheckSubscriber<MessageType>::BoundsCheckSubscriber(
  std::string topic, diagnostic_updater::Updater & updater) : {
  
  topicName = topic;
  
  _updater = updater;
  
  subscriber = nh.subscribe(topicName,100,
    &BoundsCheckSubscriber::subscriberActualCallback, this);

}


