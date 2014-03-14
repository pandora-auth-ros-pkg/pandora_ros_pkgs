#ifndef BUTTERFLY_CHECKER_H
#define BUTTERFLY_CHECKER_H

#include "controllers_and_sensors_communications/butterflyMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::butterflyMsg  butterfly;

class butterflyChecker : public BoundsCheckSubscriber<butterfly>{
  
  
  bool butterflyInfoStatus[2];
  butterfly msg;

public:

  butterflyChecker(std::string topic, diagnostic_updater::Updater & updater):
    BoundsCheckSubscriber(topic,updater){};

  void subscriberActualCallback(const butterfly & butterflyCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~butterflyChecker() {};  
  
};

#endif


