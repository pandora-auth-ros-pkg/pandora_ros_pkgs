#ifndef IRS_CHECKER_H
#define IRS_CHECKER_H

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include "controllers_and_sensors_communications/irMsg.h"
#include "controllers_and_sensors_communications/headIrMsg.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::irMsg  ir;
typedef controllers_and_sensors_communications::headIrMsg  headIr;


class irChecker : public BoundsCheckSubscriber<ir>{
  
  
  bool irInfoStatus[4];
  ir msg;

public:

  irChecker(std::string topic, diagnostic_updater::Updater & updater):
    BoundsCheckSubscriber(topic,updater) {};

  void subscriberActualCallback(const ir & irCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~irChecker() {};  
  
};


class headIrChecker : public BoundsCheckSubscriber<headIr>{
  
  
  bool headIrInfoStatus;
  headIr msg;

public:

  headIrChecker(std::string topic, diagnostic_updater::Updater & updater): 
    BoundsCheckSubscriber(topic,updater){};

  void subscriberActualCallback(const headIr & headIrCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~headIrChecker() {};  
  
};

#endif
