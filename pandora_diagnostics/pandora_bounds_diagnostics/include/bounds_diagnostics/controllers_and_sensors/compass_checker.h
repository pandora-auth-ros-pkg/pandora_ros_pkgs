#ifndef COMPASS_CHECKER_H
#define COMPASS_CHECKER_H

#include "controllers_and_sensors_communications/compassMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::compassMsg  compass;

#define PI 3.14159265359

class compassChecker : public BoundsCheckSubscriber<compass>{

  bool compassPitchInfoStatus;
  bool compassRollInfoStatus;
  
  compass msg;

public:

  compassChecker(std::string topic, diagnostic_updater::Updater & updater):
    BoundsCheckSubscriber(topic,updater) {};

  void subscriberActualCallback(const compass & compassCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);

  ~compassChecker() {};  
  
};  

#endif
