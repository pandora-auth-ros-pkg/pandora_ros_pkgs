#ifndef TPA_CHECKER_H
#define TPA_CHECKER_H

#include "controllers_and_sensors_communications/tpaMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::tpaMsg  tpa;

class tpaChecker : public BoundsCheckSubscriber<tpa>{
  
  bool tpaInfoStatus[5][9];
  std::string tpaPos[5];
  tpa msg;

public:

  tpaChecker(std::string topic, diagnostic_updater::Updater & updater) : 
    BoundsCheckSubscriber(topic,updater){};

  void subscriberActualCallback(const tpa & tpaCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~tpaChecker() {};  
  
};

#endif
