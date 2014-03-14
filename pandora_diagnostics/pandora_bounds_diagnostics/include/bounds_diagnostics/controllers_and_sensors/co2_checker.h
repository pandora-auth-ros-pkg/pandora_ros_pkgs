#ifndef CO2_CHECKER_H
#define CO2_CHECKER_H

#include "controllers_and_sensors_communications/co2Msg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::co2Msg  co2;


class co2Checker : public BoundsCheckSubscriber<co2>{
  
  
  bool co2InfoStatus;
  co2 msg;

public:

  co2Checker(std::string topic, diagnostic_updater::Updater & updater): 
    BoundsCheckSubscriber(topic,updater) {};

  void subscriberActualCallback(const co2 & co2CheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~co2Checker() {};  
  
};

#endif
