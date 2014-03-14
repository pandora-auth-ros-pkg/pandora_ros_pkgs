#ifndef SONARS_CHECKER_H
#define SONARS_CHECKER_H

#include "controllers_and_sensors_communications/sonarMsg.h"
#include "controllers_and_sensors_communications/headSonarMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::sonarMsg  sonar;
typedef controllers_and_sensors_communications::headSonarMsg  headSonar;


class sonarChecker : public BoundsCheckSubscriber<sonar>{
  
  bool sonarInfoStatus[5];
  sonar msg;

public:

  sonarChecker(std::string topic, diagnostic_updater::Updater & updater):
    BoundsCheckSubscriber(topic,updater){};

  void subscriberActualCallback(const sonar & sonarCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~sonarChecker() {};  
  
};


class headSonarChecker : public BoundsCheckSubscriber<headSonar>{
  
  bool headSonarInfoStatus;
  headSonar msg;
  
public:

  headSonarChecker(std::string topic, diagnostic_updater::Updater & updater): 
    BoundsCheckSubscriber(topic, updater) {};

  void subscriberActualCallback(const headSonar & headSonarCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~headSonarChecker() {};  
  
};

#endif
