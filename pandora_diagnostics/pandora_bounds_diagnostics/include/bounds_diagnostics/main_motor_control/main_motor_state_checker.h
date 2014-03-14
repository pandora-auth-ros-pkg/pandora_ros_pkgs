#ifndef MAIN_MOTOR_STATE_CHECKER_h
#define MAIN_MOTOR_STATE_CHECKER_h

#include "main_motor_control_communications/mainMotorStateMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef main_motor_control_communications::mainMotorStateMsg motorState;

class mainMotorStateChecker : public BoundsCheckSubscriber<motorState>{
  
  bool stateLeftInfoStatus;
  bool stateRightInfoStatus;
  
  bool currentLeftInfoStatus;
  bool currentRightInfoStatus;
  
  bool velocityLeftInfoStatus;
  bool velocityRightInfoStatus;
  
  bool velocityLinearInfoStatus;
  bool velocityAngularInfoStatus;
  
  bool powerLeftInfoStatus;
  bool powerRightInfoStatus;
  
  bool efficiencyLeftInfoStatus;
  bool efficiencyRightInfoStatus;
  
  motorState msg;

public:

  mainMotorStateChecker(
    std::string topic, diagnostic_updater::Updater & updater): 
      BoundsCheckSubscriber(topic,updater), stateLeftInfoStatus(true),
      stateRightInfoStatus(true), currentLeftInfoStatus(true),
      currentRightInfoStatus(true), velocityLeftInfoStatus(true), 
      velocityRightInfoStatus(true), velocityLinearInfoStatus(true),
      velocityAngularInfoStatus(true), powerLeftInfoStatus(true), 
      powerRightInfoStatus(true), efficiencyLeftInfoStatus(true),
      efficiencyRightInfoStatus(true) {};

  void subscriberActualCallback(const motorState & mainMotorStateCheckerMsg);

  bool sensorCheckerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~mainMotorStateChecker() {};  
  
};

#endif
