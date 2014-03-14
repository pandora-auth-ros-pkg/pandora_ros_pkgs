#include "bounds_diagnostics/controllers_and_sensors/compass_checker.h"

void compassChecker::subscriberActualCallback(const compass & compassCheckerMsg){
  
  msg = compassCheckerMsg;
  
  compassPitchInfoStatus = true;
  compassRollInfoStatus = true;
  

  if (compassCheckerMsg.pitch<(-PI/2) || compassCheckerMsg.pitch>PI/2)
    compassPitchInfoStatus = false;

  if (compassCheckerMsg.roll<(-PI/2) || compassCheckerMsg.roll>PI/2)
    compassRollInfoStatus = false;  
  
  _updater.update();

}

bool compassChecker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool compassOk = true;
  
  if ((compassPitchInfoStatus == true) && (compassRollInfoStatus == true)){
    stat.add("CompassMsg", "All Ok");
  }
  else if (compassPitchInfoStatus == false){
    stat.addf("CompassMsg", "CompassPitch Out of bounds: %f", msg.pitch);
    compassOk = false;
  }
  else {
    stat.addf("CompassMsg", "CompassRoll Out of bounds: %f", msg.roll);
    compassOk = false;
  }
  
  return compassOk;
}
