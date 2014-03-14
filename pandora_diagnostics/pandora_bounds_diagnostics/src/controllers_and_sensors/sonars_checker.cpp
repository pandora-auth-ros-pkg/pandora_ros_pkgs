#include "bounds_diagnostics/controllers_and_sensors/sonars_checker.h"

void sonarChecker::subscriberActualCallback(const sonar & sonarCheckerMsg){
  
  msg =  sonarCheckerMsg;
  
  for (int i=0;i<5;i++)
    sonarInfoStatus[i] = true;

  for (int i=0;i<5;i++){
    if (sonarCheckerMsg.distance[i]<2 || sonarCheckerMsg.distance[i]>450){
      sonarInfoStatus[i] = false;
    }
  }
  _updater.update();
      
}

bool sonarChecker::sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool sonarOk = true;
  
  for (int i=0;i<5;i++){
    if (sonarInfoStatus[i] == false){
      stat.addf("SonarMsg", "Sonar %d Out of bounds: %d", i+1, msg.distance[i]);
      sonarOk = false;
    }
  }
  
  if (sonarOk)
    stat.add("SonarMsg", "All Ok");
  
  return sonarOk;
    
}


void headSonarChecker::subscriberActualCallback(
    const headSonar & headSonarCheckerMsg){
  
  msg =  headSonarCheckerMsg;
  
  headSonarInfoStatus  = true;

  if (msg.distance<2 || msg.distance>450){
    headSonarInfoStatus = false;
  }
  _updater.force_update();
      
}

bool headSonarChecker::sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  
  bool headSonarOk = true;
  
  if (headSonarInfoStatus == true){
    stat.add("HeadSonarMsg", "All Ok");
  }
  else{
    stat.addf("HeadSonarMsg", "HeadSonar Out of bounds: %d", msg.distance);
    headSonarOk = false;
  }  
  
  
  return headSonarOk;
}
