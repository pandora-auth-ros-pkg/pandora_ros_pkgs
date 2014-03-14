#include "bounds_diagnostics/controllers_and_sensors/butterfly_checker.h"

void butterflyChecker::subscriberActualCallback(
  const butterfly & butterflyCheckerMsg){

  msg = butterflyCheckerMsg;
  
  for (int i=0;i<2;i++)
    butterflyInfoStatus[i] = true;

  for (int i=0;i<2;i++)
    if (butterflyCheckerMsg.voltage[i]<14 || butterflyCheckerMsg.voltage[i]>26)
      butterflyInfoStatus[i] = false;
      
  _updater.update();      
  
  
}

bool butterflyChecker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool butterflyOk = true;
  
  for (int i=0;i<2;i++){
    
    if (butterflyInfoStatus[i] == false){
      stat.addf("BatteryMsg", 
        "Battery %d Out of bounds: %f", i+1 ,msg.voltage[i]);
      butterflyOk = false;
    }
    
  }
  
  if (butterflyOk)
    stat.add("BatteryMsg", "All Ok");
  
  return butterflyOk;
    
}
