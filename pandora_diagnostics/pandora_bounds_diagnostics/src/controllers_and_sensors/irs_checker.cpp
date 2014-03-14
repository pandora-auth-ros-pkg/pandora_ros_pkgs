#include "bounds_diagnostics/controllers_and_sensors/irs_checker.h"

void irChecker::subscriberActualCallback(const ir & irCheckerMsg){
  
  msg = irCheckerMsg;

  for (int i=0;i<4;i++) 
    irInfoStatus[i] = true;
    
  for (int i=0;i<4;i++)      
    if (irCheckerMsg.distance[i]<0 || irCheckerMsg.distance[i]>80)
      irInfoStatus[i] = false;
      
  _updater.update();
  
}

bool irChecker::sensorCheckerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool irOk = true;
  
  for (int i=0;i<4;i++){
    
    if (irInfoStatus[i] == false){
      stat.addf("IrMsg", "Infrared  %d Out of bounds: %f", i+1, msg.distance[i]);
      irOk = false;
    }
    
  }
  
  if (irOk)
    stat.add("IrMsg", "All Ok");
  
  return irOk;
    
}


void headIrChecker::subscriberActualCallback(const headIr & headIrCheckerMsg){
  
  msg = headIrCheckerMsg;
  
  headIrInfoStatus = true;

  if (headIrCheckerMsg.distance<0 || headIrCheckerMsg.distance>80)
    headIrInfoStatus = false;
    
  _updater.update();
  
}

bool headIrChecker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool headIrOk = true;
  
  if (headIrInfoStatus == true){
    stat.add("HeadIrMsg", "All Ok");
  }
  else{
    stat.addf("HeadIrMsg", "Out of bounds: %f", msg.distance);
    headIrOk = false;
  }
  
  return headIrOk;
    
}
