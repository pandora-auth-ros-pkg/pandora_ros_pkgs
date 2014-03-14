#include "bounds_diagnostics/controllers_and_sensors/co2_checker.h"

void co2Checker::subscriberActualCallback(const co2 & co2CheckerMsg){

  msg = co2CheckerMsg;
  
  co2InfoStatus = true; 
    
  if (msg.ppm<200 || msg.ppm>50000)
    co2InfoStatus = false;
    
  _updater.update();

}

bool co2Checker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool co2Ok = true;
  
  if (co2InfoStatus == true){
    stat.add("Co2Msg", "All Ok");
  }
  else{
    stat.addf("Co2Msg", "Co2 Out of bounds: %d", msg.ppm);
    co2Ok = false;
  }
  
  return co2Ok;
}
