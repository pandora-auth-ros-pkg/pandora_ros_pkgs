#include "bounds_diagnostics/controllers_and_sensors/overall_controllers_checker.h"

overallControllersChecker::overallControllersChecker() : 
    co2Ch("/sensors/co2",updater) , 
    compassCh("/sensors/compass",updater), 
    butterflyCh("/sensors/butterfly",updater), 
    irCh("/sensors/ir",updater), 
    headIrCh("/sensors/headIr",updater), 
    tpaCh("/sensors/tpa",updater),
    mlxCh("/sensors/mlx",updater), 
    sonarCh("/sensors/sonar",updater),
    headSonarCh("/sensors/headSonar",updater) {  
  
  updater.add("MsgBounds",this,&overallControllersChecker::msgDiagnostics);
  updater.setHardwareID("none");

}
      
void overallControllersChecker::msgDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool allOk = true;  
  
  allOk = allOk & co2Ch.sensorCheckerDiagnostics(stat);
  allOk = allOk & compassCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & butterflyCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & irCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & headIrCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & tpaCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & mlxCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & sonarCh.sensorCheckerDiagnostics(stat);
  allOk = allOk & headSonarCh.sensorCheckerDiagnostics(stat);

  if (allOk)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All Ok");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, 
      "Some messages out of bounds");
    
}
