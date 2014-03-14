#include "bounds_diagnostics/main_motor_control/overall_main_motor_control_checker.h"

overallMainMotorControlChecker::overallMainMotorControlChecker(): 
  mainMotorStateCh("/mainMotorControl/state",updater) {  
      
      updater.add(
        "MsgBounds",this,&overallMainMotorControlChecker::msgDiagnostics);
      updater.setHardwareID("none");

}
      
void overallMainMotorControlChecker::msgDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool allOk;  
  
  allOk = mainMotorStateCh.sensorCheckerDiagnostics(stat);
  

  if (allOk)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All Ok");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, 
        "Some messages out of bounds");
    
}
