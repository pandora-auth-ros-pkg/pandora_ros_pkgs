#include "bounds_diagnostics/overall_checker.h"

#include "bounds_diagnostics/main_motor_control/main_motor_state_checker.h"
#include <diagnostic_updater/diagnostic_updater.h>


class overallMainMotorControlChecker : public overallChecker{
  
  mainMotorStateChecker mainMotorStateCh;
  
   
public:
  
  overallMainMotorControlChecker();
  void msgDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  
};


