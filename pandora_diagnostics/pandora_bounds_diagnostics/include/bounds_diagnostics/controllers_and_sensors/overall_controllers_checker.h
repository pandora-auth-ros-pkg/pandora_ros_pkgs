#include "bounds_diagnostics/overall_checker.h"

#include "bounds_diagnostics/controllers_and_sensors/butterfly_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/co2_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/compass_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/irs_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/mlx_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/sonars_checker.h"
#include "bounds_diagnostics/controllers_and_sensors/tpa_checker.h"
#include <diagnostic_updater/diagnostic_updater.h>


class overallControllersChecker : public overallChecker{
  
  co2Checker co2Ch;
  compassChecker compassCh;
  butterflyChecker butterflyCh;
  irChecker irCh;
  headIrChecker headIrCh;
  tpaChecker tpaCh;
     mlxChecker mlxCh;
     sonarChecker sonarCh;
     headSonarChecker headSonarCh;
   
public:
  
  overallControllersChecker();
  void msgDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
  
};


