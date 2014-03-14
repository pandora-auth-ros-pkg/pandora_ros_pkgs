#include "bounds_diagnostics/controllers_and_sensors/mlx_checker.h"

void mlxChecker::subscriberActualCallback(const mlx & mlxCheckerMsg){

  msg = mlxCheckerMsg;
  
  for (int i=0;i<2;i++)
    mlxInfoStatus[i] = true;

  for (int i=0;i<2;i++){
    if (mlxCheckerMsg.mlxTemp[i]<0 || mlxCheckerMsg.mlxTemp[i]>80)
      mlxInfoStatus[i] = false;      
  }
  
  _updater.update();
}

bool mlxChecker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool mlxOk = true;
  
  for (int i=0;i<2;i++){
    if (mlxInfoStatus[i] == false){
      stat.addf("MlxMsg", "MlxTemp %d Out of bounds: %f", i+1, msg.mlxTemp[i]);
      mlxOk = false;
    }
  }
  
  if (mlxOk)
    stat.add("MlxMsg", "All Ok");
  
  return mlxOk;  
}
