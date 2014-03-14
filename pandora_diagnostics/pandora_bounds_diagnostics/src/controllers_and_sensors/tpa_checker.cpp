#include "bounds_diagnostics/controllers_and_sensors/tpa_checker.h"

void tpaChecker::subscriberActualCallback(const tpa & tpaCheckerMsg){
  
  msg=tpaCheckerMsg;
  
  for (int i=0;i<5;i++)
    for  (int j=0;j<9;j++)
      tpaInfoStatus[i][j] = true;

  for (int i=0;i<5;i++){
    
    for (int j=0;j<8;j++)
      if (tpaCheckerMsg.pixelTemp[i]<10 || tpaCheckerMsg.pixelTemp[i]>45)
        tpaInfoStatus[i][j] = false;
    
    if (tpaCheckerMsg.ambientTemp<0 || tpaCheckerMsg.ambientTemp>45)
      tpaInfoStatus[i][8] = false;
      
  }
  
  for (int i=0;i<5;i++){
    
    if (i == tpaCheckerMsg.CENTER)
      tpaPos[i] = "CENTER";
    else if (i == tpaCheckerMsg.LEFT)
      tpaPos[i] = "LEFT";
    else if (i == tpaCheckerMsg.RIGHT)
      tpaPos[i] = "RIGHT";
    else if (i == tpaCheckerMsg.LEFT_REAR)
      tpaPos[i] = "LEFT_REAR";
    else if (i == tpaCheckerMsg.RIGHT_REAR)
      tpaPos[i] = "RIGHT_REAR";
  }
  
  _updater.update();

}

bool tpaChecker::sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool tpaOk = true;
  
  for (int i=0;i<5;i++){
    
    for (int j=0;j<8;j++){
      if (tpaInfoStatus[i][j] == false){
        stat.addf("tpaMsg","%s Tpa: pixelTemp No %d Out of bounds: %f",
          tpaPos[i].c_str(), j+1, msg.pixelTemp[j]);
        tpaOk = false;
      }
    }
    
    if (tpaInfoStatus[i][8] == false){
      stat.addf("tpaMsg","%s Tpa: ambientTemp Out of bounds: %f", 
        tpaPos[i].c_str(), msg.ambientTemp);
      tpaOk = false;
    }
  
  }
  
  if (tpaOk){
    stat.add("tpaMsg","All Ok");
  }
    
  return tpaOk;
}
