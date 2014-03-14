#include "bounds_diagnostics/main_motor_control/main_motor_state_checker.h"

void mainMotorStateChecker::subscriberActualCallback(
  const motorState & mainMotorStateCheckerMsg){

  msg = mainMotorStateCheckerMsg;

  if (mainMotorStateCheckerMsg.stateLeft<14 || 
          mainMotorStateCheckerMsg.stateLeft>26)
      stateLeftInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.stateRight<14 || 
          mainMotorStateCheckerMsg.stateRight>26)
      stateRightInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.currentLeft<14 || 
          mainMotorStateCheckerMsg.currentLeft>26)
      currentLeftInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.currentRight<14 || 
          mainMotorStateCheckerMsg.currentRight>26)
      currentRightInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.velocityLeft<14 || 
          mainMotorStateCheckerMsg.velocityLeft>26)
      velocityLeftInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.velocityRight<14 || 
          mainMotorStateCheckerMsg.velocityRight>26)
      velocityRightInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.velocityLinear<14 || 
          mainMotorStateCheckerMsg.velocityLinear>26)
      velocityAngularInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.velocityAngular<14 || 
          mainMotorStateCheckerMsg.velocityAngular>26)
      velocityLinearInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.powerLeft<14 || 
          mainMotorStateCheckerMsg.powerLeft>26)
      powerLeftInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.powerRight<14 || 
          mainMotorStateCheckerMsg.powerRight>26)
      powerRightInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.efficiencyLeft<14 || 
          mainMotorStateCheckerMsg.efficiencyLeft>26)
      efficiencyLeftInfoStatus = false;
      
  if (mainMotorStateCheckerMsg.efficiencyRight<14 || 
          mainMotorStateCheckerMsg.efficiencyRight>26)
      efficiencyRightInfoStatus = false;
      
  _updater.update();      
  
  
}

bool mainMotorStateChecker::sensorCheckerDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &stat){
  
  bool mainMotorStateOk = true;
  
  if (stateLeftInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "State left Out of bounds: %d", msg.stateLeft);
      mainMotorStateOk = false;
  }
  
  if (stateRightInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "State right Out of bounds: %f", msg.stateRight);
      mainMotorStateOk = false;
  }
  
  if (currentLeftInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Current left Out of bounds: %f", msg.currentLeft);
      mainMotorStateOk = false;
  }
  
  if (currentRightInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Current right Out of bounds: %f", msg.currentRight);
      mainMotorStateOk = false;
  }
  
  if (velocityLeftInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Velocity left Out of bounds: %f", msg.velocityLeft);
      mainMotorStateOk = false;
  }
  
  if (velocityRightInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Velocity right Out of bounds: %f", msg.velocityRight);
      mainMotorStateOk = false;
  }
  
  if (velocityLinearInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Velocity linear Out of bounds: %f", msg.velocityLinear);
      mainMotorStateOk = false;
  }
  
  if (velocityAngularInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Velocity angular Out of bounds: %f", msg.velocityAngular);
      mainMotorStateOk = false;
  }
  
  if (powerLeftInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Power left Out of bounds: %f", msg.powerLeft);
      mainMotorStateOk = false;
  }
  
  if (powerRightInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Power right Out of bounds: %f", msg.powerRight);
      mainMotorStateOk = false;
  }
  
  if (efficiencyLeftInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Efficiency left Out of bounds: %f", msg.efficiencyLeft);
      mainMotorStateOk = false;
  }
  
  if (efficiencyRightInfoStatus == false){
      stat.addf("MainMotorStateMsg", 
      "Efficiency right Out of bounds: %f", msg.efficiencyRight);
      mainMotorStateOk = false;
  }
  
  if (mainMotorStateOk)
    stat.add("MainMotorStateMsg", "All Ok");
  
  return mainMotorStateOk;
    
}
