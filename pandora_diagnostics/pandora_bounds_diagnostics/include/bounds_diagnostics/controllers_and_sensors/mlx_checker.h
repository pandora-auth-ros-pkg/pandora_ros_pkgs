#ifndef MLX_CHECKER_H
#define MLX_CHECKER_H

#include "controllers_and_sensors_communications/mlxTempMsg.h"

#include "bounds_diagnostics/bounds_check_subscriber.h"

#include <diagnostic_updater/diagnostic_updater.h>

typedef controllers_and_sensors_communications::mlxTempMsg  mlx;

class mlxChecker : public BoundsCheckSubscriber<mlx>{
  
  bool mlxInfoStatus[2];
  mlx msg;

public:

  mlxChecker(std::string topic, diagnostic_updater::Updater & updater): 
    BoundsCheckSubscriber(topic,updater){};

  void subscriberActualCallback(const mlx & mlxCheckerMsg);

  bool sensorCheckerDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat);
  
  ~mlxChecker() {};  
  
};

#endif
