#include <diagnostic_updater/diagnostic_updater.h>

#ifndef OVERALL_CHECKER_H
#define OVERALL_CHECKER_H

class overallChecker{
  
protected:
   
  diagnostic_updater::Updater updater;

public:
  
  overallChecker(){};
  virtual void msgDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) = 0;
  
};

#endif
