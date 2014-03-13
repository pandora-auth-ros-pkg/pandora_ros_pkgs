// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_

#include <stdexcept>
#include <string>

class AlertException : public std::runtime_error {
 public:
  explicit AlertException(const std::string errorDescription) :
     std::runtime_error(errorDescription) {}
};

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_
