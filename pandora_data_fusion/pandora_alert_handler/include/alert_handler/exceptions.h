// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_EXCEPTIONS_H
#define ALERT_HANDLER_EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class AlertException : public std::runtime_error
    {
      public:

        explicit AlertException(const std::string errorDescription) :
          std::runtime_error(errorDescription) {}

    };

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_EXCEPTIONS_H
