/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Vasilis Bosdelekidis
* Author: George Kouros
*********************************************************************/


#ifndef LINEAR_ACTUATOR_COM_INTERFACE_JRK_COM_INTERFACE_H
#define LINEAR_ACTUATOR_COM_INTERFACE_JRK_COM_INTERFACE_H

#include <stdexcept>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <serial/serial.h>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include "linear_actuator_com_interface/abstract_linear_actuator_com_interface.h"

namespace pandora_hardware_interface
{
namespace linear_actuator
{
  /**
  @class Class JrkComInterface
  @brief Class used to communicate with the jrk controller and control the linear actuator actuator
  **/
  class JrkComInterface : public AbstractLinearActuatorComInterface
  {
   public:
    /**
    @brief Default Constructor. Initializes serial interface parameters (speed, device...)
    @param device [const std::string&] device : device port name (/dev/<port>)
    @param speed [int] : serial communication speed
    @param timeout [int] :serial communication timeout value
    **/
    JrkComInterface(const std::string& device, int speed, int timeout);

    /**
    @brief Default Destructor. Closes Device on destruction
    **/
    ~JrkComInterface();

    /**
    @brief Opens serial communication port
    @return void
    **/
    void init();

    /**
    @brief opens serial communication port
    @return void
    **/
    void openDevice();

    /**
     @brief Closes serial communication port
     @return void
     **/
    void closeDevice();

    /**
    @brief Reads variable, according to command value
    @param command [const unsigned char] : command to the linear actuator controller
    @return [int] : variable value requested through command
    **/
    int readVariable(const unsigned char command);

    /**
    @brief Write command to [Linear Actuator] controller
    @param data [const uint8_t*] : Output Data buffer
    @param size [size_t] : Output Data buffer size
    @return TRUE : Success on writing data to output buffer
    @return FALSE : Error writing to output buffer
    **/
    bool write(const uint8_t* data, size_t size);

    /**
    @brief Read command of linear actuator controller
    @param data [uint8_t*] : input data buffer
    @param size [size_t] : input data buffer size
    @return TRUE : Success on read data from input buffer
    @return False : Error reading from input buffer
    **/
    bool read(uint8_t* data, size_t size);

    /**
    @brief Read errors from [Linear Actuator] controller
    @param command [unsigned char] : command for reading errors
    return int : error code
    **/
    int readErrors(unsigned char command);

    /**
    @brief Read position feedback from [Linear Actuator]
    @return int : feedback
    **/
    int readFeedback();

    /**
    @brief Reads position of linear joint in cm
    @return Scaled Feedback value
    **/
    float readScaledFeedback();

    /**
    @brief Asks linear actuator for duty cycle value. Returns duty cycle value
    @return Duty Cycle
    **/
    int readDutyCycle();

    /**
    @brief Asks linear actuator for position target value
    @return Target position value
    **/
    int readTarget();

    /**
    @brief Sends position target command [Linear Actuator]
    @param target Target position value
    @return true : success in setting target of linear actuator joint
    @return false : failure in setting target of linear actuator joint
    **/
    bool setTarget(float target);

    /**
    @fn int getErrors();
    @brief Read and print errors on [Linear Actuator]
    @return Error integer variable
    **/
    int getErrors();

    /**
    @brief Prints errors reported from [Linear Actuator]
    @param errors Error index
    **/
    void printErrors(int errors);

    /**
    @brief Sends the ERRORS_HALTING_VARIABLE char to [Linear Actuator] to clear errors reported on startup
    @return
    **/
    int clearErrors();

   private:
    boost::scoped_ptr<serial::Serial> serialPtr_;  //!< serial port ptr
    const std::string device_;  //!< port name
    const int speed_;  //!< serial communication speed
    const int timeout_;  //!< timeout for serial port connection
  };
}  // namespace linear_actuator
}  // namespace pandora_hardware_interface

#endif  // LINEAR_ACTUATOR_COM_INTERFACE_JRK_COM_INTERFACE_H
