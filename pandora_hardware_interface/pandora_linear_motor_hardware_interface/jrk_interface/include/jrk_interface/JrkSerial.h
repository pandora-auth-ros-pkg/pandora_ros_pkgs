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
* Author:
*********************************************************************/


#ifndef JRK_INTERFACE_JRKSERIAL_H
#define JRK_INTERFACE_JRKSERIAL_H

#include <stdexcept>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <serial/serial.h>
#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include <ros/ros.h>


namespace pandora_hardware_interface
{
namespace linear
{

class JrkSerial: private boost::noncopyable
{
  private:
    boost::scoped_ptr<serial::Serial> serialPtr_;
    const std::string device_;
    const int speed_;
    const int timeout_;
  public:

    /*!
     * @brief Constructor. Initializes serial interface parameters (speed,device...)
     * @param [in] device String indexing to serial com port buffer [Full directory under OS must be provided]
     * @param [in] speed Serial Communication speed.
     * @param [in] timeout Serial Communication Timeout value.
     */
    JrkSerial(const std::string& device,
                int speed,
                int timeout);
    /*!
     * @brief Deconstructor. Closes Device on destruction.     */
    ~JrkSerial();
    
    /*!
     * @fn void open_device()
     * @brief This method is used for opening [Linear Motor] device communication port with preconfigured parameters.
     */
    void openDevice();
    
    /*!
     * @fn int readVariable(const unsigned char command)
     * @brief Reads variable value due to given command.
     * @param [in] command Command send to [Linear Motor] controller.
     * @return Varible readen from [Linear Motor]
     */
    int readVariable(const unsigned char command);
  
    /*!
     * @fn bool write(const uint8_t *data, size_t size)
     * @brief Write command to [Linear Motor] controller.
     * @param [in] data Output Data buffer.
     * @param [in] size Output Data buffer size. 
     * @retval TRUE Success on writing data to output buffer.
     * @retval False Error writing to output buffer.
     */
    bool write(const uint8_t *data, size_t size);
    
    /*!
     * @fn bool read(const uint8_t *data, size_t size)
     * @brief Read command to [Linear Motor] controller.
     * @param [in] data Input Data buffer.
     * @param [in] size Input Data buffer size. 
     * @retval TRUE Success on read data from input buffer.
     * @retval False Error reading from input buffer.
     */
    bool read(uint8_t *data, size_t size);

    /*!
     * @fn int readErrors(unsigned char command)
     * @brief Read errors from [Linear Motor] controller.
     * @param [in] command Command for reading errors.
     */
    int readErrors(unsigned char command);

    /*!
     * @fn int readFeedback()
     * @brief Read position feedback from [Linear Motor].
     */
    int readFeedback();

    /*!
     * @fn int readScaledFeedback()
     * @brief Asks linear motor for scaled position feedback value.      
     * @return Scaled Feedback value.
     */
    int readScaledFeedback();
    
    /*!
     * @fn int readDutyCycle()
     * @brief Asks linear motor for duty cycle value. Returns duty cycle value.
     * @return Duty Cycle.
     */
    int readDutyCycle();
    
    /*!
     * @fn int readTarget()
     * @brief Asks linear motor for position target value. Returns position target value.
     * @return Target position value.
     */
    int readTarget();
   
    /*!
     * @fn int setTarget(unsigned short target)
     * @brief Sends position target command [Linear Motor].
     * @param target Target position value.
     */
    int setTarget(unsigned short target);

    /*!
     * @fn int getErrors();
     * @brief Read and print errors on [Linear Motor].
     * @return Error integer variable.
     */ 
    int getErrors();
    
    /*!
     * @fn void printErrors(int errors)
     * @brief Prints errors reported from [Linear Motor].
     * @param errors Error index.
     */
    void printErrors(int errors);
    
    /*!
     * @fn int clearErrors()
     * @brief This method sends the ERRORS_HALTING_VARIABLE char to [Linear Motor] to clear errors reported on startup.
     * @return 
     */
    int clearErrors();
    
    /*!
     * @fn void closeDevice()
     * @brief This method is used to close linear motor serial com port.
     */
    void closeDevice();
};
} //namespace linear
} //namespace pandora_hardware_interface
#endif  // JRK_INTERFACE_JRKSERIAL_H
