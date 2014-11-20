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

#include <jrk_interface/JrkSerial.h>
#include <ros/ros.h>
#include <jrk_interface/JrkDefinitions.h>


namespace pandora_hardware_interface
{
namespace linear
{

JrkSerial::JrkSerial(const std::string& device,
                      int speed,
                      int timeout):
  serialPtr_(NULL),
  device_(device),
  speed_(speed),
  timeout_(timeout)
{
}


JrkSerial::~JrkSerial()
{
  if (serialPtr_->isOpen()) {   closeDevice();  }
}


void JrkSerial::openDevice()
{
  if (serialPtr_ == NULL)
  {
    try
    {
      serialPtr_.reset(
      new serial::Serial(
        device_,
        speed_,
        serial::Timeout::simpleTimeout(timeout_)));
    }
    catch (serial::IOException& ex)
    {
      ROS_FATAL("[Linear Motor]: Failed to open serial port");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }
  else
  {
    throw std::logic_error("[Linear Motor]: Declined second call on open_device");
  }
  /* <Flushes (discards) not-send data (data still in the UART send buffer) and/or flushes (discards) received data (data already in the UART receive buffer)>*/
  //serialPtr_->flushInput; /* <Flush received, but unread data> */
  //serialPtr_->flushOutput; /* <Flush written, but not send data> */ 
  serialPtr_->flush(); /* <Flush both Input and output buffers> */

  clearErrors(); /* <Call to linear motor controller to lean any errors on startup> */
}


void JrkSerial::closeDevice()
{
  serialPtr_->flush();  /* <Flush both input and output  buffers on exit> */
  serialPtr_->close();
}

bool JrkSerial::write(const uint8_t *data, size_t size)
{
  serialPtr_->flushOutput(); /* <Flush written but not send data> */
  if (serialPtr_->write(data, size) == size)  {return true;}
  else  {return false;}
}

bool JrkSerial::read(uint8_t *data, size_t size)
{
  if (serialPtr_->read(data,size) == size)
  {
    serialPtr_->flushInput(); /* <Flush received, but unread data> */
    return true;
  }
  else
  {
    serialPtr_->flushInput();
    return false;
  }
}

int JrkSerial::setTarget(unsigned short target)
{
  uint8_t command[] = {0xB3, 0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
  if ( !this->write(command, sizeof(command)) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error writing > " << strerror(errno));
    return -1;
  }
  return 0;
}


int JrkSerial::readVariable(const unsigned char command)
{
  uint8_t message[] = {command};
  if ( !this->write(message, 1) )
  {
    ROS_ERROR_STREAM("[Linear Motor]: Error writing > " << strerror(errno));
    return -1;
  }
  uint8_t response[2];
  if ( !this->read(response, 2) )
  {
    ROS_ERROR_STREAM("[Linear Motor]: Error reading > " << strerror(errno));
    return -1;
  }
  return response[0] + 256*response[1];
}


int JrkSerial::readFeedback()
  {return readVariable(FEEDBACK_VARIABLE);}


int JrkSerial::readScaledFeedback()
  {return readVariable(SCALED_FEEDBACK_VARIABLE);}


int JrkSerial::readDutyCycle()
  {return readVariable(DUTY_CYCLE_VARIABLE);}


int JrkSerial::readTarget()
  {return readVariable(TARGET_VARIABLE);}


int JrkSerial::getErrors()
{
  int errors = readErrors(ERRORS_HALTING_VARIABLE);
  printErrors(errors);
  return errors;
}


int JrkSerial::readErrors(unsigned char command)
{
  uint8_t message[] = {command};
  if ( !this->write(message, 1) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error writing >" << strerror(errno));
    return -1;
  }
  unsigned char response[2];
  if ( !this->read(static_cast<uint8_t*>(response), 2) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error reading >" << strerror(errno));
    return -1;
  }
  return response[0];
}

void JrkSerial::printErrors(int errors)
{
  if ( errors >= 32 && errors <= 63 )
  {
    ROS_ERROR("[Linear Motor Error]: Feedback disconenct");
    return;
  }
  if ( errors >= 64 && errors <= 127 )
  {
    ROS_ERROR("[Linear Motor Error]: Maximum current exceeded");
    return;
  }
  else
  {
    switch (errors)
    {
    case 1:
      ROS_ERROR("[Linear Motor Error]: Awaiting command");
      break;
    case 2:
      ROS_ERROR("[Linear Motor Error]: No power");
      break;
    case 3:
      ROS_ERROR("[Linear Motor Error]: Awaiting Command and No power");
      break;
    case 4:
      ROS_ERROR("[Linear Motor Error]: Motor driver");
    case 5:
      ROS_ERROR("[Linear Motor Error]: Awaiting Command and Motor driver");
    case 6:
      ROS_ERROR("[Linear Motor Error]: No power and Motor driver");
      break;
    case 7:
      ROS_ERROR("[Linear Motor Error]: Awaiting command and No power and Motor driver");
      break;
    case 8:
      ROS_ERROR("[Linear Motor Error]: Input invalid");
      break;
    default:
      ROS_ERROR("[Linear Motor Error]: NONE!");
    }
  }
}

int JrkSerial::clearErrors()
{
  /*Gets error flags halting and clears any latched errors*/
  uint8_t command[] = {ERRORS_HALTING_VARIABLE};
  if ( !this->write(command, 1) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error writing > " << strerror(errno));
    return -1;
  }
  return 0;
}

} //namespace linear
} //namespace pandora_hardware_interface
