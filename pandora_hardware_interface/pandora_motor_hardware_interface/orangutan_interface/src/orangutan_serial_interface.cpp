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
* Author:  Chris Zalidis
*********************************************************************/
#include "orangutan_interface/orangutan_serial_interface.h"

namespace pandora_hardware_interface {
  
  namespace motor {
    
    OrangutanSerialInterface::OrangutanSerialInterface(
      const std::string& device,
      int speed,
      int timeout)
    :
      serialPtr_(NULL),
      device_(device),
      speed_(speed),
      timeout_(timeout),
      leftSpeed_(0),
      rightSpeed_(0)
    {
    }
    
    void OrangutanSerialInterface::init()
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
          ROS_FATAL("[motors] Cannot open port!!");
          ROS_FATAL("%s", ex.what());
          exit(-1);
        }
      }
      else
      {
        throw std::logic_error("Init called twice!!");
      }
    }
    
    void OrangutanSerialInterface::read()
    {
      if (serialPtr_ == NULL)
        throw std::logic_error("read() called before init()!");
        
      // dummy until feedback received from controllers
    }
    
    void OrangutanSerialInterface::write(int leftSpeed, int rightSpeed)
    {
      if (serialPtr_ == NULL)
        throw std::logic_error("write() called before init()!");
      
      
      std::string left = boost::str( boost::format("%+04d") % leftSpeed );
      std::string right = boost::str( boost::format("%+04d") % rightSpeed );
      
      std::string command = "$L" + left + "R" + right; // add checksum
      
      if (serialPtr_->write(command) != command.size()*sizeof(char))
        throw std::runtime_error("write() failed! Communication problem?");
        
      
      // dummy feedback!
      leftSpeed_ = leftSpeed;
      rightSpeed_ = rightSpeed;
      
    }
    
  }  // namespace motor
}  // namespace pandora_hardware_interface
