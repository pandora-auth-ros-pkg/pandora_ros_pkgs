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
* Author: Michael Niarchos
* Author: Chris Zalidis
*********************************************************************/

#include "uController/hardware_interface.h"

using namespace std;

HardwareInterface::HardwareInterface(const std::string& device, 
                                      const int& speed,
                                      const int& timeout) :
serialPtr_(NULL),                                 
CRC_(0),
device_(device),
speed_(speed),
timeout_(timeout)
{
  
}

HardwareInterface::~HardwareInterface()
{
  delete serialPtr_;
}

void HardwareInterface::init()
{
  if (serialPtr_ == NULL) {
    try {
    serialPtr_ = 
          new serial::Serial(device_, 
                              speed_, 
                              serial::Timeout::simpleTimeout(timeout_)
                            );
    }
    catch (serial::IOException& ex) {
      ROS_FATAL("[uController] Cannot open port!!");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }
  else {
    throw std::runtime_error("Init called twice!!");
  }
  
  
}

int HardwareInterface::readMessageType()
{

  uint8_t command[2];
  int size = 2; //initialize size of data.Size of command data is 1 byte

  CRC_ = 0;

  serialPtr_->read(command, size);

  if ((int)command[0] == 12 && (int)command[1] == 10)
  {
    CRC_ += (int)command[0] + (int)command[1];
    return READ_SIZE_STATE;
    //return READ_DATA_STATE;
  }
  else
    return IDLE_STATE;

}

int HardwareInterface::readSize(uint16_t *dataSize)
{

  uint8_t dataSiz[5]; //dataSiz buffer where the size of data is written
  int size = 5; //Initialize size of dataSiz.
  uint16_t temp = 0;

  serialPtr_->read(dataSiz, size);

  for(int i = 0; i < size; i++)
  {
    ROS_DEBUG("[uController] %c", dataSiz[i]);
    CRC_ += (int)dataSiz[i];
  }

  for(int i = 0; i < 4; i++)
  {
    if((int)dataSiz[i] < 58)
    {
      temp += ((int)dataSiz[i] - 48) * pow(16, 3 - i);
    }
    else
    {
      temp += ((int)dataSiz[i] - 55) * pow(16, 3 - i);
    }
  }
  //cout<<temp<<endl;
  if (temp == 0)
  {
    return IDLE_STATE;
  }
  else
  {
    *dataSize = temp;
    return READ_DATA_STATE;
  }
}

int HardwareInterface::readData(uint16_t dataSize, unsigned char *dataBuffer)
{

  //cout<<dataSize<<endl;
  if(serialPtr_->read((uint8_t*)dataBuffer, dataSize - 5) != dataSize - 5)
    ROS_DEBUG("[uController] ERROR!!\n");

  for(int i = 0; i < dataSize; i++)
  {
    ROS_DEBUG("[uController] %c", dataBuffer[i]);
    CRC_ += (int)dataBuffer[i];
  }

  //return READ_CONTROL_STATE
  return READ_CRC_STATE;

}

int HardwareInterface::readCRC()
{

  uint8_t crc[4];
  uint8_t eot;
  int size = 4;
  uint16_t temp;

  serialPtr_->read(crc, size);
  //to change????
  serialPtr_->read(&eot, 1);

  temp = 0;

  for(int i = 0; i < size; i++)
  {
    //ROS_DEBUG("%c",crc[i]);
    if((int)crc[i] < 58)
      temp += ((int)crc[i] - 48) * pow(16, size - 1 - i);
    else
      temp += ((int)crc[i] - 55) * pow(16, size - 1 - i);
  }
  //cout<<endl;
  ROS_DEBUG("[uController] PC_CRC: %d\n", CRC_);
  ROS_DEBUG("[uController] uController_CRC: %d\n", temp);
  if (CRC_ == temp)
  {
    ROS_DEBUG("[uController] successful read!\n");
    CRC_ = 0;
    //return IDLE_STATE;
    return ACK_STATE;
  }
  else
  {

    ROS_DEBUG("[uController] data error!\n");
    CRC_ = 0;
    //return IDLE_STATE;
    return NAK_STATE;
  }

}

bool HardwareInterface::write(const uint8_t *data, size_t size)
{
  if (serialPtr_->write(data, size) == size) {
    return true;
  }
  else {
    return false;
  }
}
