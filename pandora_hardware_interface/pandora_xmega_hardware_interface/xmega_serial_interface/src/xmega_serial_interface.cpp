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

#include "xmega_serial_interface/xmega_serial_interface.h"

namespace pandora_hardware_interface
{
namespace xmega
{


XmegaSerialInterface::XmegaSerialInterface(const std::string& device,
                                      int speed,
                                      int timeout) :
  serialIO_(device, speed, timeout),
  t1_(0),
  t2_(0),
  dataSize_(0),
  currentState_(IDLE_STATE)
{
}

void XmegaSerialInterface::init()
{
  serialIO_.init();
}

void XmegaSerialInterface::read()
{
  receiveData();
}

//------------- Private Members -------------------//

void XmegaSerialInterface::receiveData()
{
  timeval start, current;
  int32_t ms_elapsed, seconds, useconds;
  int timer_flag = 1;

  int currentState_ = IDLE_STATE;
  int previousState = IDLE_STATE;
  int counter_ack = 0;
  int counter_nak = 0;
  int counter_timeout = 0;

  uint8_t ACK[] = {6};
  uint8_t NAK[] = {2, 1};

  bool done = false;

  gettimeofday(&start, NULL);

  while (!done)
  {
    ROS_DEBUG("[xmega] current state: %d !", currentState_);

    ROS_DEBUG("[xmega] COUNTER_TIMEOUT: %d", counter_timeout);
    ROS_DEBUG("[xmega] COUNTER_ACK: %d", counter_ack);
    ROS_DEBUG("[xmega] COUNTER_NAK: %d", counter_nak);
    switch (currentState_)
    {
  /* <IDLE_STATE indicates the state waiting for start of transmission characters> */
    case IDLE_STATE:
      currentState_ = serialIO_.readMessageType();
      gettimeofday(&current, NULL);
      seconds = current.tv_sec - start.tv_sec;
      useconds = current.tv_usec - start.tv_usec;
      ms_elapsed = ((seconds) * 1000 + useconds / 1000.0) + 0.5;  // +0.5 is used for rounding positive values
      if (ms_elapsed >= 20000)
        throw std::runtime_error(
          "xmega doesn't seem to respond! Is it connected?");

      if (timer_flag == 0)
      {
        if (ms_elapsed >= 500)
        {
          currentState_ = previousState;
          timer_flag = 0;
          ms_elapsed = 0;
          counter_timeout++;
        }
      }
      break;
    case READ_SIZE_STATE:
      gettimeofday(&tim_, NULL);
      t1_ = tim_.tv_sec + (tim_.tv_usec / 1000000.0);
      currentState_ = serialIO_.readSize(&dataSize_);
      break;
    case READ_DATA_STATE:
      pdataBuffer_ = new unsigned char[dataSize_];
      memset(pdataBuffer_, 0x00, dataSize_);
      currentState_ = serialIO_.readData(dataSize_, pdataBuffer_);
      break;
    case READ_CRC_STATE:
      currentState_ = serialIO_.readCRC();
      break;
    case ACK_STATE:
      previousState = ACK_STATE;
      timer_flag = 1;
      gettimeofday(&start, NULL);
      if (!serialIO_.write(ACK, 1))
        ROS_ERROR("[xmega] Failed to write ACK!\n");
      counter_ack++;
      ROS_DEBUG("[xmega] counter_ack: %d \n", counter_ack);
      currentState_ = PROCESS_DATA_STATE;
      break;
    case NAK_STATE:
      previousState = NAK_STATE;
      timer_flag = 1;
      gettimeofday(&start, NULL);
      if (!serialIO_.write(NAK, 2))
        ROS_ERROR("[xmega] Failed to write NAK!\n");
      currentState_ = IDLE_STATE;
      counter_nak++;
      done = true;
      break;
    case PROCESS_DATA_STATE:
      currentState_ = processData();
      gettimeofday(&tim_, NULL);
      t2_ = tim_.tv_sec + (tim_.tv_usec / 1000000.0);
      ROS_DEBUG("[xmega] %.8f seconds elapsed\n", t2_ - t1_);
      done = true;
      break;
    default:
      currentState_ = IDLE_STATE;
      dataSize_ = 0;
      pdataBuffer_ = NULL;
      break;
    }
  }
  delete[] pdataBuffer_;  // cleanup
}

int XmegaSerialInterface::processData()
{
  int bufferPointer = 0;
  int readState = SENSOR_ID;
  int type;
  char temp[2];

  while (bufferPointer < dataSize_)
  {
    switch (readState)
    {
    case SENSOR_ID:
      ROS_DEBUG("Sensor ID: %c%c\n", pdataBuffer_[bufferPointer], pdataBuffer_[bufferPointer + 1]);
      bufferPointer += 3;  // ' ' after sensor id
      readState = SENSOR_TYPE;
      break;
    case SENSOR_TYPE:
      ROS_DEBUG("Sensor type: %c%c\n", pdataBuffer_[bufferPointer], pdataBuffer_[bufferPointer + 1]);
      temp[0] = pdataBuffer_[bufferPointer];
      temp[1] = pdataBuffer_[bufferPointer + 1];
      type = myatoi(temp, 2);
      bufferPointer += 3; // ' ' after sensor type
      readState = SENSOR_I2C_ADDRESS;
      if (type == BATTERY)  // battery, not i2c sensor
        readState = SENSOR_DATA;
      if (type == ENCODER) // encoder, not i2c sensor
        readState = SENSOR_DATA;
      break;
    case SENSOR_I2C_ADDRESS:
      ROS_DEBUG("Sensor I2C address: %c%c\n", pdataBuffer_[bufferPointer], pdataBuffer_[bufferPointer + 1]);
      temp[0] = pdataBuffer_[bufferPointer];
      temp[1] = pdataBuffer_[bufferPointer + 1];
      getSensor(type)->i2c_address = myatoi(temp, 2);
      bufferPointer += 3;  // ' ' after sensor i2c address
      readState = SENSOR_STATUS;
      break;
    case SENSOR_STATUS:
      ROS_DEBUG("Sensor status: %c%c\n", pdataBuffer_[bufferPointer], pdataBuffer_[bufferPointer + 1]);
      temp[0] = pdataBuffer_[bufferPointer];
      temp[1] = pdataBuffer_[bufferPointer + 1];
      getSensor(type)->status = myatoi(temp, 2);
      bufferPointer += 3;  // ' ' after sensor status
      readState = SENSOR_CURRENT_STATE;
      break;
    case SENSOR_CURRENT_STATE:
      ROS_DEBUG("Sensor current state: %c%c\n", pdataBuffer_[bufferPointer], pdataBuffer_[bufferPointer + 1]);
      temp[0] = pdataBuffer_[bufferPointer];
      temp[1] = pdataBuffer_[bufferPointer + 1];
      getSensor(type)->state = myatoi(temp, 2);
      bufferPointer += 3;
      readState = SENSOR_DATA;
      break;
    case SENSOR_DATA:
    {
      int i = 0;
      while ( pdataBuffer_[bufferPointer] != '\n' )
      {
        temp[0] = pdataBuffer_[bufferPointer++];
        temp[1] = pdataBuffer_[bufferPointer++];
        getSensor(type)->data[i] = myatoi(temp, 2);
        ROS_DEBUG("%d ", getSensor(type)->data[i]);
        i++;
      }
      bufferPointer++;  // LF after Data
      ROS_DEBUG("POINTER: %d\n", bufferPointer);
      getSensor(type)->handleData();
      readState = SENSOR_ID;
      ROS_DEBUG("\n");
      break;
    }
    default:
      return 0;  // processing error
    }
  }
  return 1;  // successful processing
}


SensorBase* XmegaSerialInterface::getSensor(int sensorType)
{
  switch (sensorType)
  {
  case BATTERY:
    return &batterySensor_;
  case SRF05_TINY:
    return &rangeSensors_;
  case ENCODER:
    return &encoderSensor_;
  default:
    return &defaultSensor_;
  }
}

//----------SerialIO------------------------//

SerialIO::SerialIO(const std::string& device,
                    int speed,
                    int timeout) :
  serialPtr_(NULL),
  device_(device),
  speed_(speed),
  timeout_(timeout),
  CRC_(0)
{
}

void SerialIO::init()
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
      ROS_FATAL("[xmega-serialIO] Cannot open port!!");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }
  else
  {
    throw std::logic_error("Init called twice!!");
  }
}

int SerialIO::readMessageType()
{
  uint8_t command[2];
  int size = 2; //initialize size of data.Size of command data is 1 byte

  CRC_ = 0;

  serialPtr_->read(command, size);

  /* <If start of data package chars (0x0C , 0x0A)> */
  if (static_cast<int>(command[0]) == 12 && static_cast<int>(command[1]) == 10)
  {
    CRC_ += static_cast<int>(command[0]) + static_cast<int>(command[1]);
    return READ_SIZE_STATE;
    //return READ_DATA_STATE;
  }
  else
    return IDLE_STATE;
}

int SerialIO::readSize(uint16_t *dataSize_)
{
  uint8_t dataSiz[5]; //dataSiz buffer where the size of data is written
  int size = 5; //Initialize size of dataSiz.
  uint16_t temp = 0;

  serialPtr_->read(dataSiz, size);

  for (int i = 0; i < size; i++)
  {
    ROS_DEBUG("[xmega-serialIO] %c", dataSiz[i]);
    CRC_ += static_cast<int>(dataSiz[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    if (static_cast<int>(dataSiz[i]) < 58)
    {
      temp += (static_cast<int>(dataSiz[i]) - 48) * pow(16, 3 - i);
    }
    else
    {
      temp += (static_cast<int>(dataSiz[i]) - 55) * pow(16, 3 - i);
    }
  }
  if (temp == 0)
  {
    return IDLE_STATE;
  }
  else
  {
    *dataSize_ = temp;
    return READ_DATA_STATE;
  }
}

int SerialIO::readData(uint16_t dataSize_, unsigned char *dataBuffer)
{
  if (serialPtr_->read(static_cast<uint8_t*>(dataBuffer), dataSize_ - 5) != dataSize_ - 5)
    ROS_DEBUG("[xmega-serialIO] ERROR!!\n");

  for (int i = 0; i < dataSize_; i++) {
    ROS_DEBUG("[xmega-serialIO] %c", dataBuffer[i]);
    CRC_ += static_cast<int>(dataBuffer[i]);
  }

  return READ_CRC_STATE;
}

int SerialIO::readCRC()
{
  uint8_t crc[4];
  uint8_t eot;
  int size = 4;
  uint16_t temp;

  serialPtr_->read(crc, size);
  //to change????
  serialPtr_->read(&eot, 1);

  temp = 0;

  for (int i = 0; i < size; i++)
  {
    if (static_cast<int>(crc[i]) < 58)
      temp += (static_cast<int>(crc[i]) - 48) * pow(16, size - 1 - i);
    else
      temp += (static_cast<int>(crc[i]) - 55) * pow(16, size - 1 - i);
  }
  ROS_DEBUG("[xmega-serialIO] PC_CRC: %d\n", CRC_);
  ROS_DEBUG("[xmega-serialIO] uController_CRC: %d\n", temp);
  if (CRC_ == temp)
  {
    ROS_DEBUG("[xmega-serialIO] successful read!\n");
    CRC_ = 0;
    return ACK_STATE;
  }
  else
  {
    ROS_DEBUG("[xmega-serialIO] data error!\n");
    CRC_ = 0;
    return NAK_STATE;
  }
}

bool SerialIO::write(const uint8_t *data, size_t size)
{
  if (serialPtr_->write(data, size) == size)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static unsigned char myatoi(char *array, int size)
{
  // negative or dekadiko???????????????
  int result = 0;
  for (int i = 0; i < size; i++)
  {
    if ((static_cast<int>(array[i]) >= 65) &&
      (static_cast<int>(array[i]) <= 70))
      result += (static_cast<int>(array[i]) - 55) * pow(16, size - i - 1);
    else if ((static_cast<int>(array[i]) >= 48) &&
      (static_cast<int>(array[i]) <= 57))
      result += (static_cast<int>(array[i]) - 48) * pow(16, size - i - 1);
  }

  return result;
}
}  // namespace xmega
}  // namespace pandora_hardware_interface
