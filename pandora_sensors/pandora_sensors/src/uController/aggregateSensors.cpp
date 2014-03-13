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

#include <uController/aggregateSensors.h>
#include <math.h>

unsigned char myatoi(char *array);

unsigned char myatoi(char *array, int size)
{
  // negative or dekadiko???????????????
  int result = 0;
  for(int i = 0; i < size; i++)
  {
    if(((int)array[i] >= 65) && ((int)array[i] <= 70))
      result += ((int)array[i] - 55) * pow(16, size - i - 1);
    else if(((int)array[i] >= 48) && ((int)array[i] <= 57))
      result += ((int)array[i] - 48) * pow(16, size - i - 1);
    else
      result = NULL;
  }

  return result;
}

AggregateSensors::AggregateSensors(HardwareInterfacePtr hardwareInterfacePtr) : 
  hardwareInterfacePtr_(hardwareInterfacePtr) 
{
  currentState = IDLE_STATE;
  t1 = 0;
  t2 = 0;
  deviceID = 0;
  dataSize = 0;

}

int AggregateSensors::receiveData()
{

  timeval start, current;
  long ms_elapsed, seconds, useconds;
  int timer_flag = 0;

  int currentState = IDLE_STATE;
  int previousState = IDLE_STATE;
  int counter_ack = 0;
  int counter_nak = 0;
  int counter_timeout = 0;

  uint8_t ACK[] = {6};
  uint8_t NAK[] = {2, 1};

  while (ros::ok())
  {
    //updater.update();
    ros::spinOnce();

    ROS_DEBUG("[uController] current state: %d !", currentState);

    ROS_DEBUG("[uController] COUNTER_TIMEOUT: %d", counter_timeout);
    ROS_DEBUG("[uController] COUNTER_ACK: %d", counter_ack);
    ROS_DEBUG("[uController] COUNTER_NAK: %d", counter_nak);
    switch (currentState)
    {
    case IDLE_STATE:
      currentState = hardwareInterfacePtr_->readMessageType();
      if (timer_flag == 1)
      {
        gettimeofday(&current, NULL);
        seconds = current.tv_sec - start.tv_sec;
        useconds = current.tv_usec - start.tv_usec;
        ms_elapsed = ((seconds) * 1000 + useconds / 1000.0) + 0.5; 		// +0.5 is used for rounding positive values
        if(ms_elapsed >= 500)
        {
          currentState = previousState;
          timer_flag = 0;
          ms_elapsed = 0;
          counter_timeout ++;
        }
      }
      break;
    case READ_SIZE_STATE:
      gettimeofday(&tim, NULL);
      t1 = tim.tv_sec + (tim.tv_usec / 1000000.0);
      currentState = hardwareInterfacePtr_->readSize(&dataSize);
      break;
    case READ_DATA_STATE:
      pdataBuffer = new unsigned char[dataSize];
      memset(pdataBuffer, 0x00, dataSize);
      currentState = hardwareInterfacePtr_->readData( dataSize, pdataBuffer);
      break;
    case READ_CRC_STATE:
      currentState = hardwareInterfacePtr_->readCRC();
      break;
    case ACK_STATE:
      previousState = ACK_STATE;
      timer_flag = 1;
      gettimeofday(&start, NULL);
      if(!hardwareInterfacePtr_->write(ACK, 1))
        ROS_ERROR("[uController] Failed to write ACK!\n");
      counter_ack++;
      ROS_DEBUG("[uController] counter_ack: %d \n", counter_ack);
      currentState = PROCESS_DATA_STATE;
      break;
    case NAK_STATE:
      previousState = NAK_STATE;
      timer_flag = 1;
      gettimeofday(&start, NULL);
      if(!hardwareInterfacePtr_->write(NAK, 2))
        ROS_ERROR("[uController] Failed to write NAK!\n");
      currentState = IDLE_STATE;
      counter_nak++;
      break;
    case PROCESS_DATA_STATE:
      currentState = processData();
      gettimeofday(&tim, NULL);
      t2 = tim.tv_sec + (tim.tv_usec / 1000000.0);
      ROS_DEBUG("[uController] %.8f seconds elapsed\n", t2 - t1);
      break;
    default:
      currentState = IDLE_STATE;
      dataSize = 0;
      *pdataBuffer = NULL;
      break;

    }

  }
  return 0;
}

int AggregateSensors::processData()
{

  int bufferPointer = 0;
  int readState = SENSOR_ID;
  int type;
  char temp[2];
  int dataLength;

  while(bufferPointer < dataSize)
  {
    switch(readState)
    {
    case SENSOR_ID:
      ROS_DEBUG("Sensor ID: %c%c\n", pdataBuffer[bufferPointer], pdataBuffer[bufferPointer + 1]);
      bufferPointer += 3;	// ' ' after sensor id
      readState = SENSOR_TYPE;
      break;
    case SENSOR_TYPE:
      ROS_DEBUG("Sensor type: %c%c\n", pdataBuffer[bufferPointer], pdataBuffer[bufferPointer + 1]);
      temp[0] = pdataBuffer[bufferPointer];
      temp[1] = pdataBuffer[bufferPointer + 1];
      type = myatoi(temp, 2);
      deviceID = type;
      bufferPointer += 3;	// ' ' after sensor type
      readState = SENSOR_I2C_ADDRESS;
      if(type == 7)			// battery, not i2c sensor
        readState = SENSOR_DATA;
      break;
    case SENSOR_I2C_ADDRESS:
      ROS_DEBUG("Sensor I2C address: %c%c\n", pdataBuffer[bufferPointer], pdataBuffer[bufferPointer + 1]);
      temp[0] = pdataBuffer[bufferPointer];
      temp[1] = pdataBuffer[bufferPointer + 1];
      getSensor(type) -> i2c_address = myatoi(temp, 2);
      bufferPointer += 3;	// ' ' after sensor i2c address
      readState = SENSOR_STATUS;
      break;
    case SENSOR_STATUS:
      ROS_DEBUG("Sensor status: %c%c\n", pdataBuffer[bufferPointer], pdataBuffer[bufferPointer + 1]);
      temp[0] = pdataBuffer[bufferPointer];
      temp[1] = pdataBuffer[bufferPointer + 1];
      getSensor(type) -> status = myatoi(temp, 2);
      bufferPointer += 3;	// ' ' after sensor status
      readState = SENSOR_CURRENT_STATE;
      break;
    case SENSOR_CURRENT_STATE:
      ROS_DEBUG("Sensor current state: %c%c\n", pdataBuffer[bufferPointer], pdataBuffer[bufferPointer + 1]);
      temp[0] = pdataBuffer[bufferPointer];
      temp[1] = pdataBuffer[bufferPointer + 1];
      getSensor(type) -> state = myatoi(temp, 2);
      bufferPointer += 3;
      readState = SENSOR_DATA;
      break;
    case SENSOR_DATA:
      dataLength = getSensor(type) -> dataLength;

      ROS_DEBUG("Data: %d", dataLength);
      for(int i = 0; i < dataLength; i++)
      {
        temp[0] = pdataBuffer[bufferPointer];
        temp[1] = pdataBuffer[bufferPointer + 1];
        getSensor(type) -> data[i] = myatoi(temp, 2);
        ROS_DEBUG("%d ", getSensor(type) -> data[i]) ;
        bufferPointer += 2;
      }
      bufferPointer++;	// LF after Data
      ROS_DEBUG("POINTER: %d\n", bufferPointer);
      handleData();
      readState = SENSOR_ID;
      ROS_DEBUG("\n");
      break;
    default:
      return 0; 	// processing error
    }
  }
  return 1;	// successful processing

}

sensorHandler* AggregateSensors::getSensor(int sensorType)
{

  switch (sensorType)
  {

  case COMP_TINY:
    return &compTiny;
  case TPA81:
    return &tpa81;
  case BATTERY:
    return &bat;
  case SRF05_TINY:
    return &srf05Tiny;
  default:
    return &defaultSensor;
  }

}

int AggregateSensors::handleData()
{
  return getSensor(deviceID) -> handleData();
}
