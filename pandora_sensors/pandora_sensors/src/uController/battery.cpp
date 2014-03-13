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
*********************************************************************/

#include <uController/battery.h>

Battery::Battery() :
  //~ m_butterflyDgPublisher(m_sensorPublisher,updater,freqParamButterfly,tsParam),
  minFreqButterfly(0.3), maxFreqButterfly(0.4)
  //freqParamButterfly(&minFreqButterfly,&maxFreqButterfly,0.1,10)
{
  m_sensorPublisher = m_rosNode.advertise<controllers_and_sensors_communications::butterflyMsg> ("/sensors/butterfly", 1);

  measurementCounter = 0;
  dataLength = BATTERY_LEN;
  state = 0;
  status = 0;
  i2c_address = 0;
  motorSum = 0;
  psuSum = 0;
}

int Battery::handleData()
{

  float vMotor, vPSU; //explain

  vPSU = ((data[0] << 4) | ( data[1] >> 4));
  vPSU *= (2.704 / 4095) * 10;
  vPSU -= vPSU * 0.01;

  if(vPSU > 27 || vPSU < 0)
  {
    ROS_ERROR("Error in PSU voltage measurements!\n");
    vPSU = 0;
  }
  vMotor = (((data[1] & 0x0f) << 8) | data[2]) - 153;
  vMotor *= (2.704 / 4095) * 10;
  vMotor -= vMotor * 0.01;

  if(vMotor > 27 || vPSU < 0)
  {
    ROS_ERROR("Error in Motor voltage measurements!\n");
    vMotor = 0;
  }
  ROS_INFO("%f, %f", vPSU, vMotor);

  if(measurementCounter < 10)
  {

    mVoltage[measurementCounter] = vMotor;
    pVoltage[measurementCounter] = vPSU;
    measurementCounter++;
    return IDLE_STATE;

  }

  measurementCounter--;

  for(int i = 0; i < measurementCounter;  i++)
  {
    motorSum += mVoltage[i];
    psuSum += pVoltage[i];
  }

  butterflyMsg.voltage[0] = motorSum / measurementCounter;
  butterflyMsg.voltage[1] = psuSum / measurementCounter;

  m_sensorPublisher.publish(butterflyMsg);

  measurementCounter = 0;
  motorSum = 0;
  psuSum = 0;

  return IDLE_STATE;

}
