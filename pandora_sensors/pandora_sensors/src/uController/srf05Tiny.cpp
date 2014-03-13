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

#include <uController/srf05Tiny.h>

Srf05Tiny::Srf05Tiny() //: m_sonarDgPublisher(m_sensorPublisher,updater,freqParamSonar,tsParam),
//minFreqSonar(8), maxFreqSonar(11),
//freqParamSonar(&minFreqSonar,&maxFreqSonar,0.1,10),
//m_IrDgPublisher(m_sensorPublisher,updater,freqParamIr,tsParam),
//minFreqIr(1.6), maxFreqIr(2.5),
//freqParamIr(&minFreqIr,&maxFreqIr,0.2,10),
//minFreqHeadIr(1.6), maxFreqHeadIr(2.5),
//freqParamHeadIr(&minFreqHeadIr,&maxFreqHeadIr,0.2,10),
//m_HeadIrDgPublisher(m_HeadIrPublisher,updater,freqParamHeadIr,tsParam),
{
  m_sonarPublisher = m_rosNode.advertise<controllers_and_sensors_communications::sonarMsg> ("/sensors/sonar", 1);

  m_irPublisher = m_rosNode.advertise<controllers_and_sensors_communications::irMsg> ("/sensors/ir", 1);

  dataLength = SRF05_TINY_LEN;

}


int Srf05Tiny::handleData()
{

  controllers_and_sensors_communications::sonarMsg sonarMsg;
  sonarMsg.header.stamp = ros::Time::now();

  controllers_and_sensors_communications::irMsg irMsg;
  irMsg.header.stamp = ros::Time::now();

  switch(i2c_address)
  {
  case (REAR_RIGHT_ADDRESS):
    msgBuffer[4] = ((data[2] << 8) | data[3]);
    break;
  case (REAR_LEFT_ADDRESS):
    msgBuffer[3] = ((data[2] << 8) | data[3]);
    msgBuffer[13] = data[1];
    break;
  case (LEFT_ADDRESS):
    msgBuffer[1] = ((data[2] << 8) | data[3]);
    msgBuffer[12] = data[1];
    break;
  case (RIGHT_ADDRESS):
    msgBuffer[2] = ((data[2] << 8) | data[3]);
    msgBuffer[11] = data[1];
    break;
  case (FRONT_ADDRESS):
    msgBuffer[0] = ((data[2] << 8) | data[3]);
    msgBuffer[10] = data[1];
    break;
  default:
    ROS_ERROR("WRONG I2C ADDRESS!\n");
    break;
  }

  sonarMsg.distance[0] = msgBuffer[0];
  sonarMsg.distance[1] = msgBuffer[1];
  sonarMsg.distance[2] = msgBuffer[2];
  sonarMsg.distance[3] = msgBuffer[3];
  sonarMsg.distance[4] = msgBuffer[4];

  irMsg.distance[0] = msgBuffer[10];
  irMsg.distance[1] = msgBuffer[11];
  irMsg.distance[2] = msgBuffer[12];
  irMsg.distance[3] = msgBuffer[13];

  m_sonarPublisher.publish(sonarMsg);
  m_irPublisher.publish(irMsg);

  return IDLE_STATE;
}

int Srf05Tiny::calculateIrV2D120x(float voltage)
{

  int result;

  if(voltage < 2.10)
    result = 0; //too close
  else if(voltage > 2.10)
    result = 5;
  else if( voltage < 2.10 && voltage > 1.98 )
    result = 6;
  else if( voltage < 1.98 && voltage > 1.76)
    result = 7;
  else if( voltage < 1.76 && voltage > 1.6 )
    result = 8;
  else if( voltage < 1.6 && voltage > 1.4)
    result = 9;
  else if( voltage < 1.4 && voltage > 1.2)
    result = 10;
  else if( voltage < 1.2 && voltage > 1.1)
    result = 11;
  else if( voltage < 1.1 && voltage > 1.0)
    result = 12;
  else if( voltage < 1.0 && voltage > 0.9)
    result = 13;
  else if( voltage < 0.9 && voltage > 0.8)
    result = 14;
  else if( voltage < 0.8 && voltage > 0.75)
    result = 15;
  else if( voltage < 0.75 && voltage > 0.7)
    result = 16;
  else if( voltage < 0.7 && voltage > 0.65)
    result = 17;
  else if( voltage < 0.65 && voltage > 0.6)
    result = 18;
  else if( voltage < 0.6 && voltage > 0.55)
    result = 19;
  else if( voltage < 0.55 && voltage > 0.5)
    result = 20;
  else if( voltage < 0.5 && voltage > 0.48)
    result = 21;
  else if( voltage < 0.48 && voltage > 0.46)
    result = 22;
  else if( voltage < 0.46 && voltage > 0.44)
    result = 23;
  else if( voltage < 0.44 && voltage > 0.42)
    result = 24;
  else if( voltage < 0.42 && voltage > 0.39)
    result = 25;
  else if( voltage < 0.39 && voltage > 0.36)
    result = 26;
  else if( voltage < 0.36 && voltage > 0.34)
    result = 27;
  else if( voltage < 0.34 && voltage > 0.32)
    result = 28;
  else if( voltage < 0.32 && voltage > 0.3)
    result = 29;
  else
    result = -1;	// too far

  return result;

}

int Srf05Tiny::calculateIr2YDA21(float voltage)
{

  int result;

  if( voltage > 3 )
    result = 8; // can't measure distances closer than 8 centimetres
  else if( voltage > 2.69 && voltage < 3 )
    result = 9;
  else if( voltage > 2.38 && voltage < 2.69 )
    result = 10;
  else if( voltage > 2.25 && voltage < 2.38 )
    result = 11;
  else if( voltage > 2.12 && voltage < 2.25 )
    result = 12;
  else if( voltage > 1.91 && voltage < 2.12 )
    result = 13;
  else if( voltage > 1.7 && voltage < 1.91 )
    result = 14;
  else if( voltage > 1.6 && voltage < 1.7 )
    result = 15;
  else if( voltage > 1.5 && voltage < 1.6 )
    result = 16;
  else if( voltage > 1.39 && voltage < 1.5 )
    result = 17;
  else if( voltage > 1.28 && voltage < 1.39 )
    result = 18;
  else if( voltage > 1.21 && voltage < 1.28 )
    result = 19;
  else if( voltage > 1.14 && voltage < 1.21 )
    result = 20;
  else if( voltage > 1.09 && voltage < 1.14 )
    result = 21;
  else if( voltage > 1.04 && voltage < 1.09 )
    result = 22;
  else if( voltage > 0.97 && voltage < 1.04 )
    result = 23;
  else if( voltage > 0.9 && voltage < 0.97 )
    result = 24;
  else if( voltage > 0.86 && voltage < 0.9 )
    result = 25;
  else if( voltage > 0.82 && voltage < 0.86 )
    result = 26;
  else if( voltage > 0.785 && voltage < 0.82 )
    result = 27;
  else if( voltage > 0.75 && voltage < 0.785 )
    result = 28;
  else if( voltage > 0.725 && voltage < 0.75 )
    result = 29;
  else if( voltage > 0.7 && voltage < 0.725 )
    result = 30;
  else if( voltage > 0.3 && voltage < 0.7)
    result = 60;
  else
    result = -1;

  return result;


}
