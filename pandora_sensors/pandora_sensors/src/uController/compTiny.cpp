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

#include <uController/compTiny.h>

CompTiny::CompTiny()// : m_compassDgPublisher(m_sensorPublisher,updater,freqParam,tsParam)
{
  m_sensorPublisher = m_rosNode.advertise<controllers_and_sensors_communications::compassMsg> ("/sensors/compass", 1);
  dataLength = COMP_TINY_LEN;
}

int CompTiny::handleData()
{

  unsigned int temp; //temporary intiger that helps data conversion from unsigned  char to intiger
  int pitch, roll; //pitch and roll measurement of compass

  //Compass datasize should be 8 bytes.

  pitch = (signed char)data[1];

  roll = (signed char)data[2];

  ROS_DEBUG("[uController] Pitch: %d Roll: %d \n", pitch, roll);

  controllers_and_sensors_communications::compassMsg compassMsg;

  compassMsg.header.stamp = ros::Time::now();
  compassMsg.pitch = float(pitch) / 180.0 * 3.1415926535897932384626; //convert measurement to rads
  compassMsg.roll = float(roll) / 180.0 * 3.1415926535897932384626; //convert measurement to rads

  //m_compassDgPublisher.publish(compassMsg);

  m_sensorPublisher.publish(compassMsg);

  return IDLE_STATE;

}
