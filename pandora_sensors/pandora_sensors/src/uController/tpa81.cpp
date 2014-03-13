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

#include <uController/tpa81.h>

Tpa81::Tpa81() //: m_Tpa0DgPublisher(m_sensorPublisher,updater,freqParam,tsParam),
//m_Tpa1DgPublisher(m_sensorPublisher,updater,freqParam,tsParam),
//m_Tpa2DgPublisher(m_sensorPublisher,updater,freqParam,tsParam)
{

  m_sensorPublisher = m_rosNode.advertise<controllers_and_sensors_communications::tpaMsg> ("/sensors/tpa", 1);
  dataLength = TPA81_LEN;
}

int Tpa81::handleData()
{

  msg.header.stamp = ros::Time::now();

  switch(i2c_address)
  {
  case(TPA81_LEFT_ADDRESS):
    msg.id = msg.LEFT;
    break;
  case(TPA81_CENTER_ADDRESS):
    msg.id = msg.CENTER;
    break;
  case(TPA81_RIGHT_ADDRESS):
    ROS_DEBUG("right TPA\n");
    msg.id = msg.RIGHT;
    break;
  default:
    ROS_ERROR("WROND I2C ADDRESS!\n");
    break;
  }

  msg.ambientTemp = data[0];

  for (int i = 0; i < 8; i++)
    msg.pixelTemp[i] = data[i + 1];

  m_sensorPublisher.publish(msg);

  return IDLE_STATE;

}
