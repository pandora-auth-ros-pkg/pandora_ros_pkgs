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

#ifndef SRF05TINY_H
#define SRF05TINY_H

#include "sensorHandler.h"
#include "controllers_and_sensors_communications/sonarMsg.h"
#include "controllers_and_sensors_communications/irMsg.h"

#define REAR_RIGHT_ADDRESS	0x82//to be added
#define REAR_LEFT_ADDRESS	0x80//to be added
#define LEFT_ADDRESS		0x86//to be added
#define RIGHT_ADDRESS		0x88//to be added
#define FRONT_ADDRESS		0x84//to be added

class Srf05Tiny : public sensorHandler
{

  //double minFreqSonar, maxFreqSonar;
  //diagnostic_updater::FrequencyStatusParam freqParamSonar;

  //diagnostic_updater::DiagnosedPublisher<controllers_and_sensors_communications::sonarMsg> m_sonarDgPublisher;
  ros::Publisher	m_sonarPublisher;
  ros::Publisher	m_irPublisher;
  int msgBuffer[20];
  int measurementCounter;

  int calculateIrV2D120x(float voltage);
  int calculateIr2YDA21(float voltage);

public:

  Srf05Tiny();
  int handleData();

};

#endif
