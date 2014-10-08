/** @file battery_sensor.cpp
 *  @brief Battery sensor .cpp file
 *
 *  This contains battery class and methods implementations used for batteries 
 *  voltage level measurements.
 *
 *  @author Michael Niarchos
 *  @author Chris Zalidis
 *  @author Konstantinos Panayiotou
 *  @bug No known bug
 */

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

#include <xmega_serial_interface/battery_sensor.h>

namespace pandora_hardware_interface
{
namespace xmega
{

BatterySensor::BatterySensor()
{
}

void BatterySensor::handleData()
{
  psuVoltage = ((data[0] << 4) | (data[1] >> 4));
  psuVoltage *= (AREF / 4096) * VoltageDividerScale;
  psuVoltage -= psuVoltage * 0.04;        /* <calculated scale factor error for Electronics line> */

  motorVoltage = (((data[1] & 0x0f) << 8) | data[2]);
  motorVoltage *= (AREF / 4096) * VoltageDividerScale;
  motorVoltage -= motorVoltage * 0.06;
}

BatterySensor::~BatterySensor()
{
}

}  // namespace xmega
}  // namespace pandora_hardware_interface
