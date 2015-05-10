/***************************************************************************
*   Copyright (C) 2010-2011 by Charalampos Serenis <info@devel.serenis.gr>*
*   Author: Charalampos Serenis <info@devel.serenis.gr>                   *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,  *
*   MA 02110-1301, USA.                                                   *
***************************************************************************/
#include "epos_handler/serial_epos_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{

SerialEposHandler::SerialEposHandler(const std::string& dev, const int& bauds, const int& time)
{
  gatewayImpl_.reset( new EposSerialGateway(dev, bauds, time) );
}

SerialEposHandler::~SerialEposHandler()
{
}


void SerialEposHandler::getRPM(int* leftRearRpm, int* leftFrontRpm,
  int* rightRearRpm, int* rightFrontRpm)
{
  epos::Word out[2];

  /* Read EPOS-P velocity value stored in 
   *   comm_EPOS register 0x206B
   *-------------------------------------------------*/
  gatewayImpl_->readObject(2, 0x206B, 0, &out[0]);
  if (out[1] > 10000)
  {
    /*--<Fix ST for values above 2^31>--*/
    *rightFrontRpm = (int32_t)(20000-out[1]);
  }
  else
  {
    // Minus because of motor reverse direction torsion
    *rightFrontRpm = -(int32_t)out[1];
  }
  /*-------------------------------------------------*/

  /*-----<Read Right-Rear Motor velocity>-----*/
  gatewayImpl_->readObject(2, 0x2028, 0, &out[0]);
  // Minus because of motor reverse direction torsion
  *rightRearRpm = -(int16_t)out[1];
  /*------------------------------------------*/

  /*-----<Read Left-Front Motor velocity>-----*/
  gatewayImpl_->readObject(3, 0x2028, 0, &out[0]);
  *leftFrontRpm = (int16_t)out[1];
  /*------------------------------------------*/

  /*-----<Read Left-Rear Motor velocity>------*/
  gatewayImpl_->readObject(4, 0x2028, 0, &out[0]);
  *leftRearRpm = (int16_t)out[1];
  /*------------------------------------------*/
}


void SerialEposHandler::getCurrent(int* axis0, int* axis1,
  int* axis2, int* axis3)
{
  /**
   * Axis0: Node 2, Right_Rear Wheel, EPOS communication controller.
   * Axis1: Node 1, Right_Front Wheel, EPOS-P controller.
   * Axis2: Node 3, Left_Rear Wheel, EPOS controller.
   * Axis3: Node 4, Left_Front Wheel, EPOS controller.
   */

  epos::Word out[2];

  /* Read EPOS-P (Right-Front) current value stored in 
   *   comm_EPOS register 0x2030.
   *---------------------------------------------------*/
  gatewayImpl_->readObject(2, 0x2030, 0, &out[0]);
  axis1[0] = (int16_t)out[1];
  /*---------------------------------------------------*/

  /*-----<Read Right-Rear Motor current>-----*/
  gatewayImpl_->readObject(2, 0x2027, 0, &out[0]);
  *axis0 = (int16_t)out[1];
  /*------------------------------------------*/

  /*-----<Read Left-Front Motor current>-----*/
  gatewayImpl_->readObject(3, 0x2027, 0, &out[0]);
  *axis3 = (int16_t)out[1];
  /*------------------------------------------*/

  /*-----<Read Left-Rear Motor current>------*/
  gatewayImpl_->readObject(4, 0x2027, 0, &out[0]);
  *axis2 = (int16_t)out[1];
  /*------------------------------------------*/
}

Error SerialEposHandler::getError()
{
  epos::Word out[2];
  Error error;
  gatewayImpl_->readObject(2, 0x1003, 1, &out[0]);
  error.leftRear = (int32_t)out[1];

  gatewayImpl_->readObject(2, 0x2081, 0, &out[0]);
  error.rightRear = (int32_t)out[1];

  return error;
}


uint16_t  SerialEposHandler::writeRPM(const int leftRpm, const int rightRpm)
{
  ROS_DEBUG("setting speed %d, %d", leftRpm, rightRpm);
  // Right motor rpm speed needs to be reversed because of its placement in the vehicle
  uint32_t controlWord = encodeToControlWord(leftRpm, -rightRpm);
  epos::CommandStatus error = gatewayImpl_->writeObject(2, 0x200C, 1, controlWord);

  if (error != epos::SUCCESS)
  {
    ROS_ERROR("error setting speed");
  }
  return error;
}

}  // namespace motor
}  // namespace pandora_hardware_interface


