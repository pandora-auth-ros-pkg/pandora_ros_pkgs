/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Author: Petros Evangelakos
*********************************************************************/

#ifndef LINEAR_MOTOR_COM_INTERFACE_FIRGELLI_DEFINITIONS_H
#define LINEAR_MOTOR_COM_INTERFACE_FIRGELLI_DEFINITIONS_H

#define  SET_ACCURACY 0x01
#define  SET_RETRACT_LIMIT 0x02
#define  SET_EXTEND_LIMIT 0x03
#define  SET_MOVEMENT_THRESHOLD 0x04
#define   SET_STALL_TIME 0x05
#define  SET_PWM_THRESHOLD 0x06
#define  SET_DERIVATIVE_THRESHOLD   0x07
#define   SET_DERIVATIVE_MAXIMUM     0x08
#define  SET_DERIVATIVE_MINIMUM   0x09
#define SET_PWM_MAXIMUM   0x0A
#define SET_PWM_MINIMUM  0x0B
#define  SET_PROPORTIONAL_GAIN  0x0C
#define  SET_DERIVATIVE_GAIN 0x0D
#define  SET_AVERAGE_RC  0x0E
#define  SET_AVERAGE_ADC   0x0F
#define GET_FEEDBACK  0x10
#define SET_POSITION  0x20
#define SET_SPEED  0x21
#define DISABLE_MANUAL   0x30
#define RESET 0xFF
#endif  // LINEAR_MOTOR_COM_INTERFACE_FIRGELLI_DEFINITIONS_H
