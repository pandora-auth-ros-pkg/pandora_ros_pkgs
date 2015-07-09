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
* Author:     Konstantinos Panayiotou   <klpanagi@gmail.com>
* Maintainer: Konstantinos Panayiotou   <klpanagi@gmail.com>
*
*********************************************************************/

#include <map>
#include <cstring>
#include <stdint.h>

namespace pandora_hardware_interface
{
  namespace motor
  {
    enum ErrorCodes
    {
      NO_COMM_ERR = 0x00000000,
      TOGGLE_ERR = 0x05030000,
      SDO_TIME_OUT = 0x05040000,
      CLIENT_SERVER_ERR = 0x05040001,
      CRC_ERR = 0x05040004,
      OUT_OF_MEMORY = 0x05040005,
      ACCESS_ERR = 0x06010000,
      WRITE_ONLY_ERR = 0x06010001,
      RELY_ERR = 0x06010002,
      OBJ_NOT_EXIST = 0x06020000,
      PDO_MAPPING_ERR = 0x06040041,
      PDO_LENGTH_ERR = 0x06040042,
      GEN_PARAM_ERR = 0x06040043,
      GEN_INTERNAL_ERR = 0x06040047,
      HARDWARE_ERR = 0x06060000,
      SRVC_PARAM_ERR = 0x06070010,
      SRVC_PARAM_LONG = 0x06070012,
      SRVC_PARAM_SHORT = 0x06070013,
      OBJ_SUBINDEX_ERR = 0x06090011,
      VALUE_TOO_HIGH = 0x06090030,
      VALUE_TOO_LOW = 0x06090032,
      MAX_LESS_MIN = 0x06090036,
      GENERAL_ERR = 0x08000000,
      TRANSF_STORE_ERR = 0x08000020,
      LOCAL_CONTROL_ERR = 0x08000021,
      WRONG_DEVICE_STATE = 0x80000022,
      WRONG_SMT_STATE = 0x0f00ffc0,
      ILLEGAL_COMMAND = 0x0f00ffbf,
      PASSWORD_ERR = 0x0f00ffbe,
      SERVICE_MODE_ERR = 0x00f00ffbc,
      CAN_ID_ERR = 0x0f00ffb9
    };
  }
}
