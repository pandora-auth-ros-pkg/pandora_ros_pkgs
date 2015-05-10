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
#include "epos_handler/abstract_epos_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{

Error::Error()
{
}
Error::~Error()
{
}

AbstractEposHandler::AbstractEposHandler() : gatewayImpl_(NULL)
{
}

uint32_t AbstractEposHandler::encodeToControlWord(
  const int& left, const int& right)
{
  int signLeft = left < 0 ? 1 : 0;
  int signRight = right < 0 ? 1 : 0;

  int leftSpeedAbsolute = std::abs(left);
  int rightSpeedAbsolute = std::abs(right);

  uint32_t controlWord = 0;

  controlWord = (1 << 31) | (signRight << 30) | (rightSpeedAbsolute << 16) |
    (signLeft << 14) | (leftSpeedAbsolute);
  return controlWord;
}

AbstractEposHandler::~AbstractEposHandler()
{
}

}  // namespace motor
}  // namespace pandora_hardware_interface
