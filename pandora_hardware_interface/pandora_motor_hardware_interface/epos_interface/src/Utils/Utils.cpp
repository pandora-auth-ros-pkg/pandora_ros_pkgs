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

#include <stdint.h>
#include <map>
#include <cstring>
#include <iostream>  // 
#include <fstream>  // file io streams
#include <sstream>  // string streams
#include "Utils/Utils.h"

namespace pandora_hardware_interface
{
  namespace motor
  {

    std::map<int, std::string> Utils::readErrorCodesMap(
      const std::string error_codes_file)
    {
      std::map<int, std::string> error_codes_map;
      std::map<int, std::string>::iterator it = error_codes_map.begin();
      std::ifstream file;
      std::string token;
      std::string line;
      int error_code;
      std::string error_str;
      std::string error_code_str;

      file.open(error_codes_file.c_str());
      if(file.is_open())
      {
        while(std::getline(file, line))
        {
          std::istringstream iss(line);
          getline(iss, error_code_str, ',');

          std::istringstream hex_str(error_code_str);
          hex_str >> std::hex >> error_code;

          getline(iss, error_str, ',');
          if(error_str.at(0) == ' ')
          {
            error_str.erase(0, 1);
          }
          error_codes_map.insert(it, std::pair<int, std::string>(
              error_code, error_str));
          std::cout << "ErrorCode_int: " << error_code << ", ErrorCode_str: " 
            << error_code_str << ", Error: " << error_codes_map[error_code] << "\n";
        }
      }
      return error_codes_map;

    }
  }
}
