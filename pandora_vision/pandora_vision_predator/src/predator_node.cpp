/*  Copyright (c) 2014, Victor Daropoulos
 *  All rights reserved.
 *  
 *  This file is part of Pandora_OpenTLD.

 *  Pandora_OpenTLD is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Pandora_OpenTLD is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Pandora_OpenTLD. If not, see http://www.gnu.org/licenses/.
 */

#include "ros/ros.h"
#include <ros/package.h>
#include "pandora_vision_predator/predator.h"

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "predator_node");
  pandora_vision::Predator predator("predator");  
  ros::spin();
  return 0;
}

