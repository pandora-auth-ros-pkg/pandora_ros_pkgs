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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

/**
http://www.asciiflow.com/#Draw5547639148542835085/1967816621
*
                                       +--------------+
 +-------------------+     +---------->|edge_detection|+----------->+
 |    depth_node     |     |           +--------------+             |     +-----------------------+
 +-------------------+     |           +-----------------+          |+--->|morphological_operators|
          +                +---------->|noise_elimination|+-------->|     +-----------------------+
          v                |           +-----------------+          |                 +
       +------+       +--->|           +--------------+             |                 v
       |depth |       |    +---------->|blob_detection|+----------->|          +-------------+
       +------+       |    |           +--------------+             |          |visualization|
          +           |    |           +----------------------+     |          +-------------+
          v           |    +---------->|bounding_box_detection|+--->|                 +
    +-------------+   |    |           +----------------------+     |                 v
    |hole_detector|   |    |           +----------------+           |            +----------------+
    +-------------+   |    +---------->|planes_detection|+--------->+            |depth_parameters|
          +           |                +----------------+                        +----------------+
          v           |                                                               +
     +------------+   |                                                               v
     |hole_filters+---+                                                           +-------+
     +------------+                                                               |defines|
                                                                                  +-------+
 **/

#include "depth_node/depth.h"

/**
  @brief Main function of the kinect node
  @param argc [int] Number of input arguments
  @param argv [char**] The input arguments
  @return int : 0 for success
 **/
int main(int argc, char** argv) {
  srand (time(NULL));
  ros::init(argc, argv, "DepthNode");
  pandora_vision::PandoraKinect pandora_kinect;
  ros::spin();
  return 0;
}

