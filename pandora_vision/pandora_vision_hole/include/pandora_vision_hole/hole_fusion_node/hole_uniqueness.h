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
 * Author: Alexandros Philotheou
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_UNIQUENESS_H
#define PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_UNIQUENESS_H

#include "utils/defines.h"
#include "utils/holes_conveyor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleUniqueness
    @brief Provides method for making holes inside a
    HolesConveyor container unique
   **/
  class HoleUniqueness
  {
    public:
      /**
        @brief Given a HolesConveyor container, this method detects
        multiple unique holes and deletes the extra copies.
        @param[in,out] conveyor [HolesConveyor*] The container of holes
        @return void
       **/
      static void makeHolesUnique(HolesConveyor* conveyor);

      /**
        @brief Takes as input a container of holes and a map of indices
        of holes referring to the container to validity probabilities.
        It outputs the most probable unique valid holes and a map
        adjusted to fit the altered container of holes.

        Inside the conveyor container, reside holes that have originated
        from the Depth and RGB nodes, plus any merges between them. Having
        acquired the validity probability of each one of them, this method
        locates valid holes that refer to the same physical hole in space
        inside the conveyor container and picks the one with the largest
        validity probability.
        @param[in,out] conveyor [HolesConveyor*] The conveyor of holes.
        @param[in,out] validHolesMap [std::map<int, float>*] The std::map
        that maps holes inside the conveyor conveyor to their validity
        probability
        @return void
       **/
      static void makeHolesUnique(HolesConveyor* conveyor,
        std::map<int, float>* validHolesMap);
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_UNIQUENESS_H
