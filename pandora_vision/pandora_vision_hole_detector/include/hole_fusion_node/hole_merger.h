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
#ifndef HOLE_FUSION_NODE_HOLE_MERGER_H
#define HOLE_FUSION_NODE_HOLE_MERGER_H

#include "utils/holes_conveyor.h"
#include "utils/defines.h"
#include "utils/parameters.h"
#include <math.h>

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleMerger
    @brief Provides functionalities of merging holes independently of depth
    or RGB images, or analysis
   **/
  class HoleMerger
  {
    public:

      /**
        @brief Assimilates the fragmented holes of @param assimilable into the
        existing whole ones of @param assimilator. It checks whether each
        entry of the set of assimilable's outline points reside inside
        the assimilator's set of outlines. If so, the latter keypoint
        etc are kept unchanged and the former one is deleted.
        @param[in][out] assimilator [const HolesConveyor&]
        The candidate holes conveyor that will potentially assimilate the
        assimimable's holes
        @param[in][out] assimilable [HolesConveyor*]
        The candidate holes conveyor whose holes will potentially by assimilated
        by the assimilator
        @return void
       **/
      static void assimilateUnilaterally(
        const HolesConveyor& assimilator,
        HolesConveyor* assimilable);

      /**
        @brief Indicates whether a hole assigned the role of the assimilator
        is capable of assimilating another hole assigned the role of
        the assimilable. It checks whether the assimilable's outline
        points reside entirely inside the assimilator's outline.
        @param[in] assimilatorId [const int&] The index of the specific hole
        inside the assimilators HolesConveyor
        @param[in] assimilator [const HolesConveyor&] The HolesConveyor struct
        that acts as the assimilator
        @param[in] assimilableId [const int&] The index of the specific hole
        inside the assimilables HolesConveyor
        @param[in] assimilable [const HolesConveyor&] The HolesConveyor struct
        that acts as the assimilable
        @return [bool] True if all of the outline points of the assimilable
        hole are inside the outline of the assimilator
       **/
      static bool isCapableOfAssimilating(
        const int& assimilatorId,
        const HolesConveyor& assimilator,
        const int& assimilableId,
        const HolesConveyor& assimilable);

      /**
        @brief Intended to use after the check of the
        isCapableOfAssimilating function, this function carries the burden
        of having to delete a hole entry from its HolesConveyor struct,
        thus being its executor
        @param[in] keyPointId [const int&] The identifier of the hole inside
        the HolesConveyor struct
        @param[in][out] assimilable [HolesConveyor*] The holes conveyor
        from which the keypoint, outline and bounding rectangle entries
        will be deleted
        @return void
       **/
      static void assimilateOnce(const int& keyPointId,
        HolesConveyor* assimilable);
      /**
        @brief Assimilates fragmented holes into existing whole ones
        from either source (RGB or Depth).
        @param[in][out] depthHolesConveyor [HolesConveyor*]
        The candidate holes conveyor originated from the depth node
        @param[in][out] rgbHolesConveyor [HolesConveyor*]
        The candidate holes conveyor originated from the rgb node
        @return void
       **/
      static void assimilateBilaterally(
        HolesConveyor* depthHolesConveyor,
        HolesConveyor* rgbHolesConveyor);

      /**
        @brief Connects nearby holes with a proximity criterion.
        Holes' outlines do not intersect.
        @param[in][out] connector [HolesConveyor*] The holes conveyor
        that will act as the connector of holes
        @param[in][out] connectable [HolesConveyor*] The holes conveyor
        that will act as the connectable
        @param [in] pointCloud [const PointCloudXYZPtr&] The point cloud
        needed in order to specify which holes are going to be connected
        by criterion of distance
        @return void
       **/
      static void connectUnilaterally(HolesConveyor* assimilator,
        HolesConveyor* assimilable, const PointCloudXYZPtr& pointCloud);

      /**
        @brief Indicates whether a hole assigned the role of the connectable
        is capable of being connected with another hole assigned the role of
        the connector. The connectable is capable of being connected with the
        connectable if and only if the connectable's outline
        points do not intersect with the connector's outline, the bounding
        rectangle of the connector is larger in area than the bounding
        rectangle of the connectable, and the minimum distance between the
        two outlines is lower than a threshold value.
        @param[in] connectorId [const int&] The index of the specific hole
        that acts as the connector inside the connectors HolesConveyor
        @param[in] connector [const HolesConveyor&] The HolesConveyor that
        acts as the connector struct
        @param[in] connectableId [const int&] The index of the specific hole
        that acts as the connectable inside the connectables HolesConveyor
        @param[in] connectable [const HolesConveyor&] The HolesConveyor that
        acts as the connectable struct
        @param[in] pointCloudXYZ [const PointCloudXYZPtr&] The point cloud
        obtained from the depth sensor, used to measure distances in real
        space
        @return [bool] True if the connectable is capable of being connected
        with the connector
       **/
      static bool isCapableOfConnecting(const int& connectorId,
        const HolesConveyor& connector,
        const int& connectatableId,
        const HolesConveyor& connectable,
        const PointCloudXYZPtr& pointCloudXYZ);

      /**
        @brief Intended to use after the check of the
        isCapableOfConnecting function, this function carries the burden
        of having to delete a hole entry from its HolesConveyor connectable
        struct, thus being its executor,
        and modifying the HolesConveyor connector struct entry so that it
        it has absorbed the connectable hole in terms of keypoint location,
        outline unification and bounding rectangle inclusion of the
        connectable's outline
        @param[in] connectorId [const int&] The identifier of the hole inside
        the HolesConveyor connectables struct
        @param[in][out] connector [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be modified
        @param[in] connectableId [const int&] The identifier of the hole inside
        the HolesConveyor connectables struct
        @param[in][out] connectable [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be deleted
        @return void
       **/
      static void connectOnce(const int& connectorId,
        HolesConveyor* connector,
        const int& connectableId,
        HolesConveyor* connectable);

      /**
        @brief Given the RGB and Depth HolesConveyor* structs,
        the purpose of this function is to identify blobs that are overlapping
        each other but none of them is entirely inside the other, and merge
        them in one candidate hole: the one whose bounding rectangle has
        the greater area.
        @param[in][out] depthHolesConveyor [HolesConveyor*]
        The candidate holes conveyor originated from the depth node
        @param[in][out] rgbHolesConveyor [HolesConveyor*]
        The candidate holes conveyor originated from the rgb node
        @return void
       **/
      static void amalgamateBilaterally(
        HolesConveyor* depthHolesConveyor,
        HolesConveyor* rgbHolesConveyor);

      /**
        @brief Given two HolesConveyor* structs, one with the
        potential of amalgamating the other (amalgamator) and the other with the
        potential of being amalgamated by the other (amalgamatable), the purpose
        of this function is to identify blobs that are overlapping each other
        but none of them is entirely inside the other, while the amalgamator's
        bounding box is greater than that of the amalgamatable's.
        If this is true for a given amalgamator and amalgamatable,
        the amalgamator will grow rectangle-wise and outline-wise
        by the size of the amalgamator, while the amalgamator's
        new keypoint will be the mean of the two keypoints. The amalgamatable
        then is rendered useless and deleted.
        @param[in][out] amalgamator [HolesConveyor*]
        The holes conveyor whose candidate holes will potentially
        assimilate candidate holes of @param amalgamatable
        @param[in][out] amalgamatable [HolesConveyor*]
        The holes conveyor whose candidate holes will potentially
        will be assimilated by candidate holes of @paramamalgamator
        @return void
       **/
      static void amalgamateUnilaterally(
        HolesConveyor* amalgamator,
        HolesConveyor* amalgamatable);

      /**
        @brief Indicates whether a hole assigned the role of the amalgamator
        is capable of amalgamating another hole assigned the role of
        the amalgamatable. The amalgamator is capable of amalgamating the
        amalgamatable if and only if the amalgatamable's outline
        points intersect with the amalgamator's outline, and the bounding
        rectangle of the amalgamator is larger in area than the bounding
        rectangle of the amalgamatable
        @param[in] amalgamatorId [const int&] The index of the specific hole
        that acts as the amalgamator inside the amalgamators HolesConveyor
        @param[in] amalgamator [const HolesConveyor&] The HolesConveyor that
        acts as the amalgamator struct
        @param[in] amalgamatableId [const int&] The index of the specific hole
        that acts as the amalgamatable inside the amalgamatables HolesConveyor
        @param[in] amalgamatable [const HolesConveyor&] The HolesConveyor that
        acts as the amalgamatable struct
        @return [bool] True if the amalgamator is capable of amalgamating
        the amalgamatable
       **/
      static bool isCapableOfAmalgamating(const int& amalgamatorId,
        const HolesConveyor& amalgamator,
        const int& amalgamatableId,
        const HolesConveyor& amalgamatable);

      /**
        @brief Intended to use after the check of the
        isCapableOfAmalgamating function, this function carries the burden
        of having to delete a hole entry from its HolesConveyor amalgamatable
        struct, thus being its executor,
        and modifying the HolesConveyor amalgamator struct entry so that it
        it has absorbed the amalgamatable hole in terms of keypoint location,
        outline unification and bounding rectangle inclusion of the
        amalgamatable's outline
        @param[in] amalgamatorId [const int&] The identifier of the hole inside
        the HolesConveyor amalgamator struct
        @param[in][out] amalgamator [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be modified
        @param[in] amalgamatableId [const int&] The identifier of the hole
        inside the HolesConveyor amalgamatable struct
        @param[in][out] amalgamatable [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be deleted
        @return void
       **/
      static void amalgamateOnce(const int& amalgamatorId,
        HolesConveyor* amalgamator,
        const int& amalgamatableId,
        HolesConveyor* amalgamatable);
  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_GENERIC_FILTERS_H
