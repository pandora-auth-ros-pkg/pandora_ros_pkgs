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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/
#ifndef HOLE_FUSION_NODE_HOLE_MERGER_H
#define HOLE_FUSION_NODE_HOLE_MERGER_H

#include "hole_fusion_node/filters_resources.h"
#include "hole_fusion_node/depth_filters.h"
#include "utils/blob_detection.h"
#include "utils/defines.h"
#include "utils/holes_conveyor.h"
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
        @brief Applies a merging operation of @param operationId, until
        every candidate hole, even as it changes through the various merges that
        happen, has been merged with every candidate hole that can be merged
        with it.
        @param[in,out] rgbdHolesConveyor [HolesConveyor*] The unified rgb-d
        candidate holes conveyor
        @param[in] image [const cv::Mat&] An image used for filters' resources
        creation and size usage
        @param[in] pointCloud [const PointCloudPtr] An interpolated point
        cloud used in the connection operation; it is used to obtain real world
        distances between holes
        @param[in] operationId [const int&] The identifier of the merging
        process. Values: 0 for assimilation, 1 for amalgamation and
        2 for connecting
        @return void
       **/
      static void applyMergeOperation(
        HolesConveyor* rgbdHolesConveyor,
        const cv::Mat& image,
        const PointCloudPtr& pointCloud,
        const int& operationId);

      /**
        @brief Applies a merging operation of @param operationId, until
        every candidate hole, even as it changes through the various merges that
        happen, has been merged with every candidate hole that can be merged
        with it. In contrast to the applyMergeOperation method, this method
        does not consult hole checkers to validate the newly merged holes.
        (Used when depth analysis is unattainable)
        @param[in,out] rgbdHolesConveyor [HolesConveyor*] The unified rgb-d
        candidate holes conveyor
        @param[in] image [const cv::Mat&] An image used for filters' resources
        creation and size usage
        @param[in] pointCloud [const PointCloudPtr] An interpolated point
        cloud used in the connection operation; it is used to obtain real world
        distances between holes
        @param[in] operationId [const int&] The identifier of the merging
        process. Values: 0 for assimilation, 1 for amalgamation and
        2 for connecting
        @return void
       **/
      static void applyMergeOperationWithoutValidation(
        HolesConveyor* rgbdHolesConveyor,
        const cv::Mat& image,
        const PointCloudPtr& pointCloud,
        const int& operationId);

      /**
        @brief Indicates whether a hole assigned the role of the assimilator
        is capable of assimilating another hole assigned the role of
        the assimilable. It checks whether the assimilable's outline
        points reside entirely inside the assimilator's outline.
        @param[in] assimilatorHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the assimilator's
        outline
        @param[in] assimilableHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the assimilable's
        outline
        @return [bool] True if all of the outline points of the assimilable
        hole are inside the outline of the assimilator
       **/
      static bool isCapableOfAssimilating(
        const std::set<unsigned int>& assimilatorHoleMaskSet,
        const std::set<unsigned int>& assimilableHoleMaskSet);

      /**
        @brief Indicates whether a hole assigned the role of the amalgamator
        is capable of amalgamating another hole assigned the role of
        the amalgamatable. The amalgamator is capable of amalgamating the
        amalgamatable if and only if the amalgatamable's outline
        points intersect with the amalgamator's outline, but not in their
        entirety, and the area of the amalgamator is larger than the area
        of the amalgamatable
        @param[in] amalgamatorHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the amalgamator's
        outline
        @param[in] amalgamatableHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the amalgamatable's
        outline
        @return [bool] True if the amalgamator is capable of amalgamating
        the amalgamatable
       **/
      static bool isCapableOfAmalgamating(
        const std::set<unsigned int>& amalgamatorHoleMaskSet,
        const std::set<unsigned int>& amalgamatableHoleMaskSet);

      /**
        @brief Intended to use after the check of the
        isCapableOfAmalgamating function, this function modifies the
        HolesConveyor conveyor[amalgamatorId] entry so that it
        has absorbed the amalgamatable hole (conveyor[amalgamatableId])
        in terms of keypoint location, outline unification and bounding
        rectangle inclusion of the amalgamatable's outline
        @param[in,out] conveyor [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be modified
        @param[in] amalgamatorId [const int&] The identifier of the
        hole inside the HolesConveyor amalgamator struct
        @param[in,out] amalgamatorHoleMaskSet [std::set<unsigned int>*]
        A set that includes the indices of points inside the amalgamator's
        outline
        @param[in] amalgamatableHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the amalgamatable's
        outline
        @param[in] image [const cv::Mat&] An image used for its size
        @return void
       **/
      static void amalgamateOnce(
        HolesConveyor* conveyor,
        const int& amalgamatorId,
        std::set<unsigned int>* amalgamatorHoleMaskSet,
        const std::set<unsigned int>& amalgamatableHoleMaskSet,
        const cv::Mat& image);

      /**
        @brief Indicates whether a hole assigned the role of the connectable
        is capable of being connected with another hole assigned the role of
        the connector. The connectable is capable of being connected with the
        connectable if and only if the connectable's outline
        points do not intersect with the connector's outline, the bounding
        rectangle of the connector is larger in area than the bounding
        rectangle of the connectable, and the minimum distance between the
        two outlines is lower than a threshold value.
        @param[in] conveyor [const HolesConveyor&] The HolesConveyor that
        contains the holes
        @param[in] connectorId [const int&] The index of the specific hole
        that acts as the connector inside the connectors HolesConveyor
        @param[in] connectableId [const int&] The index of the specific hole
        that acts as the connectable inside the connectables HolesConveyor
        @param[in] connectorHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the connector's
        outline
        @param[in] connectableHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the connectable's
        outline
        @param[in] pointCloud [const PointCloudPtr&] The point cloud
        obtained from the depth sensor, used to measure distances in real
        space
        @return [bool] True if the connectable is capable of being connected
        with the connector
       **/
      static bool isCapableOfConnecting(
        const HolesConveyor& conveyor,
        const int& connectorId,
        const int& connectableId,
        const std::set<unsigned int>& connectorHoleMaskSet,
        const std::set<unsigned int>& connectableHoleMaskSet,
        const PointCloudPtr& pointCloud);

      /**
        @brief Intended to use after the check of the
        isCapableOfConnecting function, this method modifies the HolesConveyor
        connector struct entry so that it it has absorbed the connectable hole
        in terms of keypoint location, outline unification and bounding
        rectangle inclusion of the connectable's outline
        @param[in,out] conveyor [HolesConveyor*] The holes conveyor
        whose keypoint, outline and bounding rectangle entries
        will be modified
        @param[in] connectorId [const int&] The identifier of the hole inside
        the HolesConveyor connectables struct
        @param[in] connectableId [const int&] The identifier of the hole inside
        the HolesConveyor connectables struct
        @param[in] connectorHoleMaskSet [const std::set<unsigned int>&]
        A set that includes the indices of points inside the connector's
        outline
        @param[in] image [const cv::Mat&] An image required only for its size
        @return void
       **/
      static void connectOnce(
        HolesConveyor* conveyor,
        const int& connectorId,
        const int& connectableId,
        std::set<unsigned int>* connectorHoleMaskSet,
        const cv::Mat& image);

  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_HOLE_MERGER_H
