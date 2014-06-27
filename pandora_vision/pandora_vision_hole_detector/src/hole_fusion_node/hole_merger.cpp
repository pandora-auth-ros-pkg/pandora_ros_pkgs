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

#include "hole_fusion_node/hole_merger.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
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
  void HoleMerger::applyMergeOperation(
    HolesConveyor* rgbdHolesConveyor,
    const cv::Mat& image,
    const PointCloudPtr& pointCloud,
    const int& operationId)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyMergeOperation", "mergeHoles");
    #endif

    // If there are no candidate holes,
    // or there is only one candidate hole,
    // there is no meaning to this operation
    if (rgbdHolesConveyor->size() < 2)
    {
      return;
    }

    // A vector that indicates when a specific hole has finished
    // examining all the other holes in the conveyor for merging.
    // Initialized at 0 for all conveyor entries, the specific hole
    // that corresponds to a vector's entry is given a 1 when it has
    // finished examining all other holes.
    std::vector<int> finishVector(rgbdHolesConveyor->size(), 0);

    // The index of the candidate hole that will
    // {assimilate, amalgamate, connect} the passiveId-th candidate hole.
    // The activeId always has a value of 0 due to the implementation's
    // rationale: The candidate hole that examines each hole in the
    // rgbdHolesConveyor is always the first one. When it has finished,
    // it goes back into the rgbdHolesConveyor, at the last position
    const int activeId = 0;

    // The index of the candidate hole that will be
    // {assimilated, amalgamated, connected} by / with
    // the activeId-th candidate hole
    int passiveId = 1;

    // Is the activeId-th candidate hole able to
    // {assimilate, amalgamate, connect to} the passiveId-th candidate hole?
    bool isAble = true;

    // The holesMaskSetVector vector is used in the merging processes
    std::vector<std::set<unsigned int> > holesMasksSetVector;

    // Indicates the end of the merging process
    bool isFuseComplete = false;

    while(!isFuseComplete)
    {
      // If a(n) {assimilation, amalgamation, connection} did happen,
      // regenerate the mask sets in order for them to reflect the new
      // state of holes
      if (isAble)
      {
        FiltersResources::createHolesMasksSetVector(
          *rgbdHolesConveyor,
          image,
          &holesMasksSetVector);
      }


      if (operationId == ASSIMILATION)
      {
        // Is the activeId-th candidate hole able to assimilate the
        // passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAssimilating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }
      if (operationId == AMALGAMATION)
      {
        // Is the activeId-th candidate hole able to amalgamate the
        // passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAmalgamating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }
      if (operationId == CONNECTION)
      {
        // Is the passiveId-th candidate hole able to be connected with the
        // activeId-th candidate hole?
        isAble = HoleMerger::isCapableOfConnecting(*rgbdHolesConveyor,
          activeId,
          passiveId,
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId],
          pointCloud);
      }


      if (isAble)
      {
        // Copy the original holes conveyor to a temp one.
        // The temp one will be tested through the hole filters
        // On success, temp will replace rgbdHolesConveyor,
        // on failure, rgbdHolesConveyor will remain unchanged
        HolesConveyor tempHolesConveyor;
        HolesConveyorUtils::copyTo(*rgbdHolesConveyor, &tempHolesConveyor);

        // Copy the original holes masks set to a temp one.
        // If the temp conveyor is tested successfully through the hole
        // filters, temp will replace the original.
        // On failure, the original will remain unchanged
        std::vector<std::set<unsigned int> > tempHolesMasksSetVector;
        tempHolesMasksSetVector = holesMasksSetVector;

        if (operationId == ASSIMILATION)
        {
          // Delete the passiveId-th candidate hole
          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }
        else if (operationId == AMALGAMATION)
        {
          // Delete the passiveId-th candidate hole,
          // alter the activeId-th candidate hole so that it has amalgamated
          // the passiveId-th candidate hole
          HoleMerger::amalgamateOnce(&tempHolesConveyor,
            activeId,
            &tempHolesMasksSetVector[activeId],
            tempHolesMasksSetVector[passiveId],
            image);

          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }
        else if (operationId == CONNECTION)
        {
          // Delete the passiveId-th candidate hole,
          // alter the activeId-th candidate hole so that it has been
          // connected with the passiveId -th candidate hole
          HoleMerger::connectOnce(&tempHolesConveyor,
            activeId, passiveId,
            &tempHolesMasksSetVector[activeId],
            image);

          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }


        // Since the {assimilator, amalgamator, connector} is able,
        // delete the assimilable's entries in the vectors needed
        // for filtering and merging
        tempHolesMasksSetVector.erase(
          tempHolesMasksSetVector.begin() + passiveId);


        // Obtain the activeId-th candidate hole in order for it
        // to be checked against the selected filters
        HolesConveyor ithHole =
          HolesConveyorUtils::getHole(tempHolesConveyor, activeId);

        // Create the necessary vectors for each hole checker and
        // merger used
        std::vector<cv::Mat> imgs;
        std::vector<std::string> msgs;
        std::vector<std::vector<cv::Point2f> > rectanglesVector;
        std::vector<int> rectanglesIndices;
        std::vector<std::set<unsigned int> > intermediatePointsSetVector;

        // The inflated rectangles vector is used in the
        // checkHolesDepthDiff and checkHolesRectangleEdgesPlaneConstitution
        // checkers
        FiltersResources::createInflatedRectanglesVector(
          ithHole,
          image,
          Parameters::HoleFusion::rectangle_inflation_size,
          &rectanglesVector,
          &rectanglesIndices);

        // The vector of depth-filters-derived probabilities
        std::vector<std::vector<float> >probabilitiesVector(
          2,
          std::vector<float>(1, 0.0));

        // Check the difference between the mean depth of the
        // vertices of the merged hole's bounding box and the depth of the
        // merged hole's keypoint
        DepthFilters::checkHolesDepthDiff(
          image,
          ithHole,
          rectanglesVector,
          rectanglesIndices,
          &msgs,
          &probabilitiesVector.at(0));

        // The probability that the merged hole is valid by the above filter
        float dd = probabilitiesVector[0][0];


        // Check the depth / area proportion for the ithHole
        DepthFilters::checkHolesDepthArea(
          ithHole,
          image,
          tempHolesMasksSetVector,
          &msgs,
          &probabilitiesVector.at(1));

        // The probability that the merged hole is valid by the above filter
        float da = probabilitiesVector[1][0];


        // Probabilities threshold for merge acceptance.
        // In the assimilation operation, the temp conveyor unconditionally
        // replaces the original conveyor
        if ((dd >= Parameters::HoleFusion::merger_depth_diff_threshold
          && da >= Parameters::HoleFusion::merger_depth_area_threshold))
        {
          // Since the tempHolesConveyor's ithHole has been positively tested,
          // the tempHolesConveyor is now the new rgbdHolesConveyor
          HolesConveyorUtils::replace(tempHolesConveyor, rgbdHolesConveyor);

          // ..and the new holesMasksSetVector is the positively tested
          // temp one
          holesMasksSetVector = tempHolesMasksSetVector;

          // Delete the passiveId-th entry of the finishVector since the
          // passiveId-th hole has been absorbed by the activeId-th hole
          finishVector.erase(finishVector.begin() + passiveId);

          // Because of the merge happening, the activeId-th
          // candidate hole must re-examine all the other holes
          passiveId = 1;
        }
        else // rgbdHolesConveyor remains unchanged
        {
          // passiveId-th hole not merged. let's see about the next one
          passiveId++;
        }
      }
      else // isAble == false
      {
        // passiveId-th hole not merged. let's see about the next one
        passiveId++;
      }

      // If the passiveId-th hole was the last one to be checked for merge,
      // the one doing the merge is rendered obsolete, so go to the next one
      // by moving the current activeId-th candidate hole to the back
      // of the rgbdHolesConveyor. This way the new activeId-th candidate
      // hole still has a value of 0, but now points to the candidate hole
      // next to the one that was moved back
      if (passiveId >= rgbdHolesConveyor->size())
      {
        // No meaning moving to the back of the rgbdHolesConveyor if
        // there is only one candidate hole
        if (rgbdHolesConveyor->size() > 1)
        {
          // activeId-th hole candidate finished examining the rest of the
          // hole candidates. move it to the back of the rgbdHolesConveyor
          HolesConveyorUtils::append(
            HolesConveyorUtils::getHole(*rgbdHolesConveyor, activeId),
            rgbdHolesConveyor);

          // Remove the activeId-th candidate hole from its former position
          HolesConveyorUtils::removeHole(rgbdHolesConveyor, activeId);

          // Remove the activeId-th set from its position and append it
          holesMasksSetVector.push_back(holesMasksSetVector[activeId]);
          holesMasksSetVector.erase(holesMasksSetVector.begin() + activeId);

          // Since the candidate hole was appended at the end of the
          // rgbdHolesConveyor, the finish vector needs to be shifted
          // once to the left because the value 1 is always set at the end
          // of the finishVector vector. See below.
          std::rotate(finishVector.begin(), finishVector.begin() + 1,
            finishVector.end());

          // Return the passive's candidate hole identifier to the
          // next of the active's candidate hole identifier, which is 0
          passiveId = 1;
        }

        // Since the ith candidate hole was appended at the end of the
        // rgbdHolesConveyor, the position to which it corresponds in the
        // finishVector is at the end of the vector.
        // Place the value of 1 in the last position, indicating that the
        // previously activeId-th candidate hole has finished examining all
        // other candidate holes for merging
        std::vector<int>::iterator finishVectorIterator =
          finishVector.end() - 1;

        *finishVectorIterator = 1;

        // Count how many aces there are in the finishVector
        // If they amount to the size of the vector, that means that
        // each hole has finished examining the others, and the current
        // operation is complete
        int numAces = 0;
        for (int i = 0; i < finishVector.size(); i++)
        {
          numAces += finishVector[i];
        }

        if (numAces == finishVector.size())
        {
          isFuseComplete = true;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyMergeOperation");
    #endif
  }



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
    @param[in] operationId [const int&] The identifier of the merging
    process. Values: 0 for assimilation, 1 for amalgamation and
    2 for connecting
    @return void
   **/
  void HoleMerger::applyMergeOperationWithoutValidation(
    HolesConveyor* rgbdHolesConveyor,
    const cv::Mat& image,
    const int& operationId)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyMergeOperation", "mergeHoles");
    #endif

    // If there are no candidate holes,
    // or there is only one candidate hole,
    // there is no meaning to this operation
    if (rgbdHolesConveyor->size() < 2)
    {
      return;
    }

    // A vector that indicates when a specific hole has finished
    // examining all the other holes in the conveyor for merging.
    // Initialized at 0 for all conveyor entries, the specific hole
    // that corresponds to a vector's entry is given a 1 when it has
    // finished examining all other holes.
    std::vector<int> finishVector(rgbdHolesConveyor->size(), 0);

    // The index of the candidate hole that will
    // {assimilate, amalgamate} the passiveId-th candidate hole.
    // The activeId always has a value of 0 due to the implementation's
    // rationale: The candidate hole that examines each hole in the
    // rgbdHolesConveyor is always the first one. When it has finished,
    // it goes back into the rgbdHolesConveyor, at the last position
    const int activeId = 0;

    // The index of the candidate hole that will be
    // {assimilated, amalgamated} by / with
    // the activeId-th candidate hole
    int passiveId = 1;

    bool isFuseComplete = false;
    while(!isFuseComplete)
    {
      // The holesMaskSetVector vector is used in the merging processes
      std::vector<std::set<unsigned int> > holesMasksSetVector;
      FiltersResources::createHolesMasksSetVector(
        *rgbdHolesConveyor,
        image,
        &holesMasksSetVector);


      // Is the activeId-th candidate hole able to
      // {assimilate, amalgamate} the passiveId-th candidate hole?
      bool isAble = false;

      if (operationId == ASSIMILATION)
      {
        // Is the activeId-th candidate hole able to assimilate the
        // passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAssimilating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }
      if (operationId == AMALGAMATION)
      {
        // Is the activeId-th candidate hole able to amalgamate the
        // passiveId-th candidate hole?
        isAble = HoleMerger::isCapableOfAmalgamating(
          holesMasksSetVector[activeId],
          holesMasksSetVector[passiveId]);
      }

      if (isAble)
      {
        // Copy the original holes conveyor to a temp one.
        // The temp one will be tested through the hole filters
        // On success, temp will replace rgbdHolesConveyor,
        // on failure, rgbdHolesConveyor will remain unchanged
        HolesConveyor tempHolesConveyor;
        HolesConveyorUtils::copyTo(*rgbdHolesConveyor, &tempHolesConveyor);

        // Copy the original holes masks set to a temp one.
        // If the temp conveyor is tested successfully through the hole
        // filters, temp will replace the original.
        // On failure, the original will remain unchanged
        std::vector<std::set<unsigned int> > tempHolesMasksSetVector;
        tempHolesMasksSetVector = holesMasksSetVector;

        if (operationId == ASSIMILATION)
        {
          // Delete the passiveId-th candidate hole
          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }
        else if (operationId == AMALGAMATION)
        {
          // Delete the passiveId-th candidate hole,
          // alter the activeId-th candidate hole so that it has amalgamated
          // the passiveId-th candidate hole
          HoleMerger::amalgamateOnce(&tempHolesConveyor,
            activeId,
            &tempHolesMasksSetVector[activeId],
            tempHolesMasksSetVector[passiveId],
            image);

          HolesConveyorUtils::removeHole(&tempHolesConveyor, passiveId);
        }

        // Since the {assimilator, amalgamator} is able,
        // delete the assimilable's entries in the vectors needed
        // for filtering and merging
        tempHolesMasksSetVector.erase(
          tempHolesMasksSetVector.begin() + passiveId);

        // Since the merge has been uncondionally happened,
        // the tempHolesConveyor is now the new rgbdHolesConveyor
        HolesConveyorUtils::replace(tempHolesConveyor, rgbdHolesConveyor);

        // ..and the new holesMasksSetVector is the positively tested
        // temp one
        holesMasksSetVector = tempHolesMasksSetVector;

        // Delete the passiveId-th entry of the finishVector since the
        // passiveId-th hole has been absorbed by the activeId-th hole
        finishVector.erase(finishVector.begin() + passiveId);

        // Because of the merge happening, the activeId-th
        // candidate hole must re-examine all the other holes
        passiveId = 1;
      }
      else // isAble == false
      {
        // passiveId-th hole not merged. let's see about the next one
        passiveId++;
      }

      // If the passiveId-th hole was the last one to be checked for merge,
      // the one doing the merge is rendered obsolete, so go to the next one
      // by moving the current activeId-th candidate hole to the back
      // of the rgbdHolesConveyor. This way the new activeId-th candidate
      // hole still has a value of 0, but now points to the candidate hole
      // next to the one that was moved back
      if (passiveId >= rgbdHolesConveyor->size())
      {
        // No meaning moving to the back of the rgbdHolesConveyor if
        // there is only one candidate hole
        if (rgbdHolesConveyor->size() > 1)
        {
          // activeId-th hole candidate finished examining the rest of the
          // hole candidates. move it to the back of the rgbdHolesConveyor
          HolesConveyorUtils::append(
            HolesConveyorUtils::getHole(*rgbdHolesConveyor, activeId),
            rgbdHolesConveyor);

          // Remove the activeId-th candidate hole from its former position
          HolesConveyorUtils::removeHole(rgbdHolesConveyor, activeId);


          // Remove the activeId-th set from its position and append it
          holesMasksSetVector.push_back(holesMasksSetVector[activeId]);
          holesMasksSetVector.erase(holesMasksSetVector.begin() + activeId);


          // Since the candidate hole was appended at the end of the
          // rgbdHolesConveyor, the finish vector needs to be shifted
          // once to the left because the value 1 is always set at the end
          // of the finishVector vector. See below.
          std::rotate(finishVector.begin(), finishVector.begin() + 1,
            finishVector.end());

          // Return the passive's candidate hole identifier to the
          // next of the active's candidate hole identifier, which is 0
          passiveId = 1;
        }

        // Since the ith candidate hole was appended at the end of the
        // rgbdHolesConveyor, the position to which it corresponds in the
        // finishVector is at the end of the vector.
        // Place the value of 1 in the last position, indicating that the
        // previously activeId-th candidate hole has finished examining all
        // other candidate holes for merging
        std::vector<int>::iterator finishVectorIterator =
          finishVector.end() - 1;

        *finishVectorIterator = 1;

        // Count how many aces there are in the finishVector
        // If they amount to the size of the vector, that means that
        // each hole has finished examining the others, and the current
        // operation is complete
        int numAces = 0;
        for (int i = 0; i < finishVector.size(); i++)
        {
          numAces += finishVector[i];
        }

        if (numAces == finishVector.size())
        {
          isFuseComplete = true;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyMergeOperation");
    #endif
  }



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
  bool HoleMerger::isCapableOfAssimilating(
    const std::set<unsigned int>& assimilatorHoleMaskSet,
    const std::set<unsigned int>& assimilableHoleMaskSet)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfAssimilating", "applyMergeOperation");
    #endif

    // If the assimilable's area is larger than the assimilator's,
    // this assimilator is not capable of assimilating the assimilatable
    if (assimilatorHoleMaskSet.size() < assimilableHoleMaskSet.size())
    {
      return false;
    }

    // Keep the size of the input assimilatorHoleMaskSet for
    // comparing against it
    int assimilatorHoleMaskSetSize = assimilatorHoleMaskSet.size();

    // Clone the assimilator's hole mask set.
    std::set<unsigned int> summation = assimilatorHoleMaskSet;

    // Try to insert every element of the assimilable hole mask set into
    // the assimilator's set.
    // This assimilator can assimilate the assimilable if and only if the
    // assimilable is inside the assimilator, in other words, if the
    // assimilator's hole mask set size remains unchanged after the
    // insertion of the assimilable's elements into it
    for (std::set<unsigned int>::iterator it = assimilableHoleMaskSet.begin();
      it != assimilableHoleMaskSet.end(); it++)
    {
      summation.insert(*it);

      if (summation.size () != assimilatorHoleMaskSetSize)
      {
        return false;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("isCapableOfAssimilating");
    #endif

    return true;
  }



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
  bool HoleMerger::isCapableOfAmalgamating(
    const std::set<unsigned int>& amalgamatorHoleMaskSet,
    const std::set<unsigned int>& amalgamatableHoleMaskSet)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfAmalgamating", "applyMergeOperation");
    #endif

    // If the amalgatamable's area is larger than the amalgamator's,
    // this amalgamator is not capable of amalgamating the amalgamatable
    if (amalgamatorHoleMaskSet.size() < amalgamatableHoleMaskSet.size())
    {
      return false;
    }

    // Keep the size of the input assimilatorHoleMaskSet for
    // comparing against it
    int amalgamatorHoleMaskSetSize = amalgamatorHoleMaskSet.size();

    // Clone the amalgamator's hole mask set.
    std::set<unsigned int> summation = amalgamatorHoleMaskSet;

    // Try to insert every element of the amalgamatable hole mask set into
    // the amalgamator's set.
    // This amalgamator can amalgamate the amalgamatable if and only if the
    // amalgamatable is not entirely inside the amalgamator or
    // the amalgamatable and the amalgamator are not not connected
    for (std::set<unsigned int>::iterator it = amalgamatableHoleMaskSet.begin();
      it != amalgamatableHoleMaskSet.end(); it++)
    {
      summation.insert(*it);
    }

    if ((summation.size() == amalgamatorHoleMaskSetSize) || (summation.size() ==
          amalgamatorHoleMaskSetSize + amalgamatableHoleMaskSet.size()))
    {
      return false;
    }

    #ifdef DEBUG_TIME
    Timer::tick("isCapableOfAmalgamating");
    #endif

    return true;
  }



  /**
    @brief Intended to use after the check of the
    isCapableOfAmalgamating function, this function modifies the
    HolesConveyor conveyor[amalgamatorId] entry so that it
    has absorbed the amalgamatable hole (conveyor[amalgamatableId])
    in terms of keypoint location, outline unification and bounding
    rectangle inclusion of the amalgamatable's outline
    @param[in,out] conveyor [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be modified. CAUTION: the amalgamatable is not deleted upon
    amalgamation, only the internals of the amalgamator are modified
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
  void HoleMerger::amalgamateOnce(
    HolesConveyor* conveyor,
    const int& amalgamatorId,
    std::set<unsigned int>* amalgamatorHoleMaskSet,
    const std::set<unsigned int>& amalgamatableHoleMaskSet,
    const cv::Mat& image)
  {
    #ifdef DEBUG_TIME
    Timer::start("amalgamateOnce", "applyMergeOperation");
    #endif


    // Now, we need to find the combined outline points
    // On an image, draw the holes' masks.
    // Invert the image and brushfire in the black space in order to
    // obtain the new outline points
    cv::Mat canvas = cv::Mat::zeros(image.size(), CV_8UC1);

    unsigned char* ptr = canvas.ptr();

    // Draw the amalgamator's mask onto canvas
    for (std::set<unsigned int>::iterator it = amalgamatorHoleMaskSet->begin();
      it != amalgamatorHoleMaskSet->end(); it++)
    {
      ptr[*it] = 255;
    }

    // Draw the amalgamatable's mask onto canvas
    for (std::set<unsigned int>::iterator it = amalgamatableHoleMaskSet.begin();
      it != amalgamatableHoleMaskSet.end(); it++)
    {
      ptr[*it] = 255;

      // In the meantime, construct the amalgamator's new hole set
      amalgamatorHoleMaskSet->insert(*it);
    }

    // Invert canvas
    for (int rows = 0; rows < image.rows; rows++)
    {
      for (int cols = 0; cols < image.cols; cols++)
      {
        unsigned int ind = cols + rows * image.cols;
        if (ptr[ind] == 0)
        {
          ptr[ind] = 255;
        }
        else
        {
          ptr[ind] = 0;
        }
      }
    }

    // Locate the outline of the combined hole
    std::vector<cv::Point2f> newAmalgamatorOutline;
    float area;
    BlobDetection::brushfireKeypoint(
      conveyor->holes[amalgamatorId].keypoint,
      &canvas,
      &newAmalgamatorOutline,
      &area);

    conveyor->holes[amalgamatorId].outline = newAmalgamatorOutline;


    // The amalgamator's new least area rotated bounding box will be the
    // one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(conveyor->holes[amalgamatorId].outline);

    // Obtain the four vertices of the new rotated rectangle
    cv::Point2f substituteVerticesArray[4];
    substituteRotatedRectangle.points(substituteVerticesArray);

    // Same as substituteVerticesArray array, but vector
    std::vector<cv::Point2f> substituteVerticesVector;

    // Store the four vertices to the substituteVerticesVector
    for(int v = 0; v < 4; v++)
    {
      substituteVerticesVector.push_back(substituteVerticesArray[v]);
    }

    // Replace the amalgamator's vertices with the new vertices
    conveyor->holes[amalgamatorId].rectangle.erase(
      conveyor->holes[amalgamatorId].rectangle.begin(),
      conveyor->holes[amalgamatorId].rectangle.end());

    conveyor->holes[amalgamatorId].rectangle = substituteVerticesVector;


    // Set the overall candidate hole's keypoint to the center of the
    // newly created bounding rectangle
    float x = 0;
    float y = 0;
    for (int k = 0; k < 4; k++)
    {
      x += conveyor->holes[amalgamatorId].rectangle[k].x;
      y += conveyor->holes[amalgamatorId].rectangle[k].y;
    }

    conveyor->holes[amalgamatorId].keypoint.pt.x = x / 4;
    conveyor->holes[amalgamatorId].keypoint.pt.y = y / 4;

    #ifdef DEBUG_TIME
    Timer::tick("amalgamateOnce");
    #endif
  }



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
  bool HoleMerger::isCapableOfConnecting(
    const HolesConveyor& conveyor,
    const int& connectorId,
    const int& connectableId,
    const std::set<unsigned int>& connectorHoleMaskSet,
    const std::set<unsigned int>& connectableHoleMaskSet,
    const PointCloudPtr& pointCloud)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfConnecting", "applyMergeOperation");
    #endif

    // If the connectable's area is greater than the connector's,
    // this connectable is not capable of being connected with the connector
    if (connectorHoleMaskSet.size() < connectableHoleMaskSet.size())
    {
      return false;
    }

    // Keep the size of the input assimilatorHoleMaskSet for
    // comparing against it
    int connectorHoleMaskSetSize = connectorHoleMaskSet.size();

    // Clone the connector's hole mask set.
    std::set<unsigned int> summation = connectorHoleMaskSet;

    // Try to insert every element of the assimilable hole mask set into
    // the connector's set.
    // This connectable can be connected with the connector if and only if the
    // connectable is outside of the connector
    for (std::set<unsigned int>::iterator it = connectableHoleMaskSet.begin();
      it != connectableHoleMaskSet.end(); it++)
    {
      summation.insert(*it);
    }

    if (summation.size () !=
      (connectorHoleMaskSetSize + connectableHoleMaskSet.size()))
    {
      return false;
    }


    // The real min distance (in meters) between two points of the
    // connector's and connectable's outlines
    double minOutlinesDistance = 10000.0;

    // The real max distance (in meters) between two points of the
    // connector's and connectable's outlines
    double maxOutlinesDistance = 0.0;

    for (int av = 0; av < conveyor.holes[connectableId].outline.size(); av++)
    {
      // The connectable's current outline point x,y,z coordinates
      // measured by the depth sensor
      float connectableOutlinePointX = pointCloud->points[
        static_cast<int>(conveyor.holes[connectableId].outline[av].x)
        + pointCloud->width *
        static_cast<int>(conveyor.holes[connectableId].outline[av].y)].x;

      float connectableOutlinePointY = pointCloud->points[
        static_cast<int>(conveyor.holes[connectableId].outline[av].x)
        + pointCloud->width *
        static_cast<int>(conveyor.holes[connectableId].outline[av].y)].y;

      float connectableOutlinePointZ = pointCloud->points[
        static_cast<int>(conveyor.holes[connectableId].outline[av].x)
        + pointCloud->width *
        static_cast<int>(conveyor.holes[connectableId].outline[av].y)].z;


      for (int ac = 0; ac < conveyor.holes[connectorId].outline.size(); ac++)
      {
        // The connector's current outline point x,y,z coordinates
        // measured by the depth sensor
        float connectorOutlinePointX = pointCloud->points[
          static_cast<int>(conveyor.holes[connectorId].outline[ac].x)
          + pointCloud->width *
          static_cast<int>(conveyor.holes[connectorId].outline[ac].y)].x;

        float connectorOutlinePointY = pointCloud->points[
          static_cast<int>(conveyor.holes[connectorId].outline[ac].x)
          + pointCloud->width *
          static_cast<int>(conveyor.holes[connectorId].outline[ac].y)].y;

        float connectorOutlinePointZ = pointCloud->points[
          static_cast<int>(conveyor.holes[connectorId].outline[ac].x)
          + pointCloud->width *
          static_cast<int>(conveyor.holes[connectorId].outline[ac].y)].z;


        // The current outline points distance
        float outlinePointsDistance = sqrt(
          pow(connectableOutlinePointX - connectorOutlinePointX, 2) +
          pow(connectableOutlinePointY - connectorOutlinePointY, 2) +
          pow(connectableOutlinePointZ - connectorOutlinePointZ, 2));

        if (outlinePointsDistance < minOutlinesDistance)
        {
          minOutlinesDistance = outlinePointsDistance;
        }
        if (outlinePointsDistance > maxOutlinesDistance)
        {
          maxOutlinesDistance = outlinePointsDistance;
        }
      }
    }

    // If the minimum distance between the connector's and connectable's
    // outlines is greater than a distance thrshold,
    // this connectable is not a candidate to be connected with
    // the connector
    if (minOutlinesDistance > Parameters::HoleFusion::connect_holes_min_distance)
    {
      return false;
    }

    // If the maximum distance between the connector's and connectable's
    // outlines is greater than a distance thrshold,
    // this connectable is not a candidate to be connected with
    // the connector
    if (maxOutlinesDistance > Parameters::HoleFusion::connect_holes_max_distance)
    {
      return false;
    }


    #ifdef DEBUG_TIME
    Timer::tick("isCapableOfConnecting");
    #endif

    return true;
  }



  /**
    @brief Intended to use after the check of the
    isCapableOfConnecting function, this method modifies the HolesConveyor
    connector struct entry so that it it has absorbed the connectable hole
    in terms of keypoint location, outline unification and bounding
    rectangle inclusion of the connectable's outline
    @param[in,out] conveyor [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be modified. CAUTION: the connectable is not deleted upon
    connection, only the internals of the connector are modified
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
  void HoleMerger::connectOnce(
    HolesConveyor* conveyor,
    const int& connectorId,
    const int& connectableId,
    std::set<unsigned int>* connectorHoleMaskSet,
    const cv::Mat& image)
  {
    #ifdef DEBUG_TIME
    Timer::start("connectOnce", "applyMergeOperation");
    #endif

    // The connection rationale is as follows:
    // Since the two holes are not overlapping each other,
    // some sort of connection regime has to be established.
    // The idea is that the points that consist the outline of each hole
    // are connected via a cv::line so that the overall connector's outline
    // is the overall shape's outline


    // The image on which the hole's outline connection will be drawn
    cv::Mat canvas = cv::Mat::zeros(image.size(), CV_8UC1);
    unsigned char* ptr = canvas.ptr();

    for (int i = 0; i < conveyor->holes[connectorId].outline.size(); i++)
    {
      for (int j = 0; j < conveyor->holes[connectableId].outline.size(); j++)
      {
        cv::line(canvas,
          conveyor->holes[connectorId].outline[i],
          conveyor->holes[connectableId].outline[j],
          cv::Scalar(255, 0, 0), 1, 8);
      }
    }

    // Construct the connector's new hole mask
    // First, clear the former one
    connectorHoleMaskSet->erase(
      connectorHoleMaskSet->begin(),
      connectorHoleMaskSet->end());

    for (int rows = 0; rows < canvas.rows; rows++)
    {
      for (int cols = 0; cols < canvas.cols; cols++)
      {
        unsigned int ind = cols + rows * image.cols;
        if (ptr[ind] != 0)
        {
          connectorHoleMaskSet->insert(ptr[ind]);
        }
      }
    }

    // Invert canvas
    for (int rows = 0; rows < image.rows; rows++)
    {
      for (int cols = 0; cols < image.cols; cols++)
      {
        unsigned int ind = cols + rows * image.cols;
        if (ptr[ind] == 0)
        {
          ptr[ind] = 255;
        }
        else
        {
          ptr[ind] = 0;
        }
      }
    }

    // Locate the outline of the combined hole
    std::vector<cv::Point2f> newConnectorOutline;
    float area;
    BlobDetection::brushfireKeypoint(
      conveyor->holes[connectorId].keypoint,
      &canvas,
      &newConnectorOutline,
      &area);

    conveyor->holes[connectorId].outline = newConnectorOutline;


    // The connectable's new least area rotated bounding box will be the
    // one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(conveyor->holes[connectorId].outline);

    // Obtain the four vertices of the new rotated rectangle
    cv::Point2f substituteVerticesArray[4];
    substituteRotatedRectangle.points(substituteVerticesArray);

    // Same as substituteVerticesArray array, but vector
    std::vector<cv::Point2f> substituteVerticesVector;

    // Store the four vertices to the substituteVerticesVector
    for(int v = 0; v < 4; v++)
    {
      substituteVerticesVector.push_back(substituteVerticesArray[v]);
    }

    // Replace the connector's vertices with the new vertices
    conveyor->holes[connectorId].rectangle.erase(
      conveyor->holes[connectorId].rectangle.begin(),
      conveyor->holes[connectorId].rectangle.end());

    // Replace the connector's vertices with the new vertices
    conveyor->holes[connectorId].rectangle = substituteVerticesVector;


    // Set the overall candidate hole's keypoint to the mean of the keypoints
    // of the connector and the connectable
    conveyor->holes[connectorId].keypoint.pt.x =
      (conveyor->holes[connectorId].keypoint.pt.x
       + conveyor->holes[connectableId].keypoint.pt.x) / 2;

    conveyor->holes[connectorId].keypoint.pt.y =
      (conveyor->holes[connectorId].keypoint.pt.y
       + conveyor->holes[connectableId].keypoint.pt.y) / 2;

    #ifdef DEBUG_TIME
    Timer::tick("connectOnce");
    #endif
  }



  /**
    @brief With an input a conveyor of holes, this method, depending on
    the depth image's interpolation method, has holes assimilating,
    amalgamating or being connected with holes that can be assimilated,
    amalgamated or connected with or by them. The interpolation method is
    a basic criterion for the mode of merging holes because the two
    filters that verify the validity of each merger are depth-based.
    If there is no valid depth image on which to run these filters, it is
    sure that the depth sensor is closer to the scene it is witnessing
    than 0.5-0.6m. In this way of operation, the merging of holes does not
    consider employing validator filters and simply merges holes that can
    be merged with each other (assimilated, amalgamated, or connected).
    @param[in,out] conveyor [HolesConveyor*]
    The conveyor of holes to be merged with one another, where applicable.
    @param[in] filteringMethod [const int&]
    Indicates whether candidate holes can be merged on conditions
    dictated by Depth filters, or depth analysis is altogether
    not possible, and so merges will happen unconditionally.
    @param[in] interpolatedDepthImage [const cv::Mat&]
    The interpolated depth image
    @param[in] pointCloud [const PointCloudPtr&]
    The interpolated point cloud. Needed in the connection process.
    @return void
   **/
  void HoleMerger::mergeHoles(HolesConveyor* conveyor,
    const int& filteringMethod,
    const cv::Mat& interpolatedDepthImage,
    const PointCloudPtr& pointCloud)
  {
    #ifdef DEBUG_TIME
    Timer::start("mergeHoles", "processCandidateHoles");
    #endif

    // Keep a copy of the initial (not merged) candidate holes for
    // debugging and exibition purposes
    HolesConveyor conveyorBeforeMerge;

    HolesConveyorUtils::copyTo(*conveyor, &conveyorBeforeMerge);

    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> canvases;
    std::vector<std::string> titles;

    if(Parameters::Debug::show_merge_holes)
    {
      // Push back the identifier of each keypoint
      for (int i = 0; i < conveyorBeforeMerge.size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage,
          conveyorBeforeMerge,
          -1,
          msgs));

      titles.push_back(
        TOSTR(conveyor->size()) + " holes before merging");
    }
    #endif

    // Try to merge holes that can be assimilated, amalgamated or connected.
    // If Depth analysis is applicable, {assimilate, amalgamate, connect}
    // conditionally, based on depth filters. In the opposite case,
    // {assimilate, amalgamate, connect} unconditionally.
    if (filteringMethod == RGBD_MODE)
    {
      for (int i = 0; i < 3; i++)
      {
        HoleMerger::applyMergeOperation(
          conveyor,
          interpolatedDepthImage,
          pointCloud,
          i);
      }
    }
    else if (filteringMethod == RGB_ONLY_MODE)
    {
      for (int i = 0; i < 3; i++)
      {
        HoleMerger::applyMergeOperationWithoutValidation(
          conveyor,
          interpolatedDepthImage,
          i);
      }
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME, "[Hole Fusion node] Merging process failure");
    }

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_merge_holes)
    {
      msgs.clear();

      // Push back the identifier of each keypoint
      for (int i = 0; i < conveyor->size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage,
          *conveyor,
          -1,
          msgs));

      titles.push_back(
        TOSTR(conveyor->size()) + " holes after merging");

      Visualization::multipleShow("Merged Keypoints",
        canvases, titles, Parameters::Debug::show_merge_holes_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("mergeHoles");
    #endif
  }

} // namespace pandora_vision
