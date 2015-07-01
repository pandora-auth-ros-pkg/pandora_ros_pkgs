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

#include "hole_fusion_node/hole_uniqueness.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  /**
    @brief Given a HolesConveyor container, this method detects
    multiple unique holes and deletes the extra copies.
    @param[in,out] conveyor [HolesConveyor*] The container of holes
    @return void
   **/
  void HoleUniqueness::makeHolesUnique(HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("makeHolesUniqueA", "processCandidateHoles");
    #endif

    // A container in which one of every duplicate hole will be inserted
    HolesConveyor uniqueDuplicates;

    // A set of sets containing the indices of duplicate holes.
    // Each internal set points to one unique hole
    std::set<std::set<int> > duplicateIndices;
    for (int i = 0; i < conveyor->size(); i++)
    {
      // This set is comprised by indices of holes that are identical to the
      // i-th hole
      std::set<int> identical;
      for (int j = 0; j < conveyor->size(); j++)
      {
        if (i != j)
        {
          // First off, the two holes' keypoints must be identical.
          if (conveyor->holes[i].keypoint.pt.x ==
            conveyor->holes[j].keypoint.pt.x &&
            conveyor->holes[i].keypoint.pt.y ==
            conveyor->holes[j].keypoint.pt.y)
          {
            // Second off, it is sufficient to check the identicalness of
            // the vertices of the two holes' bounding boxes.
            for (int v = 0; v < conveyor->holes[i].rectangle.size(); v++)
            {
              if (conveyor->holes[i].rectangle[v].x ==
                conveyor->holes[j].rectangle[v].x &&
                conveyor->holes[i].rectangle[v].y ==
                conveyor->holes[j].rectangle[v].y)
              {
                identical.insert(j);
              }
            }
          }
        }
      }

      if (identical.size() > 0)
      {
        // The i-th hole is part of the overall set
        identical.insert(i);

        duplicateIndices.insert(identical);
      }
    }


    // Populate the uniqueDuplicates container with a copy of each duplicate
    // hole. All the duplicate holes will be deleted from the input conveyor,
    // and then merged with the uniqueDuplicates conveyor.
    for (std::set<std::set<int> >::iterator d_it = duplicateIndices.begin();
      d_it != duplicateIndices.end(); d_it++)
    {
      std::set<int>::iterator it = d_it->begin();

      HolesConveyorUtils::append(
        HolesConveyorUtils::getHole(*conveyor, *it),
        &uniqueDuplicates);
    }

    // Now, all the duplicate holes must be deleted from the input conveyor.
    // Construct ONE set of indices of all holes that need to be deleted.
    std::set<int> duplicates;
    for (std::set<std::set<int> >::iterator d_it = duplicateIndices.begin();
      d_it != duplicateIndices.end(); d_it++)
    {
      for (std::set<int>::iterator it = d_it->begin(); it != d_it->end(); it++)
      {
        duplicates.insert(*it);
      }
    }

    // Perform the actual deletion of duplicate holes.
    for (std::set<int>::reverse_iterator it = duplicates.rbegin();
      it != duplicates.rend(); it++)
    {
      conveyor->holes.erase(conveyor->holes.begin() + *it);
    }

    // Add one copy of each duplicate hole deleted to the conveyor container
    HolesConveyorUtils::append(uniqueDuplicates, conveyor);

    #ifdef DEBUG_TIME
    Timer::tick("makeHolesUniqueA");
    #endif
  }



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
  void HoleUniqueness::makeHolesUnique(HolesConveyor* conveyor,
    std::map<int, float>* validHolesMap)
  {
    #ifdef DEBUG_TIME
    Timer::start("makeHolesUniqueB", "processCandidateHoles");
    #endif

    // Each set inside the map refers a valid hole inside the conveyor conveyor.
    // The entries of each set are indices to valid holes inside
    // the conveyor conveyor.
    // Each map.first refers to a specific valid hole inside the conveyor
    // conveyor.
    // map.first-> map.second means the residence of the keypoint of the
    // hole with index map.first within the bounding box of holes with
    // indices in map.second
    std::map<int, std::set<int> > residenceMap;

    // Find the indices of holes that the keypoint of each valid hole
    // is located in.
    for (std::map<int, float>::iterator o_it = validHolesMap->begin();
      o_it != validHolesMap->end(); o_it++)
    {
      // The keypoint of the i-th hole in cv::Point2f format
      cv::Point2f keypoint;
      keypoint.x = conveyor->holes[o_it->first].keypoint.pt.x;
      keypoint.y = conveyor->holes[o_it->first].keypoint.pt.y;

      // A set of the identifiers of holes inside the conveyor conveyor,
      // whose bounding box the i-th hole is inside, recursively.
      std::set<int> parents;
      for (std::map<int, float>::iterator i_it = validHolesMap->begin();
        i_it != validHolesMap->end(); i_it++)
      {
        if (cv::pointPolygonTest(
            conveyor->holes[i_it->first].rectangle, keypoint, false) > 0)
        {
          parents.insert(i_it->first);
        }
      }

      residenceMap[o_it->first] = parents;
    }

    // The outcome is a map of ints to sets, one per valid hole.
    // Each set contains the indices of parent holes
    // that have been deemed valid.
    // What we want now is to locate which holes need to be
    // compared against one another.

    // Each set inside the groupSet contains holes whose overall validity
    // probability needs to be compared against all the others within that set.
    // The largest of them corresponds to a unique hole, whose keypoint,
    // outline and bounding rectangle are considered most accurate between
    // the holes in its respective set.
    std::set<std::set<int> > groupSet;
    for (std::map<int, std::set<int> >::iterator hole_it = residenceMap.begin();
      hole_it != residenceMap.end(); hole_it++)
    {
      // Traverse the parents (the hole_it->second set)
      // of this hole (its id is hole_it->first)
      std::set<int> holeComparingGroupSet;
      for (std::set<int>::iterator set_it = hole_it->second.begin();
        set_it != hole_it->second.end(); set_it++)
      {
        // For each parent of the hole_id->first-th hole,
        // check the set of its corresponding parent holes
        // and add them to the set of the hole_id->first-th hole's parents.
        // The grandfathers added are the hole_id->first-th hole's recursive
        // parents.
        for (std::map<int, std::set<int> >::iterator
          comp_it = residenceMap.begin();
          comp_it != residenceMap.end(); comp_it++)
        {
          // A parent to the parent of hole hole_it->first is also its parent.
          if (std::find(
              comp_it->second.begin(),
              comp_it->second.end(),
              *set_it) != comp_it->second.end())
          {
            for (std::set<int>::iterator a_it = comp_it->second.begin();
              a_it != comp_it->second.end(); a_it++)
            {
              holeComparingGroupSet.insert(*a_it);
            }
          }
        }

        // Insert the set of indices of holes with which the
        // hole_it->first-th hole will be compared to inside the overall
        // set of sets.
        groupSet.insert(holeComparingGroupSet);
      }
    }

    // The conveyor of unique and valid holes
    HolesConveyor uniqueHoles;

    // Maps a unique, valid hole to its validity probability
    std::map<int, float> finalMap;

    // A hole counter
    int numHoles = 0;

    // Iterate over the set of sets within which the validity probability
    // of holes needs to be compared with one another. We will locate the
    // hole in a set within groupSet that has the highest probability among
    // the holes inside the set
    for (std::set<std::set<int> >::iterator o_it = groupSet.begin();
      o_it != groupSet.end(); o_it++)
    {
      // The maximum probability of the cluser of nearby holes
      float maxProbability = 0.0;

      // The index of the hole with maximum probability inside the cluster
      // of nearby holes
      int index = 0;

      // Iterate over the current set inside the groupSet
      for (std::set<int>::iterator g_it = o_it->begin();
        g_it != o_it->end(); g_it++)
      {
        // An iterator over the input validHolesMap.
        // It points to the index and probability of the *g_it-th hole
        std::map<int, float>::iterator map_it = validHolesMap->find(*g_it);
        if (map_it != validHolesMap->end())
        {
          if (map_it->second > maxProbability)
          {
            maxProbability = map_it->second;
            index = map_it->first;
          }
        }
      }

      // The index-th hole inside this cluster of nearby holes is the most
      // accurate one, since its probability is the highest among its rearby
      // holes. Append it to the uniqueHoles conveyor.
      HolesConveyorUtils::append(
        HolesConveyorUtils::getHole(*conveyor, index),
        &uniqueHoles);

      // Map the most accurate hole to its probability.
      finalMap[numHoles] = maxProbability;

      numHoles++;
    }

    // All unique holes have been found.
    // Replace the uniqueHoles conveyor with the one containing
    // clusters of holes and the input map with the one corresponding to the
    // unique holes found.
    HolesConveyorUtils::replace(uniqueHoles, conveyor);
    *validHolesMap = finalMap;

    #ifdef DEBUG_TIME
    Timer::tick("makeHolesUniqueB");
    #endif
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
