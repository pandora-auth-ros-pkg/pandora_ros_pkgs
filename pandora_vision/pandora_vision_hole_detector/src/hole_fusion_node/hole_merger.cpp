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

namespace pandora_vision
{
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

    //!< If the assimilable's area is larger than the assimilator's,
    //!< this assimilator is not capable of assimilating the assimilatable
    if (assimilatorHoleMaskSet.size() < assimilableHoleMaskSet.size())
    {
      return false;
    }

    //!< Keep the size of the input assimilatorHoleMaskSet for
    //!< comparing against it
    int assimilatorHoleMaskSetSize = assimilatorHoleMaskSet.size();

    //!< Clone the assimilator's hole mask set.
    std::set<unsigned int> summation = assimilatorHoleMaskSet;

    //!< Try to insert every element of the assimilable hole mask set into
    //!< the assimilator's set.
    //!< This assimilator can assimilate the assimilable if and only if the
    //!< assimilable is inside the assimilator, in other words, if the
    //!< assimilator's hole mask set size remains unchanged after the
    //!< insertion of the assimilable's elements into it
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

    //!< If the amalgatamable's area is larger than the amalgamator's,
    //!< this amalgamator is not capable of amalgamating the amalgamatable
    if (amalgamatorHoleMaskSet.size() < amalgamatableHoleMaskSet.size())
    {
      return false;
    }

    //!< Keep the size of the input assimilatorHoleMaskSet for
    //!< comparing against it
    int amalgamatorHoleMaskSetSize = amalgamatorHoleMaskSet.size();

    //!< Clone the amalgamator's hole mask set.
    std::set<unsigned int> summation = amalgamatorHoleMaskSet;

    //!< Try to insert every element of the amalgamatable hole mask set into
    //!< the amalgamator's set.
    //!< This amalgamator can amalgamate the amalgamatable if and only if the
    //!< amalgamatable is not entirely inside the amalgamator or
    //!< the amalgamatable and the amalgamator are not not connected
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
    @param[in out] conveyor [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be modified
    @param[in] amalgamatorId [const int&] The identifier of the
    hole inside the HolesConveyor amalgamator struct
    @param[in out] amalgamatorHoleMaskSet [std::set<unsigned int>*]
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


    //!< Now, we need to find the combined outline points
    //!< On an image, draw the holes' masks.
    //!< Invert the image and brushfire in the black space in order to
    //!< obtain the new outline points
    cv::Mat canvas = cv::Mat::zeros(image.size(), CV_8UC1);

    unsigned char* ptr = canvas.ptr();

    //!< Draw the amalgamator's mask onto canvas
    for (std::set<unsigned int>::iterator it = amalgamatorHoleMaskSet->begin();
      it != amalgamatorHoleMaskSet->end(); it++)
    {
      ptr[*it] = 255;
    }

    //!< Draw the amalgamatable's mask onto canvas
    for (std::set<unsigned int>::iterator it = amalgamatableHoleMaskSet.begin();
      it != amalgamatableHoleMaskSet.end(); it++)
    {
      ptr[*it] = 255;

      //!< In the meantime, construct the amalgamator's new hole set
      amalgamatorHoleMaskSet->insert(*it);
    }

    //!< Invert canvas
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

    //!< Locate the outline of the combined hole
    std::vector<cv::Point2f> newAmalgamatorOutline;
    float area;
    BlobDetection::brushfireKeypoint(
      conveyor->keyPoints[amalgamatorId],
      &canvas,
      &newAmalgamatorOutline,
      &area);

    conveyor->outlines[amalgamatorId] = newAmalgamatorOutline;


    //!< The amalgamator's new least area rotated bounding box will be the
    //!< one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(conveyor->outlines[amalgamatorId]);

    //!< Obtain the four vertices of the new rotated rectangle
    cv::Point2f substituteVerticesArray[4];
    substituteRotatedRectangle.points(substituteVerticesArray);

    //!< Same as substituteVerticesArray array, but vector
    std::vector<cv::Point2f> substituteVerticesVector;

    //!< Store the four vertices to the substituteVerticesVector
    for(int v = 0; v < 4; v++)
    {
      substituteVerticesVector.push_back(substituteVerticesArray[v]);
    }

    //!< Replace the amalgamator's vertices with the new vertices
    conveyor->rectangles[amalgamatorId].erase(
      conveyor->rectangles[amalgamatorId].begin(),
      conveyor->rectangles[amalgamatorId].end());

    conveyor->rectangles.at(amalgamatorId) = substituteVerticesVector;


    //!< Set the overall candidate hole's keypoint to the center of the
    //!< newly created bounding rectangle
    float x = 0;
    float y = 0;
    for (int k = 0; k < 4; k++)
    {
      x += conveyor->rectangles[amalgamatorId][k].x;
      y += conveyor->rectangles[amalgamatorId][k].y;
    }

    conveyor->keyPoints[amalgamatorId].pt.x = x / 4;
    conveyor->keyPoints[amalgamatorId].pt.y = y / 4;

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
    @param[in] pointCloudXYZ [const PointCloudXYZPtr&] The point cloud
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
    const PointCloudXYZPtr& pointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfConnecting", "applyMergeOperation");
    #endif

    //!< If the connectable's area is greater than the connector's,
    //!< this connectable is not capable of being connected with the connector
    if (connectorHoleMaskSet.size() < connectableHoleMaskSet.size())
    {
      return false;
    }

    //!< Keep the size of the input assimilatorHoleMaskSet for
    //!< comparing against it
    int connectorHoleMaskSetSize = connectorHoleMaskSet.size();

    //!< Clone the connector's hole mask set.
    std::set<unsigned int> summation = connectorHoleMaskSet;

    //!< Try to insert every element of the assimilable hole mask set into
    //!< the connector's set.
    //!< This connectable can be connected with the connector if and only if the
    //!< connectable is outside of the connector
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


    //!< The real min distance (in meters) between two points of the
    //!< connector's and connectable's outlines
    double minOutlinesDistance = 10000.0;

    //!< The real max distance (in meters) between two points of the
    //!< connector's and connectable's outlines
    double maxOutlinesDistance = 0.0;

    for (int av = 0; av < conveyor.outlines[connectableId].size(); av++)
    {
      //!< The connectable's current outline point x,y,z coordinates
      //!< measured by the depth sensor
      float connectableOutlinePointX = pointCloudXYZ->points[
        static_cast<int>(conveyor.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(conveyor.outlines[connectableId][av].y)].x;

      float connectableOutlinePointY = pointCloudXYZ->points[
        static_cast<int>(conveyor.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(conveyor.outlines[connectableId][av].y)].y;

      float connectableOutlinePointZ = pointCloudXYZ->points[
        static_cast<int>(conveyor.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(conveyor.outlines[connectableId][av].y)].z;


      for (int ac = 0; ac < conveyor.outlines[connectorId].size(); ac++)
      {
        //!< The connector's current outline point x,y,z coordinates
        //!< measured by the depth sensor
        float connectorOutlinePointX = pointCloudXYZ->points[
          static_cast<int>(conveyor.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(conveyor.outlines[connectorId][ac].y)].x;

        float connectorOutlinePointY = pointCloudXYZ->points[
          static_cast<int>(conveyor.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(conveyor.outlines[connectorId][ac].y)].y;

        float connectorOutlinePointZ = pointCloudXYZ->points[
          static_cast<int>(conveyor.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(conveyor.outlines[connectorId][ac].y)].z;


        //!< The current outline points distance
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

    //!< If the minimum distance between the connector's and connectable's
    //!< outlines is greater than a distance thrshold,
    //!< this connectable is not a candidate to be connected with
    //!< the connector
    if (minOutlinesDistance > Parameters::connect_holes_min_distance)
    {
      return false;
    }

    //!< If the maximum distance between the connector's and connectable's
    //!< outlines is greater than a distance thrshold,
    //!< this connectable is not a candidate to be connected with
    //!< the connector
    if (maxOutlinesDistance > Parameters::connect_holes_max_distance)
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
    @param[in out] conveyor [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be modified
    @param[in] connectorId [const int&] The identifier of the hole inside
    the HolesConveyor connectables struct
    @param[in] connectableId [const int&] The identifier of the hole inside
    the HolesConveyor connectables struct
    @param[in] connectorHoleMaskSet [const std::set<unsigned int>&]
    A set that includes the indices of points inside the connector's
    outline
    @param[in] connectableHoleMaskSet [const std::set<unsigned int>&]
    A set that includes the indices of points inside the connectable's
    outline
    @return void
   **/
  void HoleMerger::connectOnce(
    HolesConveyor* conveyor,
    const int& connectorId,
    const int& connectableId,
    std::set<unsigned int>* connectorHoleMaskSet,
    const std::set<unsigned int>& connectableHoleMaskSet)
  {
    #ifdef DEBUG_TIME
    Timer::start("connectOnce", "applyMergeOperation");
    #endif

    for (std::set<unsigned int>::iterator it = connectableHoleMaskSet.begin();
      it != connectableHoleMaskSet.end(); it++)
    {
      //!< In the meanwhile, construct the amalgamator's new hole set
      connectorHoleMaskSet->insert(*it);
    }

    //!< The connector's outline will be the sum of the outline points
    //!< of the two conveyors
    conveyor->outlines[connectorId].insert(
      conveyor->outlines[connectorId].end(),
      conveyor->outlines[connectableId].begin(),
      conveyor->outlines[connectableId].end());


    //!< The connectable's new least area rotated bounding box will be the
    //!< one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(conveyor->outlines[connectorId]);

    //!< Obtain the four vertices of the new rotated rectangle
    cv::Point2f substituteVerticesArray[4];
    substituteRotatedRectangle.points(substituteVerticesArray);

    //!< Same as substituteVerticesArray array, but vector
    std::vector<cv::Point2f> substituteVerticesVector;

    //!< Store the four vertices to the substituteVerticesVector
    for(int v = 0; v < 4; v++)
    {
      substituteVerticesVector.push_back(substituteVerticesArray[v]);
    }

    //!< Replace the connector's vertices with the new vertices
    conveyor->rectangles[connectorId].erase(
      conveyor->rectangles[connectorId].begin(),
      conveyor->rectangles[connectorId].end());

    //!< Replace the connector's vertices with the new vertices
    conveyor->rectangles.at(connectorId) = substituteVerticesVector;


    //!< Set the overall candidate hole's keypoint to the center of the
    //!< newly created bounding rectangle
    float x = 0;
    float y = 0;
    for (int k = 0; k < 4; k++)
    {
      x += conveyor->rectangles[connectorId][k].x;
      y += conveyor->rectangles[connectorId][k].y;
    }

    conveyor->keyPoints[connectorId].pt.x = x / 4;
    conveyor->keyPoints[connectorId].pt.y = y / 4;

    #ifdef DEBUG_TIME
    Timer::tick("connectOnce");
    #endif
  }

} // namespace pandora_vision
