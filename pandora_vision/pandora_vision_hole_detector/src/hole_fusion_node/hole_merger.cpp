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

#include "hole_fusion_node/hole_merger.h"

namespace pandora_vision
{
  /**
    @brief Assimilates fragmented holes into existing whole ones
    from either source (RGB or Depth).
    @param[in][out] depthHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the depth node
    @param[in][out] rgbHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the rgb node
    @return void
   **/
  void HoleMerger::assimilateBilaterally(
    HolesConveyor* depthHolesConveyor,
    HolesConveyor* rgbHolesConveyor)
  {
    //!< Assimilating holes has a meaning only if both nodes have published
    //!< candidate holes
    if (depthHolesConveyor->keyPoints.size() > 0 &&
      rgbHolesConveyor->keyPoints.size() > 0)
    {
      assimilateUnilaterally(*rgbHolesConveyor, depthHolesConveyor);

      assimilateUnilaterally(*depthHolesConveyor, rgbHolesConveyor);
    }

  }



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
  void HoleMerger::assimilateUnilaterally(
    const HolesConveyor& assimilator,
    HolesConveyor* assimilable)
  {
    //!< Validate assimilable's holes against the assimilator's ones
    for (int i = 0; i < assimilator.keyPoints.size(); i++)
    {
      for (int j = assimilable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If all of assimilable's outline points reside inside the
        //!< assimilator's outline delete the keypoint along with its
        //!< associated conveyor entries
        if (isCapableOfAssimilating(i, assimilator, j, *assimilable))
        {
          assimilateOnce(j, assimilable);
        }
      }
    }
  }



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
  bool HoleMerger::isCapableOfAssimilating(
    const int& assimilatorId,
    const HolesConveyor& assimilator,
    const int& assimilableId,
    const HolesConveyor& assimilable)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfAssimilating", "applyMergeOperation");
    #endif

    //!< Are all the outline points of assimilable inside the
    //!< assimilator's outline?
    for (int av = 0; av < assimilable.outlines[assimilableId].size(); av++)
    {
      if (cv::pointPolygonTest(assimilator.outlines[assimilatorId],
          assimilable.outlines[assimilableId][av], false) < 0)
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
  void HoleMerger::assimilateOnce(const int& keyPointId,
    HolesConveyor* assimilable)
  {
    #ifdef DEBUG_TIME
    Timer::start("assimilateOnce", "applyMergeOperation");
    #endif

    //!< Delete the keypoint from the conveyor
    assimilable->keyPoints.erase(
      assimilable->keyPoints.begin() + keyPointId);


    //!< Delete each outline point from its respective vector
    assimilable->outlines[keyPointId].erase(
      assimilable->outlines[keyPointId].begin(),
      assimilable->outlines[keyPointId].end());

    //!< Delete the outline vector entry alltogether
    assimilable->outlines.erase(assimilable->outlines.begin() + keyPointId);


    //!< Delete each rectangle point from its respective vector
    assimilable->rectangles[keyPointId].erase(
      assimilable->rectangles[keyPointId].begin(),
      assimilable->rectangles[keyPointId].end());

    //!< Delete the rectangle vector entry alltogether
    assimilable->rectangles.erase(assimilable->rectangles.begin() + keyPointId);

    #ifdef DEBUG_TIME
    Timer::tick("assimilateOnce");
    #endif
  }



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
  void HoleMerger::connectUnilaterally(HolesConveyor* connector,
    HolesConveyor* connectable, const PointCloudXYZPtr& pointCloudXYZ)
  {
    //!< Validate the connectable's holes against the connector's ones
    for (int i = 0; i < connector->keyPoints.size(); i++)
    {
      for (int j = connectable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If the connectable is able to be connected with the connectable,
        //!< connect the connector with the connectable,
        //!< replacing the connector and deleting the connectable
        if (isCapableOfConnecting(
            i, *connector, j, *connectable, pointCloudXYZ))
        {
          connectOnce(i, connector, j, connectable);
        }
      }
    }
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
  bool HoleMerger::isCapableOfConnecting(const int& connectorId,
    const HolesConveyor& connector,
    const int& connectableId,
    const HolesConveyor& connectable,
    const PointCloudXYZPtr& pointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfConnecting", "applyMergeOperation");
    #endif

    for (int av = 0; av < connectable.outlines[connectableId].size(); av++)
    {
      if (cv::pointPolygonTest(connector.outlines[connectorId],
          connectable.outlines[connectableId][av], false) >= 0)
      {
        return false;
      }
    }


    //!< The real min distance (in meters) between two points of the
    //!< connector's and connectable's outlines
    double minOutlinesDistance = 10000.0;
    for (int av = 0; av < connectable.outlines[connectableId].size(); av++)
    {
      //!< The connectable's current outline point x,y,z coordinates
      //!< measured by the depth sensor
      float connectableOutlinePointX = pointCloudXYZ->points[
        static_cast<int>(connectable.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(connectable.outlines[connectableId][av].y)].x;

      float connectableOutlinePointY = pointCloudXYZ->points[
        static_cast<int>(connectable.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(connectable.outlines[connectableId][av].y)].y;

      float connectableOutlinePointZ = pointCloudXYZ->points[
        static_cast<int>(connectable.outlines[connectableId][av].x)
        + pointCloudXYZ->width *
        static_cast<int>(connectable.outlines[connectableId][av].y)].z;


      for (int ac = 0; ac < connector.outlines[connectorId].size(); ac++)
      {
        //!< The connector's current outline point x,y,z coordinates
        //!< measured by the depth sensor
        float connectorOutlinePointX = pointCloudXYZ->points[
          static_cast<int>(connector.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(connector.outlines[connectorId][ac].y)].x;

        float connectorOutlinePointY = pointCloudXYZ->points[
          static_cast<int>(connector.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(connector.outlines[connectorId][ac].y)].y;

        float connectorOutlinePointZ = pointCloudXYZ->points[
          static_cast<int>(connector.outlines[connectorId][ac].x)
          + pointCloudXYZ->width *
          static_cast<int>(connector.outlines[connectorId][ac].y)].z;


        //!< The current outline points distance
        float outlinePointsDistance = sqrt(
          pow(connectableOutlinePointX - connectorOutlinePointX, 2) +
          pow(connectableOutlinePointY - connectorOutlinePointY, 2) +
          pow(connectableOutlinePointZ - connectorOutlinePointZ, 2));

        if (outlinePointsDistance < minOutlinesDistance)
        {
          minOutlinesDistance = outlinePointsDistance;
        }
      }
    }
    //!< If not all of connectable's outline points
    //!< are outside the connector's outline,
    //!< or the minimum distance between the connector's and connectable's
    //!< outlines is greater than a distance thrshold,
    //!< this connectable is not a candidate to be connected with
    //!< the connector
    if (minOutlinesDistance > Parameters::connect_holes_min_distance)
    {
      return false;
    }


    //!< Calculate the connector's bounding box area
    float connectorRectangleWidthX =
      connector.rectangles[connectorId][0].x
      - connector.rectangles[connectorId][1].x;
    float connectorRectangleWidthY =
      connector.rectangles[connectorId][0].y
      - connector.rectangles[connectorId][1].y;
    float connectorRectangleHeightX =
      connector.rectangles[connectorId][1].x
      - connector.rectangles[connectorId][2].x;
    float connectorRectangleHeightY =
      connector.rectangles[connectorId][1].y
      - connector.rectangles[connectorId][2].y;

    float connectorBoxArea = sqrt(pow(connectorRectangleWidthX, 2)
      + pow(connectorRectangleWidthY, 2)) *
      sqrt(pow(connectorRectangleHeightX, 2)
        + pow(connectorRectangleHeightY, 2));

    //!< Calculate the connectable's bounding box area
    float connectableRectangleWidthX =
      connectable.rectangles[connectableId][0].x
      - connectable.rectangles[connectableId][1].x;
    float connectableRectangleWidthY =
      connectable.rectangles[connectableId][0].y
      - connectable.rectangles[connectableId][1].y;
    float connectableRectangleHeightX =
      connectable.rectangles[connectableId][1].x
      - connectable.rectangles[connectableId][2].x;
    float connectableRectangleHeightY =
      connectable.rectangles[connectableId][1].y
      - connectable.rectangles[connectableId][2].y;

    float connectableBoxArea = sqrt(pow(connectableRectangleWidthX, 2) +
      pow(connectableRectangleWidthY, 2)) *
      sqrt(pow(connectableRectangleHeightX, 2)
        + pow(connectableRectangleHeightY, 2));

    #ifdef DEBUG_TIME
    Timer::tick("isCapableOfConnecting");
    #endif

    //!< If the connectable's area is smaller than the connector's,
    //!< this connectable is capable of being connected with the connector
    return (connectableBoxArea < connectorBoxArea);
  }



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
  void HoleMerger::connectOnce(const int& connectorId,
    HolesConveyor* connector,
    const int& connectableId,
    HolesConveyor* connectable)
  {
    #ifdef DEBUG_TIME
    Timer::start("connectOnce", "applyMergeOperation");
    #endif

    //!< The connector's outline will be the sum of the outline points
    //!< of the two conveyors
    connector->outlines[connectorId].insert(
      connector->outlines[connectorId].end(),
      connectable->outlines[connectableId].begin(),
      connectable->outlines[connectableId].end());


    //!< The connectable's new least area rotated bounding box will be the
    //!< one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(connector->outlines[connectorId]);

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
    connector->rectangles[connectorId].erase(
      connector->rectangles[connectorId].begin(),
      connector->rectangles[connectorId].end());

    //!< Replace the connector's vertices with the new vertices
    connector->rectangles.at(connectorId) = substituteVerticesVector;


    //!< Set the overall candidate hole's keypoint to the center of the
    //!< newly created bounding rectangle
    float x = 0;
    float y = 0;
    for (int k = 0; k < 4; k++)
    {
      x += connector->rectangles[connectorId][k].x;
      y += connector->rectangles[connectorId][k].y;
    }

    connector->keyPoints[connectorId].pt.x = x / 4;
    connector->keyPoints[connectorId].pt.y = y / 4;


    //!< The connectable has now been merged with the connector,
    //!< so delete it. So long, connectable

    //!< Delete the keypoint from the conveyor
    connectable->keyPoints.erase(
      connectable->keyPoints.begin() + connectableId);


    //!< Delete each outline point from its respective vector
    connectable->outlines[connectableId].erase(
      connectable->outlines[connectableId].begin(),
      connectable->outlines[connectableId].end());

    //!< Delete the outline vector entry alltogether
    connectable->outlines.erase(
      connectable->outlines.begin() + connectableId);


    //!< Delete each rectangle point from its respective vector
    connectable->rectangles[connectableId].erase(
      connectable->rectangles[connectableId].begin(),
      connectable->rectangles[connectableId].end());

    //!< Delete the rectangle vector entry alltogether
    connectable->rectangles.erase(
      connectable->rectangles.begin() + connectableId);

    #ifdef DEBUG_TIME
    Timer::tick("connectOnce");
    #endif
  }



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
  void HoleMerger::amalgamateBilaterally(
    HolesConveyor* depthHolesConveyor,
    HolesConveyor* rgbHolesConveyor)
  {
    //!< Merging holes has a meaning only if both nodes have published
    //!< candidate holes
    if (depthHolesConveyor->keyPoints.size() > 0 &&
      rgbHolesConveyor->keyPoints.size() > 0)
    {
      amalgamateUnilaterally(rgbHolesConveyor, depthHolesConveyor);

      amalgamateUnilaterally(depthHolesConveyor, rgbHolesConveyor);
    }
  }



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
  void HoleMerger::amalgamateUnilaterally(
    HolesConveyor* amalgamator,
    HolesConveyor* amalgamatable)
  {
    //!< Validate the amalgamatable's holes against the amalgamator's ones
    for (int i = 0; i < amalgamator->keyPoints.size(); i++)
    {
      for (int j = amalgamatable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If the amalgamatable's area is smaller than the amalgamator's,
        //!< merge the amalgamator with the amalgamatable into theamalgamator
        //!< and delete the amalgamatable
        if (isCapableOfAmalgamating(i, *amalgamator, j, *amalgamatable))
        {
          amalgamateOnce(i, amalgamator, j, amalgamatable);
        }
      }
    }
  }



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
    that acts as the amalgamatable inside the amalgamatable HolesConveyor
    @param[in] amalgamatable [const HolesConveyor&] The HolesConveyor that
    acts as the amalgamatables struct
    @return [bool] True if the amalgamator is capable of amalgamating
    the amalgamatable
   **/
  bool HoleMerger::isCapableOfAmalgamating(
    const int& amalgamatorId,
    const HolesConveyor& amalgamator,
    const int& amalgamatableId,
    const HolesConveyor& amalgamatable)
  {
    #ifdef DEBUG_TIME
    Timer::start("isCapableOfAmalgamating", "applyMergeOperation");
    #endif

    //!< Are all the assimilable's outline points inside
    //!< the assimilator's bounding box? If not all but some, continue
    int numAmalgamatableOutlinePointsInAmalgamator = 0;
    for (int av = 0; av < amalgamatable.outlines[amalgamatableId].size(); av++)
    {
      if (cv::pointPolygonTest(amalgamator.outlines[amalgamatorId],
          amalgamatable.outlines[amalgamatableId][av], false) >= 0)
      {
        numAmalgamatableOutlinePointsInAmalgamator++;
      }
    }


    //!< If zero or all of amalgamatable's outline points
    //!< are inside the amalgamator's outline, this amalgamatable is
    //!< not a candidate to be amalgamated by the amalgamator
    if (numAmalgamatableOutlinePointsInAmalgamator == 0 ||
      numAmalgamatableOutlinePointsInAmalgamator ==
      amalgamatable.outlines[amalgamatableId].size())
    {
      return false;
    }


    //!< Calculate the amalgamator's bounding box area
    float amalgamatorRectangleWidthX =
      amalgamator.rectangles[amalgamatorId][0].x
      - amalgamator.rectangles[amalgamatorId][1].x;
    float amalgamatorRectangleWidthY =
      amalgamator.rectangles[amalgamatorId][0].y
      - amalgamator.rectangles[amalgamatorId][1].y;

    float amalgamatorRectangleHeightX =
      amalgamator.rectangles[amalgamatorId][1].x
      - amalgamator.rectangles[amalgamatorId][2].x;
    float amalgamatorRectangleHeightY =
      amalgamator.rectangles[amalgamatorId][1].y
      - amalgamator.rectangles[amalgamatorId][2].y;

    float amalgamatorBoxArea = sqrt(pow(amalgamatorRectangleWidthX, 2)
      + pow(amalgamatorRectangleWidthY, 2)) *
      sqrt(pow(amalgamatorRectangleHeightX, 2)
        + pow(amalgamatorRectangleHeightY, 2));


    //!< Calculate the amalgamatable's bounding box area
    float amalgamatableRectangleWidthX =
      amalgamatable.rectangles[amalgamatableId][0].x
      - amalgamatable.rectangles[amalgamatableId][1].x;
    float amalgamatableRectangleWidthY =
      amalgamatable.rectangles[amalgamatableId][0].y
      - amalgamatable.rectangles[amalgamatableId][1].y;

    float amalgamatableRectangleHeightX =
      amalgamatable.rectangles[amalgamatableId][1].x
      - amalgamatable.rectangles[amalgamatableId][2].x;
    float amalgamatableRectangleHeightY =
      amalgamatable.rectangles[amalgamatableId][1].y
      - amalgamatable.rectangles[amalgamatableId][2].y;

    float amalgamatableBoxArea = sqrt(pow(amalgamatableRectangleWidthX, 2) +
      pow(amalgamatableRectangleWidthY, 2)) *
      sqrt(pow(amalgamatableRectangleHeightX, 2)
        + pow(amalgamatableRectangleHeightY, 2));

    #ifdef DEBUG_TIME
    Timer::tick("isCapableOfAmalgamating");
    #endif

    //!< If the amalgatamable's area is smaller than the assimilator's,
    //!< this amalgamator is capable of amalgamating the amalgamatable
    return (amalgamatableBoxArea < amalgamatorBoxArea);
  }



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
    @param[in] amalgamatableId [const int&] The identifier of the hole inside
    the HolesConveyor amalgamatable struct
    @param[in][out] amalgamatable [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be deleted
    @return void
   **/
  void HoleMerger::amalgamateOnce(const int& amalgamatorId,
    HolesConveyor* amalgamator,
    const int& amalgamatableId,
    HolesConveyor* amalgamatable)
  {
    #ifdef DEBUG_TIME
    Timer::start("amalgamateOnce", "applyMergeOperation");
    #endif

    //!< Viewing the two outlines as sets,
    //!< the final outline should not have the intersection
    //!< of the two sets.
    std::vector<cv::Point2f> amalgamatorOutline;
    std::vector<cv::Point2f> amalgamatableOutline;

    //!< Add all the amalgamatable's outline points that are not located
    //!< inside the amalgamator's outline, to the amalgamatableOutline
    //!< vector
    for (int o = 0; o < amalgamatable->outlines[amalgamatableId].size(); o++)
    {
      if (pointPolygonTest(amalgamator->outlines[amalgamatorId],
          amalgamatable->outlines[amalgamatableId][o], false) < 0)
      {
        amalgamatableOutline.push_back(
          amalgamatable->outlines[amalgamatableId][o]);
      }
    }

    //!< Add all the amalgamator's outline points that are not located
    //!< inside the amalgamatable's outline, to the amalgamatorOutline
    //!< vector
    for (int o = 0; o < amalgamator->outlines[amalgamatorId].size(); o++)
    {
      if (pointPolygonTest(amalgamatable->outlines[amalgamatableId],
          amalgamator->outlines[amalgamatorId][o], false) < 0)
      {
        amalgamatorOutline.push_back(amalgamator->outlines[amalgamatorId][o]);
      }
    }

    //!< The amalgamator's outline will be the sum of the outline points
    //!< that are not located inside each other
    amalgamator->outlines[amalgamatorId] = amalgamatorOutline;
    amalgamator->outlines[amalgamatorId].insert(
      amalgamator->outlines[amalgamatorId].end(),
      amalgamatableOutline.begin(), amalgamatableOutline.end());


    //!< The amalgamator's new least area rotated bounding box will be the
    //!< one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      cv::minAreaRect(amalgamator->outlines[amalgamatorId]);

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
    amalgamator->rectangles[amalgamatorId].erase(
      amalgamator->rectangles[amalgamatorId].begin(),
      amalgamator->rectangles[amalgamatorId].end());

    amalgamator->rectangles.at(amalgamatorId) = substituteVerticesVector;


    //!< Set the overall candidate hole's keypoint to the center of the
    //!< newly created bounding rectangle
    float x = 0;
    float y = 0;
    for (int k = 0; k < 4; k++)
    {
      x += amalgamator->rectangles[amalgamatorId][k].x;
      y += amalgamator->rectangles[amalgamatorId][k].y;
    }

    amalgamator->keyPoints[amalgamatorId].pt.x = x / 4;
    amalgamator->keyPoints[amalgamatorId].pt.y = y / 4;


    //!< The amalgamatable has now been merged with the amalgamator,
    //!< so delete it. So long, amalgamatable

    //!< Delete the keypoint from the conveyor
    amalgamatable->keyPoints.erase(
      amalgamatable->keyPoints.begin() + amalgamatableId);


    //!< Delete each outline point from its respective vector
    amalgamatable->outlines[amalgamatableId].erase(
      amalgamatable->outlines[amalgamatableId].begin(),
      amalgamatable->outlines[amalgamatableId].end());

    //!< Delete the outline vector entry alltogether
    amalgamatable->outlines.erase(
      amalgamatable->outlines.begin() + amalgamatableId);


    //!< Delete each rectangle point from its respective vector
    amalgamatable->rectangles[amalgamatableId].erase(
      amalgamatable->rectangles[amalgamatableId].begin(),
      amalgamatable->rectangles[amalgamatableId].end());

    //!< Delete the rectangle vector entry alltogether
    amalgamatable->rectangles.erase(
      amalgamatable->rectangles.begin() + amalgamatableId);

    #ifdef DEBUG_TIME
    Timer::tick("amalgamateOnce");
    #endif
  }

} // namespace pandora_vision
