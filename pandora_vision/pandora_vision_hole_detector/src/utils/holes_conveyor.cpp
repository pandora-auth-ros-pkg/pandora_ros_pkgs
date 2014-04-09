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

#include "utils/holes_conveyor.h"

namespace pandora_vision
{
  /**
    @brief Appends one HolesConveyor struct to another.
    @param[in] src [const HolesConveyor&] The source struct
    @param[out] dst [HolesConveyor*] The destination struct
    @return void
   **/
  void HolesConveyorUtils::append(const HolesConveyor& src, HolesConveyor* dst)
  {
    for (int i = 0; i < src.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(src.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < src.outlines[i].size(); j++)
      {
        tempOutline.push_back(src.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < src.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(src.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }
  }



  /**
    @brief Hollows a HolesConveyor struct, deleting every entry in it
    @param[in][out] conveyor [HolesConveyor*] The conveyor struct that will
    be cleared
    @return void
   **/
  void HolesConveyorUtils::clear(HolesConveyor* conveyor)
  {
    //!< Delete the keypoints
    conveyor->keyPoints.erase(conveyor->keyPoints.begin(),
      conveyor->keyPoints.end());


    //!< Delete each outline point from its respective vector
    for (int i = 0; i < conveyor->outlines.size(); i++)
    {
      conveyor->outlines[i].erase(conveyor->outlines[i].begin(),
        conveyor->outlines[i].end());
    }

    //!< Delete the outline vector alltogether
    conveyor->outlines.erase(conveyor->outlines.begin(),
      conveyor->outlines.end());


    //!< Delete each outline point from its respective vector
    for (int i = 0; i < conveyor->rectangles.size(); i++)
    {
      conveyor->rectangles[i].erase(conveyor->rectangles[i].begin(),
        conveyor->rectangles[i].end());
    }

    //!< Delete the rectangles vector alltogether
    conveyor->rectangles.erase(conveyor->rectangles.begin(),
      conveyor->rectangles.end());
  }



  /**
    @brief Copies one HolesConveyor struct to another. If the dst conveyor
    is not empty, it empties it first, and then copies the src to the dst
    @param[in] src [const HolesConveyor&] The source struct
    @param[out] dst [HolesConveyor*] The destination struct
    @return void
   **/
  void HolesConveyorUtils::copyTo(const HolesConveyor& src,
    HolesConveyor* dst)
  {
    //!< If the dst is not empty, clear it
    if (dst->keyPoints.size() > 0)
    {
      clear(dst);
    }

    //!< Append the src to the dst
    append(src, dst);
  }



  /**
    @brief Extracts the specified hole from a HolesConveyor into a new
    HolesConveyor struct that is returned
    @param[in] conveyor [const HolesConveyor&] The HolesConveyor struct
    @param[in] index [const int&] The index of the hole inside the conveyor
    @return A HolesConveyor struct that containes the index-th hole of the
    conveyor
   **/
  HolesConveyor HolesConveyorUtils::getHole(const HolesConveyor& conveyor,
    const int& index)
  {
    HolesConveyor dst;

    //!< Get the index-th hole's keypoint
    dst.keyPoints.push_back(conveyor.keyPoints[index]);

    //!< Get the index-th hole's outline
    std::vector<cv::Point> tempOutline;
    for (int j = 0; j < conveyor.outlines[index].size(); j++)
    {
      tempOutline.push_back(conveyor.outlines[index][j]);
    }
    dst.outlines.push_back(tempOutline);

    //!< Get the index-th hole's rectangle
    std::vector<cv::Point2f> tempRectangle;
    for (int j = 0; j < conveyor.rectangles[index].size(); j++)
    {
      tempRectangle.push_back(conveyor.rectangles[index][j]);
    }
    dst.rectangles.push_back(tempRectangle);

    return dst;
  }



  /**
    @brief Given two sources of struct HolesConveyor, this function
    merge them into one struct.
    @param[in] srcA [const HolesConveyor&] The first HolesConveyor source
    @param[in] srcB [const HolesConveyor&] The second HolesConveyor source
    @param[out] dst [HolesConveyor*] The final struct
   **/
  void HolesConveyorUtils::merge(const HolesConveyor& srcA,
    const HolesConveyor& srcB, HolesConveyor* dst)
  {
    //!< Insert the srcA HolesConveyor into the dst HolesConveyor
    for (int i = 0; i < srcA.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(srcA.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < srcA.outlines[i].size(); j++)
      {
        tempOutline.push_back(srcA.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < srcA.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(srcA.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }

    //!< Insert the srcB HolesConveyor into the dst HolesConveyor
    for (int i = 0; i < srcB.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(srcB.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < srcB.outlines[i].size(); j++)
      {
        tempOutline.push_back(srcB.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < srcB.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(srcB.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }
  }



  /**
    @brief Prints data pertaining to the contents of a HolesConveyor struct,
    that is, the keypoints, rectangle points and outline points of the
    holes it contains
    @param[in] conveyor [const HolesConveyor&] The conveyor
    @param[in] id [const int&] The identifier of a specific hole
    @return void
   **/
  void HolesConveyorUtils::print(const HolesConveyor& conveyor,
    const int& id)
  {
    if (id < 0)
    {
      ROS_INFO("Conveyor has %zu holes :", conveyor.keyPoints.size());
      for (int i = 0; i < conveyor.keyPoints.size(); i++)
      {
        ROS_INFO("---------------");
        ROS_INFO("Hole #%d :", i);

        ROS_INFO("KeyPoint: [%f %f]",
          conveyor.keyPoints[i].pt.x, conveyor.keyPoints[i].pt.y);

        ROS_INFO("Rectangle points:");
        for (int j = 0; j < conveyor.rectangles[i].size(); j++)
        {
          ROS_INFO("[%f %f]",
            conveyor.rectangles[i][j].x, conveyor.rectangles[i][j].y);
        }

        ROS_INFO("Outline points:");
        for (int j = 0; j < conveyor.outlines[i].size(); j++)
        {
          ROS_INFO("[%d %d]",
            conveyor.outlines[i][j].x, conveyor.outlines[i][j].y);
        }
      }
    }
    else
    {
      ROS_INFO("---------------");
      ROS_INFO("Hole #%d :", id);

      ROS_INFO("KeyPoint: [%f %f]",
        conveyor.keyPoints[id].pt.x, conveyor.keyPoints[id].pt.y);

      ROS_INFO("Rectangle points:");
      for (int j = 0; j < conveyor.rectangles[id].size(); j++)
      {
        ROS_INFO("[%f %f]",
          conveyor.rectangles[id][j].x, conveyor.rectangles[id][j].y);
      }

      ROS_INFO("Outline points:");
      for (int j = 0; j < conveyor.outlines[id].size(); j++)
      {
        ROS_INFO("[%d %d]",
          conveyor.outlines[id][j].x, conveyor.outlines[id][j].y);
      }
    }
  }



  /**
    @brief Deletes a hole from HolesConveyor struct,
    @param[in][out] conveyor [HolesConveyor*] The conveyor struct from which
    the hole will be removed
    @param[in] id [const int&] The index of the hole in the conveyor
    @return void
   **/
  void HolesConveyorUtils::removeHole(HolesConveyor* conveyor, const int& id)
  {
    //!< Delete the respective keypoint
    conveyor->keyPoints.erase(conveyor->keyPoints.begin() + id);


    //!< Delete each outline point from its respective vector
    conveyor->outlines[id].erase(conveyor->outlines[id].begin(),
      conveyor->outlines[id].end());

    //!< Delete the respective outline vector
    conveyor->outlines.erase(conveyor->outlines.begin() + id);


    //!< Delete each outline point from its respective vector
    conveyor->rectangles[id].erase(conveyor->rectangles[id].begin(),
      conveyor->rectangles[id].end());

    //!< Delete the respective rectangles vector
    conveyor->rectangles.erase(conveyor->rectangles.begin() + id);
  }



  /**
    @brief Replaces an entire HolesConveyor struct with another
    @param[in] src [const HolesConveyor&] The source conveyor struct
    @param[out] dst [HolesConveyor*] The destination conveyor struct
    @return void
   **/
  void HolesConveyorUtils::replace(const HolesConveyor& src, HolesConveyor* dst)
  {
    //!< Clear the dst
    clear(dst);

    //!< Fill it with the src
    copyTo(src, dst);
  }



  /**
    @brief Replaces a specified hole from a HolesConveyor dst struct
    with the hole of index srcIndex of the src HolesConveyor struct entry
    @param[in] src [const HolesConveyor&] The HolesConveyor source struct
    @param[in] srcIndex [const int&] The index of the hole inside the
    src conveyor that will be copied into the dst HolesConveyor struct,
    in the dstIndex position
    @param[out] dst [HolesConveyor*] The HolesConveyor destination struct
    @param[in] dstIndex [const int&] The index of the hole inside the
    dst conveyor that will be replaced by the srcIndex-th of the src
    HolesConveyor struct
    @return void
   **/
  void HolesConveyorUtils::replaceHole(const HolesConveyor& src,
    const int& srcIndex, HolesConveyor* dst, const int& dstIndex)
  {
    //!< Replace the dst's dstIndex-th hole's keypoint
    dst->keyPoints.at(dstIndex) = src.keyPoints.at(srcIndex);

    //!< Erase the outline points for entry dstIndex of the dst
    dst->outlines[dstIndex].erase(
      dst->outlines[dstIndex].begin(),
      dst->outlines[dstIndex].end());

    //!< Replace the dst's dstIndex-th hole's outline points
    dst->outlines.at(dstIndex) = src.outlines[srcIndex];

    //!< Erase the rectangle points for entry dstIndex of the dst
    dst->rectangles[dstIndex].erase(
      dst->rectangles[dstIndex].begin(),
      dst->rectangles[dstIndex].end());

    //!< Replace the dst's dstIndex-th hole's rectangle points
    dst->rectangles.at(dstIndex) = src.rectangles[srcIndex];
  }



  /**
    @brief Shuffles the contents of a HolesConveyor
    @param[in][out] src [HolesConveyor*] The conveyor
    @return void
   **/
  void HolesConveyorUtils::shuffle(HolesConveyor* src)
  {
    //!< Keep the original holes' arrangement
    HolesConveyor temp;
    copyTo(*src, &temp);

    //!< Hollow-out the src
    clear(src);

    //!< The vector of holes' indices
    std::vector<int> indices;
    for (int i = 0; i < temp.keyPoints.size(); i++)
    {
      indices.push_back(i);
    }

    //!< Shuffle the indices
    std::random_shuffle(indices.begin(), indices.end());

    //!< Fill the src conveyor with the shuffled holes
    for (int i = 0; i < temp.keyPoints.size(); i++)
    {
      append(getHole(temp, indices[i]), src);
    }
  }



  /**
    @brief Gets the number of holes in a HolesConveyor
    @param[in] conveyor [const HolesConveyor&] The HolesConveyor struct
    @return int The size of @param conveyor
   **/
  int HolesConveyorUtils::size(const HolesConveyor& conveyor)
  {
    return conveyor.keyPoints.size();
  }



  /**
    @brief Appends a dummy HolesConveyor to a HoleConveyor struct
    @param[in] rectangleUpperLeft [const cv::Point2f&] The upper left
    vertex of the bounding rectangle
    @param[in] outlineUpperLeft [const cv::Point] The upper left
    vertex of the hole's outline
    @param[in] rx [const int&] The width of the rectangle
    @param[in] ry [const int&] The height of the rectangle
    @param[in] ox [const int&] The width of the outline rectangle
    @param[in] ry [const int&] The height of the outline rectangle
    @param[in][out] conveyor [HolesConveyor*] The conveyor to which the
    dummy HolesConveyor will be appended
    @return void
   **/
  void HolesConveyorUtils::appendDummyConveyor(
    const cv::Point2f& rectangleUpperLeft,
    const cv::Point& outlineUpperLeft,
    const int& rx, const int& ry,
    const int& ox, const int& oy,
    HolesConveyor* conveyor)
  {
    conveyor->rectangles.push_back(
      generateRectangle(rectangleUpperLeft, rx, ry));

    conveyor->outlines.push_back(
      generateRectangle(outlineUpperLeft, ox, oy));

    cv::KeyPoint k(outlineUpperLeft.x + ox / 2,
      outlineUpperLeft.y + oy / 2, 1);

    conveyor->keyPoints.push_back(k);
  }



  /**
    @brief Generates a vector of cv::Point2f that represents the 4 vertices
    of a rectangle
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex point
    of the rectangle
    @param[in] x [const int&] The length at x direction
    @param[in] y [const int&] The length at y direction
    @return std::vector<cv::Point2f> A vector of four vertices
    of type cv::Point2f
   **/
  std::vector<cv::Point2f> HolesConveyorUtils::generateRectangle(
    const cv::Point2f& upperLeft, const int& x, const int& y)
  {
    std::vector<cv::Point2f> rectangleVertices;

    cv::Point2f vertex_1(upperLeft.x, upperLeft.y);
    rectangleVertices.push_back(vertex_1);

    cv::Point2f vertex_2(upperLeft.x, upperLeft.y + y);
    rectangleVertices.push_back(vertex_2);

    cv::Point2f vertex_3(upperLeft.x + x, upperLeft.y + y);
    rectangleVertices.push_back(vertex_3);

    cv::Point2f vertex_4(upperLeft.x + x, upperLeft.y);
    rectangleVertices.push_back(vertex_4);

    return rectangleVertices;
  }



  /**
    @brief Generates a vector of cv::Point that represents the 4 vertices
    of a rectangle
    @param[in] upperLeft [const cv::Point&] The upper left vertex point
    of the rectangle
    @param[in] x [const int&] The length at x direction
    @param[in] y [const int&] The length at y direction
    @return std::vector<cv::Point> A vector of four vertices
    of type cv::Point
   **/
  std::vector<cv::Point> HolesConveyorUtils::generateRectangle(
    const cv::Point& upperLeft, const int& x, const int& y)
  {
    std::vector<cv::Point> rectangleVertices;

    cv::Point vertex_1(upperLeft.x, upperLeft.y);
    rectangleVertices.push_back(vertex_1);

    cv::Point vertex_2(upperLeft.x, upperLeft.y + y);
    rectangleVertices.push_back(vertex_2);

    cv::Point vertex_3(upperLeft.x + x, upperLeft.y + y);
    rectangleVertices.push_back(vertex_3);

    cv::Point vertex_4(upperLeft.x + x, upperLeft.y);
    rectangleVertices.push_back(vertex_4);

    return rectangleVertices;
  }


  /**
    @brief Draws the keypoints, rectangles and outlines of holes
    @param[in] coneyor [const HolesConveyor&] The conveyor whose holes
    will be drawn on @param img
    @param[in][out] img [cv::Mat*] The image drawn
    @return void
   **/
  void HolesConveyorUtils::draw(const HolesConveyor& conveyor,
    cv::Mat* img)
  {
    cv::drawKeypoints(*img, conveyor.keyPoints, *img, CV_RGB(0, 255, 0),
      cv::DrawMatchesFlags::DEFAULT);

    for(unsigned int i = 0; i < conveyor.outlines.size(); i++)
    {
      for(unsigned int j = 0; j < conveyor.outlines[i].size(); j++)
      {
        cv::line(*img,
          cvPoint(conveyor.outlines[i][j].x - 1, conveyor.outlines[i][j].y - 1),
          cvPoint(conveyor.outlines[i][j].x + 1, conveyor.outlines[i][j].y + 1),
          cv::Scalar(0, 255, 0), 1, 8 , CV_AA);

        cv::line(*img,
          cvPoint(conveyor.outlines[i][j].x - 1, conveyor.outlines[i][j].y + 1),
          cvPoint(conveyor.outlines[i][j].x + 1, conveyor.outlines[i][j].y - 1),
          cv::Scalar(0, 255, 0), 1, 8);
      }
    }

    for (int i = 0; i < conveyor.rectangles.size(); i++)
    {
      for(int j = 0; j < 4; j++)
      {
        cv::line(*img, conveyor.rectangles[i][j],
          conveyor.rectangles[i][(j + 1) % 4], CV_RGB(255, 0, 0), 1, 8);
      }
    }

  }

} // namespace pandora_vision
