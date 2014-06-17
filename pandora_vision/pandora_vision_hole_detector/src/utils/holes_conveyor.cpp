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

#include "utils/holes_conveyor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Appends one HolesConveyor struct to another.
    @param[in] src [const HolesConveyor&] The source conveyor
    @param[out] dst [HolesConveyor*] The destination conveyor
    @return void
   **/
  void HolesConveyorUtils::append(const HolesConveyor& src, HolesConveyor* dst)
  {
    for (int i = 0; i < src.size(); i++)
    {
      dst->holes.push_back(src.holes[i]);
    }
  }



  /**
    @brief Appends a dummy HolesConveyor to a HoleConveyor struct
    @param[in] rectangleUpperLeft [const cv::Point2f&] The upper left
    vertex of the bounding rectangle
    @param[in] outlineUpperLeft [const cv::Point2f] The upper left
    vertex of the hole's outline
    @param[in] rx [const int&] The width of the rectangle
    @param[in] ry [const int&] The height of the rectangle
    @param[in] ox [const int&] The width of the outline rectangle
    @param[in] ry [const int&] The height of the outline rectangle
    @param[in,out] conveyor [HolesConveyor*] The conveyor to which the
    dummy HolesConveyor will be appended
    @return void
   **/
  void HolesConveyorUtils::appendDummyConveyor(
    const cv::Point2f& rectangleUpperLeft,
    const cv::Point2f& outlineUpperLeft,
    const int& rx, const int& ry,
    const int& ox, const int& oy,
    HolesConveyor* conveyor)
  {
    // A conveyor of a single hole
    HoleConveyor hole;

    // Assign the keypoint
    cv::KeyPoint k(outlineUpperLeft.x + ox / 2,
      outlineUpperLeft.y + oy / 2, 1);

    hole.keypoint = k;

    // Assign the rectangle points
    hole.rectangle =
      generateRectangle(rectangleUpperLeft, rx, ry, 0);

    // Assign the outline points
    hole.outline =
      generateRectangle(outlineUpperLeft, ox, oy, 1);

    // Append hole into conveyor
    conveyor->holes.push_back(hole);
  }



  /**
    @brief Hollows a HolesConveyor struct, deleting every entry in it
    @param[in,out] conveyor [HolesConveyor*] The conveyor struct that will
    be cleared
    @return void
   **/
  void HolesConveyorUtils::clear(HolesConveyor* conveyor)
  {
    conveyor->holes.clear();
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
    // If the dst is not empty, clear it
    if (dst->size() > 0)
    {
      clear(dst);
    }

    // Append the src to the dst
    append(src, dst);
  }



  /**
    @brief Generates a vector of cv::Point2f that represents the 4 vertices
    of a rectangle
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex point
    of the rectangle
    @param[in] x [const int&] The length at x direction
    @param[in] y [const int&] The length at y direction
    @param[in] intent [const int&] 0 for vertices'
    construction, 1 for a coherent outline construction
    @return std::vector<cv::Point2f> A vector of four vertices
    of type cv::Point2f
   **/
  std::vector<cv::Point2f> HolesConveyorUtils::generateRectangle(
    const cv::Point2f& upperLeft, const int& x, const int& y,
    const int& intent)
  {
    // The vector of the rectangle's vertices
    std::vector<cv::Point2f> rectangleVertices;

    // The four vertices of the rectangle
    cv::Point2f vertex_1(upperLeft.x, upperLeft.y);
    cv::Point2f vertex_2(upperLeft.x, upperLeft.y + y);
    cv::Point2f vertex_3(upperLeft.x + x, upperLeft.y + y);
    cv::Point2f vertex_4(upperLeft.x + x, upperLeft.y);

    // Push them back into the vector
    rectangleVertices.push_back(vertex_1);
    rectangleVertices.push_back(vertex_2);
    rectangleVertices.push_back(vertex_3);
    rectangleVertices.push_back(vertex_4);

    // Outline construction
    if (intent == 1)
    {
      rectangleVertices.clear();

      cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

      cv::Mat canvas = cv::Mat::zeros(480, 640, CV_8UC1);

      for(unsigned int j = 0; j < 4; j++)
      {
        cv::line(canvas, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
      }

      for (int i = 0; i < 480; i++)
      {
        for (int j = 0; j < 640; j++)
        {
          if (canvas.at<unsigned char>(i, j) != 0)
          {
            rectangleVertices.push_back(cv::Point2f(j, i));
          }
        }
      }
      return rectangleVertices;
    }
    return rectangleVertices;
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
    // The conveyor that will be returned
    HolesConveyor temp;

    // Push back the index-th hole of the conveyor into temp
    temp.holes.push_back(conveyor.holes[index]);

    return temp;
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
    // Clear the destination conveyor if not empty
    if (dst->size() > 0)
    {
      clear(dst);
    }

    // Append the first source to dst
    append(srcA, dst);

    // Append the second source to dst
    append(srcB, dst);
  }



  /**
    @brief Deletes a hole from HolesConveyor struct,
    @param[in,out] conveyor [HolesConveyor*] The conveyor struct from which
    the hole will be removed
    @param[in] id [const int&] The index of the hole in the conveyor
    @return void
   **/
  void HolesConveyorUtils::removeHole(HolesConveyor* conveyor, const int& id)
  {
    // Delete the respective keypoint
    conveyor->holes.erase(conveyor->holes.begin() + id);

  }



  /**
    @brief Replaces an entire HolesConveyor struct with another
    @param[in] src [const HolesConveyor&] The source conveyor struct
    @param[out] dst [HolesConveyor*] The destination conveyor struct
    @return void
   **/
  void HolesConveyorUtils::replace(const HolesConveyor& src, HolesConveyor* dst)
  {
    // Clear the dst
    clear(dst);

    // Fill it with the src
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
    // Replace the dst's dstIndex-th hole's keypoint
    dst->holes[dstIndex].keypoint = src.holes[srcIndex].keypoint;

    // Replace the dst's dstIndex-th hole's rectangle points
    dst->holes[dstIndex].rectangle = src.holes[srcIndex].rectangle;

    // Replace the dst's dstIndex-th hole's outline points
    dst->holes[dstIndex].outline = src.holes[srcIndex].outline;
  }



  /**
    @brief Shuffles the contents of a HolesConveyor
    @param[in,out] src [HolesConveyor*] The conveyor
    @return void
   **/
  void HolesConveyorUtils::shuffle(HolesConveyor* src)
  {
    // Keep the original holes' arrangement
    HolesConveyor temp;
    copyTo(*src, &temp);

    // Hollow-out the src
    clear(src);

    // The vector of holes' indices
    std::vector<int> indices;
    for (int i = 0; i < temp.size(); i++)
    {
      indices.push_back(i);
    }

    // Shuffle the indices
    std::random_shuffle(indices.begin(), indices.end());

    // Fill the src conveyor with the shuffled holes
    for (int i = 0; i < temp.size(); i++)
    {
      append(getHole(temp, indices[i]), src);
    }
  }

} // namespace pandora_vision
