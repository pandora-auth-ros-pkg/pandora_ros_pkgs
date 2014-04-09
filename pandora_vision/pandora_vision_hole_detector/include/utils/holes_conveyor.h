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

#ifndef UTILS_HOLES_CONVEYOR_H
#define UTILS_HOLES_CONVEYOR_H

#include "utils/defines.h"

namespace pandora_vision
{
  /**
    @brief The structure that represents holes found.
    @param keyPoints [std::vector<cv::KeyPoint>] The vector of the
    holes' keypoints
    @param rectangles [std::vector< std::vector<cv::Point2f> >] The
    vector of the holes' rotated bounding boxes vertices
    @param outlines [std::vector<std::vector<cv::Point> >] The
    vector of the holes' outlines
   **/
  struct HolesConveyor
  {
    std::vector<cv::KeyPoint> keyPoints;
    std::vector< std::vector<cv::Point2f> > rectangles;
    std::vector<std::vector<cv::Point> > outlines;
  };


  /**
    @Class HoleConveyor
    @brief Provides methods pertinent to the HolesConveyor struct
   **/
  class HolesConveyorUtils
  {
    public:

      /**
        @brief Appends one HolesConveyor struct to another.
        @param[in] src [const HolesConveyor&] The source struct
        @param[out] dst [HolesConveyor*] The destination struct
        @return void
       **/
      static void append(const HolesConveyor& src, HolesConveyor* dst);

      /**
        @brief Hollows a HolesConveyor struct, deleting every entry in it
        @param[in][out] conveyor [HolesConveyor*] The conveyor struct that will
        be cleared
        @return void
       **/
      static void clear(HolesConveyor* conveyor);

      /**
        @brief Copies one HolesConveyor struct to another. If the dst conveyor
        is not empty, it empties it first, and then copies the src to the dst
        @param[in] src [const HolesConveyor&] The source struct
        @param[out] dst [HolesConveyor*] The destination struct
        @return void
       **/
      static void copyTo(const HolesConveyor& src, HolesConveyor* dst);

      /**
        @brief Extracts the specified hole from a HolesConveyor into a new
        HolesConveyor struct that is returned
        @param[in] conveyor [const HolesConveyor&] The HolesConveyor struct
        @param[in] index [const int&] The index of the hole inside the conveyor
        @return A HolesConveyor struct that containes the index-th hole of the
        conveyor
       **/
      static HolesConveyor getHole(const HolesConveyor& conveyor,
        const int& index);

      /**
        @brief Given two sources of struct HolesConveyor, this function
        merge them into one struct.
        @param[in] srcA [const HolesConveyor&] The first HolesConveyor source
        @param[in] srcB [const HolesConveyor&] The second HolesConveyor source
        @param[out] dst [HolesConveyor*] The final struct
       **/
      static void merge(const HolesConveyor& srcA,
        const HolesConveyor& srcB, HolesConveyor* dst);

      /**
        @brief Prints data pertaining to the contents of a HolesConveyor struct,
        that is, the keypoints, rectangle points and outline points of the
        holes it contains
        @param[in] conveyor [const HolesConveyor&] The conveyor
        @param[in] id [const int&] The identifier of a specific hole
        @return void
       **/
      static void print(const HolesConveyor& conveyor, const int& id = -1);

      /**
        @brief Replaces an entire HolesConveyor struct with another
        @param[in] src [const HolesConveyor&] The source conveyor struct
        @param[out] dst [HolesConveyor*] The destination conveyor struct
        @return void
       **/
      static void replace(const HolesConveyor& src, HolesConveyor* dst);

      /**
        @brief Deletes a hole from HolesConveyor struct,
        @param[in][out] conveyor [HolesConveyor*] The conveyor struct from which
        the hole will be removed
        @param[in] id [const int&] The index of the hole in the conveyor
        @return void
       **/
      static void removeHole(HolesConveyor* conveyor, const int& id);

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
      static void replaceHole(const HolesConveyor& src,
        const int& srcIndex, HolesConveyor* dst, const int& dstIndex);

      /**
        @brief Shuffles the contents of a HolesConveyor
        @param[in][out] src [HolesConveyor*] The conveyor
        @return void
       **/
      static void shuffle(HolesConveyor* src);

      /**
        @brief Gets the number of holes in a HolesConveyor
        @param[in] conveyor [const HolesConveyor&] The HolesConveyor struct
        @return int The size of @param conveyor
       **/
      static int size(const HolesConveyor& conveyor);
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
      static void appendDummyConveyor(
        const cv::Point2f& rectangleUpperLeft,
        const cv::Point& outlineUpperLeft,
        const int& rx, const int& ry,
        const int& ox, const int& oy,
        HolesConveyor* conveyor);

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
      static std::vector<cv::Point2f> generateRectangle(
        const cv::Point2f& upperLeft, const int& x, const int& y);

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
      static std::vector<cv::Point> generateRectangle(
        const cv::Point& upperLeft, const int& x, const int& y);

      /**
        @brief Draws the keypoints, rectangles and outlines of holes
        @param[in] coneyor [const HolesConveyor&] The conveyor whose holes
        will be drawn on @param img
        @param[in][out] img [cv::Mat*] The image drawn
        @return void
       **/
      static void draw(const HolesConveyor& conveyor,
        cv::Mat* img);
  };

} // namespace pandora_vision

#endif  // UTILS_HOLES_CONVEYOR_H
