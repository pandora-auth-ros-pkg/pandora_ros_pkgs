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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,
 *          Christos Tsirigotis <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_DEPTH_NODE_UTILS_BLOB_VECTOR_H
#define PANDORA_VISION_HOLE_DEPTH_NODE_UTILS_BLOB_VECTOR_H

#include "pandora_vision_msgs/BlobVector.h"
#include "pandora_vision_msgs/Blob.h"
#include "pandora_vision_msgs/Keypoint.h"

#include "depth_node/utils/defines.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace depth
{
  /**
   * @Class BlobVector
   * @brief Wrapper class to BlobVector msg of pandora_vision_msgs
   */
  class BlobVector
  {
  public:
    BlobVector();
    explicit BlobVector(const pandora_vision_msgs::BlobVector& src);
    BlobVector(const pandora_vision_msgs::BlobVector& src,
        int raycastKeypointPartitions);
    virtual
      ~BlobVector();

    pandora_vision_msgs::BlobVector createMessage(const cv::Mat& image,
        const std::string& encoding, const std_msgs::Header& header);

    cv::Mat
      getCvImage(const std::string& encoding);

    void
      fromWavelets(int raycastKeypointPartitions);

    int
      size() const;

    /**
     * @brief Extends one BlobVector struct to another.
     * @param[in] src [const BlobVector&] The source conveyor
     * @return void
     */
    void
      extend(const BlobVector& src);
    void
      extend(const pandora_vision_msgs::BlobVector& src);

    void
      append(const pandora_vision_msgs::Blob& blob);

    /**
     * @brief Appends a dummy BlobVector to a HoleConveyor struct
     * @param[in] rectangleUpperLeft [const cv::Point2f&] The upper left
     * vertex of the bounding rectangle
     * @param[in] outlineUpperLeft [const cv::Point2f] The upper left
     * vertex of the hole's outline
     * @param[in] rx [int] The width of the rectangle
     * @param[in] ry [int] The height of the rectangle
     * @param[in] ox [int] The width of the outline rectangle
     * @param[in] oy [int] The height of the outline rectangle
     * @return void
     */
    void
      append(const cv::Point2f& rectangleUpperLeft,
          const cv::Point2f& outlineUpperLeft,
          int rx, int ry, int ox, int oy);

    /**
     * @brief Hollows a BlobVector struct, deleting every entry in it
     * @return void
     */
    void
      clear();

    /**
     * @brief Copies one BlobVector struct to another. If the dst conveyor
     * is not empty, it empties it first, and then copies the src to the dst
     * @param[in] src [const BlobVector&] The source struct
     * @return void
     */
    void
      copy(const pandora_vision_msgs::BlobVector& src);
    void
      copy(const BlobVector& src);

    /**
     * @brief Extracts the specified hole from a BlobVector into a new
     * BlobVector struct that is returned
     * @param[in] index [int] The index of the hole inside the conveyor
     * @return A pandora_vision_msgs::blob that containes the index-th hole of the
     * conveyor
     */
    pandora_vision_msgs::Blob
      getBlob(int index) const;
    pandora_vision_msgs::Blob&
      getBlob(int index);

    /**
     * @brief Given two sources of struct BlobVector, this function
     * merge them into one struct.
     * @param[in] srcA [const BlobVector&] The first BlobVector source
     * @param[in] srcB [const BlobVector&] The second BlobVector source
     */
    void
      merge(const BlobVector& srcA,
          const BlobVector& srcB);
    void
      merge(const pandora_vision_msgs::BlobVector& srcA,
          const BlobVector& srcB);
    void
      merge(const BlobVector& srcA,
          const pandora_vision_msgs::BlobVector& srcB);
    void
      merge(const pandora_vision_msgs::BlobVector& srcA,
          const pandora_vision_msgs::BlobVector& srcB);

    /**
     * @brief Deletes a hole from BlobVector struct,
     * @param[in] index [int] The index of the hole in the conveyor
     * @return void
     */
    void
      removeHole(int index);

    /**
     * @brief Replaces an entire BlobVector struct with another
     * @param[in] src [const BlobVector&] The source conveyor struct
     * @return void
     */
    void
      replace(const BlobVector& src);
    void
      replace(const pandora_vision_msgs::BlobVector& src);

    /**
     * @brief Replaces a specified hole from a BlobVector dst struct
     * with the hole of index srcIndex of the src BlobVector struct entry
     * @param[in] src [const BlobVector&] The BlobVector source struct
     * @param[in] srcIndex [int] The index of the hole inside the
     * src conveyor that will be copied into the dst BlobVector struct,
     * in the dstIndex position
     * @param[in] dstIndex [int] The index of the hole inside the
     * @return void
     */
    void
      replaceHole(const BlobVector& src,
          int srcIndex, int dstIndex);
    void
      replaceHole(const pandora_vision_msgs::BlobVector& src,
          int srcIndex, int dstIndex);

    /**
     * @brief Shuffles the contents of a BlobVector
     * @return void
     */
    void
      shuffle();

    /**
     * @brief Prints data pertaining to the contents of a BlobVector struct,
     * that is, the keypoints, rectangle points and outline points of the
     * holes it contains
     * @param[in] conveyor [const BlobVector&] The conveyor
     * @param[in] id [int] The identifier of a specific hole
     * @return void
     */
    void
      print(int id = -1) const;

  private:
    /**
     * @brief Generates a vector of cv::Point2f that represents the 4 vertices
     * of a rectangle
     * @param[in] upperLeft [const cv::Point2f&] The upper left vertex point
     * of the rectangle
     * @param[in] x [int] The length at x direction
     * @param[in] y [int] The length at y direction
     * @param[in] intent [int] 0 for vertices'
     * construction, 1 for a coherent outline construction
     * @return std::vector<pandora_vision_msgs::Keypoint> A vector of four vertices
     * of type cv::Point2f
     */
    std::vector<pandora_vision_msgs::Keypoint>
      generateRectangle(const cv::Point2f& upperLeft,
          int x, int y, int intent);

  private:
    pandora_vision_msgs::BlobVector blobVector_;
  };

}  // namespace depth
}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_DEPTH_NODE_UTILS_BLOB_VECTOR_H
