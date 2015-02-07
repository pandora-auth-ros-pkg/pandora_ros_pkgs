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
 * Authors: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#include "rgb_node/hole_detector.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Finds holes, provided a RGB image in CV_8UC3 format.

    First, the edges of the RGB image are detected.
    Then, keypoints of blobs are detected in the above image.
    Finally, the potential holes' outline is found, along with the bounding
    boxes of those outlines.
    @param[in] rgbImage [const cv::Mat&] The RGB image to be processed,
    in CV_8UC3 format
    @param[in] histogram [const std::vector<cv::MatND>&]
    The vector of histograms of images of wooden walls
    @return HolesConveyor The struct that contains the holes found
   **/
  HolesConveyor HoleDetector::findHoles(const cv::Mat& rgbImage,
    const std::vector<cv::MatND>& histogram)
  {
    #ifdef DEBUG_TIME
    Timer::start("findHoles", "inputRgbImageCallback");
    #endif

    #ifdef DEBUG_SHOW
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_find_holes) // Debug
    {
      cv::Mat tmp;
      rgbImage.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial RGB image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Detect edges in rgbImage
    cv::Mat edges;
    EdgeDetection::computeRgbEdges(
      rgbImage,
      Parameters::Rgb::edges_extraction_method,
      histogram,
      &edges);

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_find_holes) // Debug
    {
      cv::Mat tmp;
      edges.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Edges after denoise");
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Find blobs in the edges image. Each blob is represented as
    // a keypoint which is the center of the blob found
    std::vector<cv::KeyPoint> keyPoints;
    BlobDetection::detectBlobs(edges, &keyPoints);

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Initial keypoints");
      msgs.push_back(msg);
      imgs.push_back(Visualization::showKeypoints(msg, edges, -1, keyPoints));
    }
    #endif

    // The final vectors of keypoints, rectangles and blobs' outlines.
    HolesConveyor conveyor;

    /**
      Get me blobs that their center point is inside the image,
      their bounding box is also entirely inside the image, and their area is
      greater than Parameters::bounding_box_min_area_threshold.
      Each keypoint is associated with exactly one rectangle.
      The end product here is a set of keypoints, a set of rectangles that
      enclose them and a set of the outlines of the blobs found, all tightly
      packed in the conveyor struct.
     **/
    HoleFilters::validateBlobs(
      keyPoints,
      &edges,
      Parameters::Outline::outline_detection_method,
      &conveyor);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_holes)
    {
      msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Blobs");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showHoles(
          msg,
          rgbImage,
          conveyor,
          -1,
          std::vector<std::string>())
        );
    }
    #endif

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_find_holes)
    {
      Visualization::multipleShow("RGB node", imgs, msgs,
        Parameters::Debug::show_find_holes_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("findHoles");
    #endif

    return conveyor;
  }

} // namespace pandora_vision
