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

namespace pandora_vision
{
  /**
    @brief Function that locates the position of potentional holes
    in the current frame.
    @param[in] holeFrame [const cv::Mat&] current frame to be processed
    @param[in] histogram [const cv::MatND&] A histogram made of pictures
    of walls where holes reside
    @return [HolesConveyor] A collection of holes and found information
    about them
    */
  HolesConveyor HoleDetector::findHoles(const cv::Mat& holeFrame,
    const cv::MatND& histogram)
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
      holeFrame.copyTo(tmp);
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += " : Initial RGB image";
      msgs.push_back(msg);
      imgs.push_back(tmp);
    }
    #endif

    // Locate the edges of the RGB image
    cv::Mat edges;
    EdgeDetection::computeRgbEdges(
      holeFrame,
      Parameters::Rgb::edges_extraction_method,
      Parameters::Rgb::segmentation_blur_method,
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

    // Find pixels in current frame where there is the same texture
    // according to the given histogram and calculate
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
          holeFrame,
          -1,
          conveyor.keyPoints,
          conveyor.rectangles,
          std::vector<std::string>(),
          conveyor.outlines)
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
