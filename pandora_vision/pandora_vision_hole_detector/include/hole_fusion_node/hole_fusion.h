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

#ifndef HOLE_FUSION_NODE_HOLE_FUSION_H
#define HOLE_FUSION_NODE_HOLE_FUSION_H

#include <dirent.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include "vision_communications/CandidateHolesVectorMsg.h"
#include "vision_communications/CandidateHoleMsg.h"
#include "utils/message_conversions.h"
#include "utils/noise_elimination.h"
#include "utils/defines.h"
#include "utils/parameters.h"
#include "utils/visualization.h"
#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/rgb_filters.h"
#include "hole_fusion_node/hole_merger.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  class HoleFusion
  {
    private:

      //!< The ROS node handle
      ros::NodeHandle nodeHandle_;

      //!< The ROS publisher that will be used for unlocking the
      //!< synchronizer_node
      ros::Publisher unlockPublisher_;

      //!< The ROS subscriber for acquisition of candidate holes originated
      //!< from the depth node
      ros::Subscriber depthCandidateHolesSubscriber_;

      //!< The ROS subscriber for acquisition of candidate holes originated
      //!< from the rgb node
      ros::Subscriber rgbCandidateHolesSubscriber_;

      //!< The ROS subscriber for acquisition of the point cloud originated
      //!< from th e synchronizer node
      ros::Subscriber pointCloudSubscriber_;

      //!< Indicates how many of the depth_node and rgb_node nodes have
      //!< received hole candidates and are ready to send them for processing
      int numNodesReady_;

      //!< The rgb received by the RGB node
      cv::Mat rgbImage_;

      //!< The point cloud received by the depth node
      PointCloudXYZPtr pointCloudXYZ_;

      //!< The interpolated depth image received by the depth node
      cv::Mat interpolatedDepthImage_;

      //!< The conveyor of hole candidates received by the depth node
      HolesConveyor depthHolesConveyor_;

      //!< The conveyor of hole candidates received by the rgb node
      HolesConveyor rgbHolesConveyor_;

      //!< A histogramm for the texture of walls
      cv::MatND wallsHistogram_;

      //!< The dynamic reconfigure (hole fusion's) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        hole_fusion_cfgConfig> server;

      //!< The dynamic reconfigure (hole fusion's) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        hole_fusion_cfgConfig>:: CallbackType f;

      /**
        @brief The function called when a parameter is changed
        @param[in] config
        [const pandora_vision_hole_detector::hole_fusion_cfgConfig&]
        @param[in] level [const uint32_t] The level (?)
        @return void
       **/
      void parametersCallback(
        const pandora_vision_hole_detector::hole_fusion_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief Callback for the candidate holes via the depth node
        @param[in] depthCandidateHolesVector
        [const vision_communications::CandidateHolesVectorMsg&]
        The message containing the necessary information to filter hole
        candidates acquired through the depth node
        @return void
       **/
      void depthCandidateHolesCallback(
        const vision_communications::CandidateHolesVectorMsg&
        depthCandidateHolesVector);

      /**
        @brief Callback for the candidate holes via the rgb node
        @param[in] depthCandidateHolesVector
        [const vision_communications::CandidateHolesVectorMsg&]
        The message containing the necessary information to filter hole
        candidates acquired through the rgb node
        @return void
       **/
      void rgbCandidateHolesCallback(
        const vision_communications::CandidateHolesVectorMsg&
        rgbCandidateHolesVector);

      /**
        @brief Callback for the point cloud that the synchronizer node
        publishes
        @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The message
        containing the point cloud
        @return void
       **/
      void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

      /**
        @brief Recreates the HolesConveyor struct for the
        candidate holes from the
        vision_communications::CandidateHolerMsg message
        @param[in]candidateHolesVector
        [const std::vector<vision_communications::CandidateHoleMsg>&]
        The input candidate holes
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @return void
       **/
      void fromCandidateHoleMsgToConveyor(
        const std::vector<vision_communications::CandidateHoleMsg>&
        candidateHolesVector,
        HolesConveyor* conveyor);

      /**
        @brief Computes a cv::MatND histogram from images loaded in directory
        ${pandora_vision_hole_detector}/src/wall_pictures and stores it in
        a private member variable so as to be used in texture comparing
        @parameters void
        @return void
       **/
      void getWallsHistogram();

      /**
        @brief Unpacks the the HolesConveyor struct for the
        candidate holes and the image
        from the vision_communications::CandidateHolesVectorMsg message
        @param[in] holesMsg
        [vision_communications::CandidateHolesVectorMsg&] The input
        candidate holes message
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image's encoding
        @return void
       **/
      void unpackMessage(
        const vision_communications::CandidateHolesVectorMsg& holesMsg,
        HolesConveyor* conveyor, cv::Mat* image, const std::string& encoding);

      /**
        @brief Implements a strategy to combine
        information from both sources in order to accurately find valid holes
        @return void
       **/
      void processCandidateHoles();

      /**
        @brief Runs candidate holes through selected filters.
        Probabilities for each candidate hole and filter
        are printed in the console, with an order specified by the
        hole_fusion_cfg of the dynamic reconfigure utility
        @param[in] conveyor [const HolesConveyor&] The conveyor
        containing candidate holes
        @return void
       **/
      void sift(const HolesConveyor& conveyor);

      /**
        @brief Applies a merging operation of @param operationId, until
        every candidate hole, even as it changes through the various merges that
        happen, has been merged with every candidate hole that can be merged
        with it.
        @param[in][out] rgbdHolesConveyor [HolesConveyor*] The unified rgb-d
        candidate holes conveyor
        @param[in] operationId [const int&] The identifier of the merging
        process. Values: 0 for assimilation, 1 for amalgamation and
        2 for connecting
        @return void
       **/
      void applyMergeOperation(HolesConveyor* rgbdHolesConveyor,
        const int& operationId);

      /**
        @brief Requests from the synchronizer to process a new point cloud
        @return void
       **/
      void unlockSynchronizer();

      /**
        @brief Sets the depth values of a point cloud according to the
        values of a depth image
        @param[in] inImage [const cv::Mat&] The depth image in CV_32FC1 format
        @param[out] pointCloudXYZPtr [PointCloudXYZPtr*] The point cloud
        @return void
       **/
      void setDepthValuesInPointCloud(const cv::Mat& inImage,
        PointCloudXYZPtr* pointCloudXYZPtr);

      /**
        @brief Tests the merging operations on artificial holes
        @param[out] dummy [HolesConveyor*] The hole candidates
        @return void
       **/
      void testDummyHolesMerging(HolesConveyor* dummy);

      /**
        @brief Each Depth and RGB filter requires the construction of a set
        of vectors which uses to determine the validity of each hole.
        The total number of vectors is finite; every filter uses vectors from
        this pool of vectors. This method centrally constructs the necessary
        vectors in runtime, depending on which filters are commanded to run
        @param[in] conveyor [const HolesConeveyor&] The candidate holes
        from which each element of the vector will be constructed
        @param[in] image [const cv::Mat&] An image needed for its size
        @param[in] inflationSize [const int&] The bounding rectangles
        inflation size
        @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
        A vector containing an image (the mask) for each hole
        @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
        A vector that holds sets of points's indices; each set holds the
        indices of the points inside the outline of each hole
        @param[out] inflatedRectanglesVector
        [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
        vertices of the in-image-bounds inflated rectangles
        @param[out] inflatedRectanglesIndices [std::vector<int>*]
        The vector that holds the indices of the original holes whose
        inflated bounding rectangles is within the image's bounds.
        @param[out] intermediatePointsImageVector [std::vector<cv::Mat>*]
        A vector that holds the image of the intermediate points between
        a hole's outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
        A vector that holds the intermediate points' indices between a hole's
        outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @return void
       **/
      void createCheckerRequiredVectors(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const int& inflationSize,
        std::vector<cv::Mat>* holesMasksImageVector,
        std::vector<std::set<unsigned int> >* holesMasksSetVector,
        std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
        std::vector<int>* inflatedRectanglesIndices,
        std::vector<cv::Mat>* intermediatePointsImageVector,
        std::vector<std::set<unsigned int> >* intermediatePointsSetVector);

      /**
        @brief Some hole checkers require the construction of a hole's mask,
        that is, the pixels inside the hole with a value of
        value 255 while the background pixels are with 0 value.
        Construct each mask here, instead of in each checker.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image required only for the
        masks' size
        @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
        A vector containing an image (the mask) for each hole
        @return void
       **/
      void createHolesMasksImageVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        std::vector<cv::Mat>* holesMasksImageVector);

      /**
        @brief Some hole checkers require access to a hole's inside points,
        Construct a set of points for each hole, and store all of them in
        a vector
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image required to access
        each hole
        @param[out] holesMasksSetVector [std::vector<unsigned int>*] A vector
        that holds sets of points; each set holds the inside points of each hole
        @return void
       **/
      void createHolesMasksSetVector(const HolesConveyor& conveyor,
        const cv::Mat& image,
        std::vector<std::set<unsigned int> >* holesMasksSetVector);

      /**
        @brief Some checkers require the construction of a hole's inflated
        rectangle in order to validate a hole. Construct each mask here.
        Each vector element contains the four vertices of the inflated
        rectangle. A hole's bounding rectangle is inflated by a standard size;
        inflated rectangles that go beyond the image's bounds are discarded,
        that is, the output vector contains the indices of the original
        keypoints whose inflated bounding rectangles is within the image's
        bounds.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflationSize [const int&] The bounding rectangles
        inflation size in pixels
        @param[out] inflatedRectanglesVector
        [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
        vertices of the in-image-bounds inflated rectangles
        @param[out] inflatedRectanglesIndices [std::vector<int>*]
        The vector that holes the indices of the original holes whose
        inflated bounding rectangles is within the image's bounds.
        @return void
       **/
      void createInflatedRectanglesVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const int& inflationSize,
        std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
        std::vector<int>* inflatedRectanglesIndices);

      /**
        @brief For each hole, this function finds the points between the hole's
        outline and the rectangle (inflated or not) that corrensponds to it.
        These points are then stored in an image
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] rectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of each rectangle that corresponds to a specific hole
        inside the coveyor
        @param[in] rectanglesIndices [const std::vector<int>&] A vector that
        is used to identify a hole's corresponding rectangle. Used primarily
        because the rectangles used are inflated rectangles; not all holes
        possess an inflated rectangle
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflationSize [const int&] The bounding rectangles
        inflation size in pixels
        @param[out] intermediatePointsVector [std::vector<cv::Mat>*]
        A vector that holds the image of the intermediate points for each
        hole whose identifier exists in the @param rectanglesIndices vector
        @return void
       **/
      void createIntermediateHolesPointsImageVector(
        const HolesConveyor& conveyor,
        const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
        const std::vector<int>& rectanglesIndices,
        const cv::Mat& image,
        const int& inflationSize,
        std::vector<cv::Mat>* intermediatePointsImageVector);

      /**
        @brief For each hole, this function finds the points between the hole's
        outline and the rectangle (inflated or not) that corrensponds to it.
        These points are then stored in a std::set of ints.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] rectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of each rectangle that corresponds to a specific hole
        inside the coveyor
        @param[in] rectanglesIndices [const std::vector<int>&] A vector that
        is used to identify a hole's corresponding rectangle. Used primarily
        because the rectangles used are inflated rectangles; not all holes
        possess an inflated rectangle
        @param[out] intermediatePointsSetVector
        [std::vector<std::set<unsigned int> >*]
        A vector that holds the intermediate points' indices for each hole
        whose identifier exists in the @param rectanglesIndices vector
        @return void
       **/
      void createIntermediateHolesPointsSetVector(
        const HolesConveyor& conveyor,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        const cv::Mat& image,
        std::vector<std::set<unsigned int> >* intermediatePointsSetVector);


    public:

      /**
        @brief The HoleFusion constructor
       **/
      HoleFusion(void);

      /**
        @brief The HoleFusion deconstructor
       **/
      ~HoleFusion(void);
  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_HOLE_FUSION_H
