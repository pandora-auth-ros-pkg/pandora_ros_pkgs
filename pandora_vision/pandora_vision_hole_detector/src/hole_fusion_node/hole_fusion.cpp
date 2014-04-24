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

#include "hole_fusion_node/hole_fusion.h"

namespace pandora_vision
{
  /**
    @brief The HoleFusion constructor
   **/
  HoleFusion::HoleFusion(void) : pointCloudXYZ_(new PointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("HoleFusion");
    #endif

    ros::Duration(0.5).sleep();

    //!< Calculate the histogram cv::MatND needed for texture comparing
    HistogramCalculation::getHistogram(2, &wallsHistogram_);

    //!< Initialize the numNodesReady variable
    numNodesReady_ = 0;

    //!< Advertise the topic that the rgb_depth_synchronizer will be
    //!< subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>
      ("/vision/hole_fusion/unlock_rgb_depth_synchronizer", 1000, true);

    //!< Subscribe to the topic where the depth node publishes
    //!< candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/candidate_holes", 1,
      &HoleFusion::depthCandidateHolesCallback, this);

    //!< Subscribe to the topic where the rgb node publishes
    //!< candidate holes
    rgbCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/rgb/candidate_holes", 1,
      &HoleFusion::rgbCandidateHolesCallback, this);

    //!< Subscribe to the topic where the synchronizer node publishes
    //!< the point cloud
    pointCloudSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/points", 1,
      &HoleFusion::pointCloudCallback, this);

    //!< The dynamic reconfigure (hole fusion's) parameter's callback
    server.setCallback(boost::bind(&HoleFusion::parametersCallback,
        this, _1, _2));


    ROS_INFO("HoleFusion node initiated");

    //!< Start the synchronizer
    unlockSynchronizer();

    #ifdef DEBUG_TIME
    Timer::tick("HoleFusion");
    #endif
  }



  /**
    @brief The HoleFusion deconstructor
   **/
  HoleFusion::~HoleFusion(void)
  {
    ROS_INFO("HoleFusion node terminated");
  }



  /**
    @brief Callback for the candidate holes via the depth node
    @param[in] depthCandidateHolesVector
    [const vision_communications::CandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
    const vision_communications::CandidateHolesVectorMsg&
    depthCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("depthCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion Depth callback");
    #endif

    //!< Clear the current depthHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&depthHolesConveyor_);

    //!< Unpack the message
    unpackMessage(depthCandidateHolesVector,
      &depthHolesConveyor_,
      &interpolatedDepthImage_,
      sensor_msgs::image_encodings::TYPE_32FC1);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("depthCandidateHolesCallback");
    Timer::printAllMeansTree();
    #endif
  }



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
  void HoleFusion::fromCandidateHoleMsgToConveyor(
    const std::vector<vision_communications::CandidateHoleMsg>&
    candidateHolesVector,
    HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor");
    #endif

    for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
    {
      //!< Recreate conveyor.keypoints
      cv::KeyPoint holeKeypoint;
      holeKeypoint.pt.x = candidateHolesVector[i].keypointX;
      holeKeypoint.pt.y = candidateHolesVector[i].keypointY;
      conveyor->keyPoints.push_back(holeKeypoint);

      //!< Recreate conveyor.rectangles
      std::vector<cv::Point2f> renctangleVertices;
      for (unsigned int v = 0;
        v < candidateHolesVector[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = candidateHolesVector[i].verticesX[v];
        vertex.y = candidateHolesVector[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }
      conveyor->rectangles.push_back(renctangleVertices);

      //!< Recreate conveyor.outlines
      std::vector<cv::Point2f> outlinePoints;
      for (unsigned int o = 0;
        o < candidateHolesVector[i].outlineX.size(); o++)
      {
        cv::Point2f outlinePoint;
        outlinePoint.x = candidateHolesVector[i].outlineX[o];
        outlinePoint.y = candidateHolesVector[i].outlineY[o];
        outlinePoints.push_back(outlinePoint);
      }
      conveyor->outlines.push_back(outlinePoints);
    }

    #ifdef DEBUG_TIME
    Timer::tick("fromCandidateHoleMsgToConveyor");
    #endif
  }



  /**
    @brief With an input a conveyor of holes, this method, depending on
    the depth image's interpolation method, has holes assimilating,
    amalgamating or being connected with holes that can be assimilated,
    amalgamated or connected with or by them. The interpolation method is
    a basic criterion for the mode of merging holes because the two
    filters that verify the validity of each merger are depth-based ones.
    If there is no valid depth image on which to run these filters, it is
    sure that the depth sensor is closer to the scene it is witnessing
    than 0.5-0.6m. In this way of operation, the merging of holes does not
    consider employing validator filters and simply merges holes that can
    be merged with each other (assimilated, amalgamated, or connected).
    @param[in out] conveyor [HolesConveyor*] The conveyor of holes to be
    merged with one another, where applicable.
    @return void
   **/
  void HoleFusion::mergeHoles(HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("mergeHoles", "processCandidateHoles");
    #endif

    //!< Keep a copy of the initial (not merged) candidate holes for
    //!< debugging and exibition purposes
    HolesConveyor conveyorBeforeMerge;

    HolesConveyorUtils::copyTo(*conveyor, &conveyorBeforeMerge);


    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> canvases;
    std::vector<std::string> titles;

    if(Parameters::debug_show_merge_holes)
    {
      //!< Push back the identifier of each keypoint
      for (int i = 0; i < conveyorBeforeMerge.keyPoints.size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage_,
          conveyorBeforeMerge,
          -1,
          msgs));

      titles.push_back(
        TOSTR(conveyor->keyPoints.size()) + " holes before merging");
    }
    #endif


    //!< Try to merge holes that can be assimilated, amalgamated or connected
    if (Parameters::interpolation_method == 0)
    {
      for (int i = 0; i < 3; i++)
      {
        HoleMerger::applyMergeOperation(
          conveyor,
          interpolatedDepthImage_,
          pointCloudXYZ_,
          i);
      }
    }
    else
    {
      for (int i = 0; i < 3; i++)
      {
        HoleMerger::applyMergeOperationWithoutValidation(
          conveyor,
          interpolatedDepthImage_,
          pointCloudXYZ_,
          i);
      }
    }


    #ifdef DEBUG_SHOW
    if(Parameters::debug_show_merge_holes)
    {
      msgs.clear();
      //!< Push back the identifier of each keypoint
      for (int i = 0; i < conveyor->keyPoints.size(); i++)
      {
        msgs.push_back(TOSTR(i));
      }

      canvases.push_back(
        Visualization::showHoles(
          "",
          interpolatedDepthImage_,
          *conveyor,
          -1,
          msgs));

      titles.push_back(
        TOSTR(conveyor->keyPoints.size()) + " holes after merging");

      Visualization::multipleShow("Merged Keypoints",
        canvases, titles, Parameters::debug_show_merge_holes_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("mergeHoles");
    #endif
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config
    [const pandora_vision_hole_detector::hole_fusion_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void HoleFusion::parametersCallback(
    const pandora_vision_hole_detector::hole_fusion_cfgConfig &config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Parameters callback called");
    #endif

    //!< canny parameters
    Parameters::canny_ratio =
      config.canny_ratio;
    Parameters::canny_kernel_size =
      config.canny_kernel_size;
    Parameters::canny_low_threshold =
      config.canny_low_threshold;
    Parameters::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;

    Parameters::contrast_enhance_beta =
      config.contrast_enhance_beta;
    Parameters::contrast_enhance_alpha =
      config.contrast_enhance_alpha;

    //!< Threshold parameters
    Parameters::threshold_lower_value =
      config.threshold_lower_value;

    //!< Blob detection parameters
    Parameters::blob_min_threshold =
      config.blob_min_threshold;
    Parameters::blob_max_threshold =
      config.blob_max_threshold;
    Parameters::blob_threshold_step =
      config.blob_threshold_step;
    Parameters::blob_min_area =
      config.blob_min_area;
    Parameters::blob_max_area =
      config.blob_max_area;
    Parameters::blob_min_convexity =
      config.blob_min_convexity;
    Parameters::blob_max_convexity =
      config.blob_max_convexity;
    Parameters::blob_min_inertia_ratio =
      config.blob_min_inertia_ratio;
    Parameters::blob_max_circularity =
      config.blob_max_circularity;
    Parameters::blob_min_circularity =
      config.blob_min_circularity;
    Parameters::blob_filter_by_color =
      config.blob_filter_by_color;
    Parameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;

    //!< Bounding boxes parameters
    Parameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;

    //!< The bounding box detection zzmethod
    //!< 0 for detecting by means of brushfire starting
    //!< from the keypoint of the blob
    //!< 1 for detecting by means of contours around the edges of the blob
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;

    //!< When using raycast instead of brushfire to find the (approximate here)
    //!< outline of blobs, raycast_keypoint_partitions dictates the number of
    //!< rays, or equivalently, the number of partitions in which the blob is
    //!< partitioned in search of the blob's borders
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //<! Loose ends connection parameters
    Parameters::AB_to_MO_ratio =
      config.AB_to_MO_ratio;
    Parameters::minimum_curve_points =
      config.minimum_curve_points;

    ////!< Interpolation parameters

    //!< The interpolation method for noise removal
    //!< 0 for averaging the pixel's neighbor values
    //!< 1 for brushfire near
    //!< 2 for brushfire far
    Parameters::interpolation_method =
      config.interpolation_method;
    //!< Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::scale_method =
      config.scale_method;

    //!< Hole checkers and their thresholds
    Parameters::run_checker_depth_diff =
      config.run_checker_depth_diff;
    Parameters::checker_depth_diff_threshold =
      config.checker_depth_diff_threshold;

    Parameters::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    Parameters::checker_outline_of_rectangle_threshold =
      config.checker_outline_of_rectangle_threshold;

    Parameters::run_checker_depth_area =
      config.run_checker_depth_area;
    Parameters::checker_depth_area_threshold =
      config.checker_depth_area_threshold;

    Parameters::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    Parameters::checker_brushfire_outline_to_rectangle_threshold =
      config.checker_brushfire_outline_to_rectangle_threshold;

    Parameters::run_checker_depth_homogeneity =
      config.run_checker_depth_homogeneity;
    Parameters::checker_depth_homogeneity_threshold =
      config.checker_depth_homogeneity_threshold;


    Parameters::rectangle_inflation_size =
      config.rectangle_inflation_size;

    Parameters::holes_gaussian_mean=
      config.holes_gaussian_mean;
    Parameters::holes_gaussian_stddev=
      config.holes_gaussian_stddev;


    Parameters::run_checker_color_homogeneity =
      config.run_checker_color_homogeneity;
    Parameters::checker_color_homogeneity_threshold =
      config.checker_color_homogeneity_threshold;

    Parameters::run_checker_luminosity_diff =
      config.run_checker_luminosity_diff;
    Parameters::checker_luminosity_diff_threshold =
      config.checker_luminosity_diff_threshold;

    Parameters::run_checker_texture_diff =
      config.run_checker_texture_diff;
    Parameters::checker_texture_diff_threshold =
      config.checker_texture_diff_threshold;

    Parameters::run_checker_texture_backproject =
      config.run_checker_texture_backproject;
    Parameters::checker_texture_backproject_threshold =
      config.checker_texture_backproject_threshold;

    Parameters::segmentation_method =
      config.segmentation_method;
    Parameters::max_iterations =
      config.max_iterations;
    Parameters::num_points_to_exclude =
      config.num_points_to_exclude;
    Parameters::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;


    //!< Debug
    Parameters::debug_show_find_holes =
      config.debug_show_find_holes;
    Parameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;
    Parameters::debug_show_denoise_edges =
      config.debug_show_denoise_edges;
    Parameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;
    Parameters::debug_show_connect_pairs =
      config.debug_show_connect_pairs;
    Parameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    Parameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    Parameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;

    Parameters::debug_show_check_holes =
      config.debug_show_check_holes;
    Parameters::debug_show_check_holes_size =
      config.debug_show_check_holes_size;

    Parameters::debug_show_merge_holes =
      config.debug_show_merge_holes;
    Parameters::debug_show_merge_holes_size =
      config.debug_show_merge_holes_size;


    //!< Texture parameters
    //!< The threshold for texture matching
    Parameters::match_texture_threshold =
      config.match_texture_threshold;

    Parameters::non_zero_points_in_box_blob_histogram =
      config.non_zero_points_in_box_blob_histogram;

    //!<Color homogeneity parameters
    Parameters::num_bins_threshold =
      config.num_bins_threshold;

    //!< Histogram parameters
    Parameters::number_of_hue_bins =
      config.number_of_hue_bins;
    Parameters::number_of_saturation_bins =
      config.number_of_saturation_bins;
    Parameters::number_of_value_bins =
      config.number_of_value_bins;

    //!< Holes connection - merger
    Parameters::connect_holes_min_distance =
      config.connect_holes_min_distance;
    Parameters::connect_holes_max_distance =
      config.connect_holes_max_distance;
  }



  /**
    @brief Callback for the point cloud that the synchronizer node
    publishes
    @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The message
    containing the point cloud
    @return void
   **/
  void HoleFusion::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("pointCloudCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion Point Cloud callback");
    #endif

    //!< Unpack the point cloud and store it in the member variable
    //!< pointCloudXYZ_
    MessageConversions::extractPointCloudXYZFromMessage(msg,
      &pointCloudXYZ_);

    //!< Extract the depth image from the point cloud message
    cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
      msg, CV_32FC1);

    //!< Interpolate the depthImage
    cv::Mat interpolatedDepthImage;
    NoiseElimination::performNoiseElimination(depthImage,
      &interpolatedDepthImage);

    //!< Set the interpolatedDepthImage's values as the depth values
    //!< of the point cloud
    setDepthValuesInPointCloud(interpolatedDepthImage, &pointCloudXYZ_);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("pointCloudCallback");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Implements a strategy to combine
    information from both sources in order to accurately find valid holes
    @return void
   **/
  void HoleFusion::processCandidateHoles()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Processing candidate holes");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("processCandidateHoles", "", true);
    #endif

    //!< Merge the conveyors from the RGB and Depth sources
    HolesConveyor rgbdHolesConveyor;
    HolesConveyorUtils::merge(depthHolesConveyor_, rgbHolesConveyor_,
      &rgbdHolesConveyor);

    /*//////////////////////////////////////////////////////////////////////////
    //!< Uncomment for testing artificial holes' merging process
    HolesConveyor dummy;
    testDummyHolesMerging(&dummy);
    return;
    //////////////////////////////////////////////////////////////////////////*/

    //!< Apply the {assimilation, amalgamation, connection} processes
    mergeHoles(&rgbdHolesConveyor);

    //!< Apply all active filters
    siftHoles(rgbdHolesConveyor);

    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Callback for the candidate holes via the rgb node
    @param[in] depthCandidateHolesVector
    [const vision_communications::CandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the rgb node
    @return void
   **/
  void HoleFusion::rgbCandidateHolesCallback(
    const vision_communications::CandidateHolesVectorMsg&
    rgbCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("rgbCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion RGB callback");
    #endif

    //!< Clear the current rgbHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&rgbHolesConveyor_);

    //!< Unpack the message
    unpackMessage(rgbCandidateHolesVector,
      &rgbHolesConveyor_,
      &rgbImage_,
      sensor_msgs::image_encodings::TYPE_8UC3);

    numNodesReady_++;

    //!< If the RGB and the depth nodes are ready
    //!< and the point cloud has been delivered and interpolated,
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 3)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("rgbCandidateHolesCallback");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Sets the depth values of a point cloud according to the
    values of a depth image
    @param[in] inImage [const cv::Mat&] The depth image in CV_32FC1 format
    @param[out] pointCloudXYZPtr [PointCloudXYZPtr*] The point cloud
    @return void
   **/
  void HoleFusion::setDepthValuesInPointCloud(const cv::Mat& inImage,
    PointCloudXYZPtr* pointCloudXYZPtr)
  {
    #ifdef DEBUG_TIME
    Timer::start("setDepthValuesInPointCloud", "pointCloudCallback");
    #endif

    //!< If the inImage is not of type CV_32FC1, return
    if(inImage.type() != CV_32FC1)
    {
      return;
    }

    for (unsigned int row = 0; row < (*pointCloudXYZPtr)->height; ++row)
    {
      for (unsigned int col = 0; col < (*pointCloudXYZPtr)->width; ++col)
      {
        (*pointCloudXYZPtr)->points[col + (*pointCloudXYZPtr)->width * row].z =
          inImage.at<float>(row, col);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("setDepthValuesInPointCloud");
    #endif
  }



  /**
    @brief Runs candidate holes through selected filters.
    Probabilities for each candidate hole and filter
    are printed in the console, with an order specified by the
    hole_fusion_cfg of the dynamic reconfigure utility
    @param[in] conveyor [const HolesConveyor&] The conveyor
    containing candidate holes
    @return void
   **/
  void HoleFusion::siftHoles(const HolesConveyor& conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("siftHoles", "processCandidateHoles");
    #endif

    //!< A vector of images that each one of them represents the corresponding
    //!< hole's mask: non-zero value pixels are within a hole's outline points
    std::vector<cv::Mat> holesMasksImageVector;

    //!< A vector of sets that each one of them contains indices of points
    //!< inside the hole's outline
    std::vector<std::set<unsigned int> > holesMasksSetVector;

    //!< A vector of vertices of each inflated bounding rectangle.
    std::vector<std::vector<cv::Point2f> > inflatedRectanglesVector;

    //!< Since inflated rectangle's vertices may go outside the image's bounds,
    //!< this vector stores the indices of the keypoints whose corresponding
    //!< inflated rectangle is in totality within the image's bounds
    std::vector<int> inflatedRectanglesIndices;

    //!< A vector of sets that each one of them contains indices of points
    //!< between the hole's outline and its respective bounding box
    std::vector<std::set<unsigned int> > intermediatePointsSetVector;

    //!< A vector of images that each one of them contains points
    //!< between the hole's outline and its respective bounding box
    std::vector<cv::Mat> intermediatePointsImageVector;

    //!< Construct the necessary vectors, depending on which filters
    //!< are to run in runtime and on the interpolation method used
    FiltersResources::createCheckerRequiredVectors(
      conveyor,
      interpolatedDepthImage_,
      Parameters::rectangle_inflation_size,
      Parameters::interpolation_method,
      &holesMasksImageVector,
      &holesMasksSetVector,
      &inflatedRectanglesVector,
      &inflatedRectanglesIndices,
      &intermediatePointsImageVector,
      &intermediatePointsSetVector);


    //!< The overall 2D vector that contains the probabilities from
    //!< both the depth and rgb filtering regimes
    //!< Each column is a specific hole.
    //!< In each row there are values of probabilities of a specific filter
    std::vector<std::vector<float> > probabilitiesVector2D;


    //!< Initialize the rgb probabilities 2D vector. But first we need to know
    //!< how many rows the vector will accomodate
    int rgbActiveFilters = 0;

    if (Parameters::run_checker_color_homogeneity > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_luminosity_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_texture_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::run_checker_texture_backproject > 0)
    {
      rgbActiveFilters++;
    }

    if (rgbActiveFilters > 0)
    {
      std::vector<std::vector<float> > rgbProbabilitiesVector2D(
        rgbActiveFilters,
        std::vector<float>(conveyor.keyPoints.size(), 0.0));

      //!< check holes for debugging purposes
      RgbFilters::checkHoles(
        conveyor,
        rgbImage_,
        wallsHistogram_,
        inflatedRectanglesIndices,
        holesMasksImageVector,
        holesMasksSetVector,
        intermediatePointsImageVector,
        intermediatePointsSetVector,
        &rgbProbabilitiesVector2D);

      #ifdef DEBUG_SHOW
      if (conveyor.keyPoints.size() > 0)
      {
        ROS_ERROR("RGB: Candidate Holes' probabilities");
        for (int j = 0; j < conveyor.keyPoints.size(); j++)
        {
          std::string probsString;
          for (int i = 0; i < rgbActiveFilters; i++)
          {
            probsString += TOSTR(rgbProbabilitiesVector2D[i][j]) + " | ";
          }

          ROS_ERROR("P_%d [%f %f] = %s", j, conveyor.keyPoints[j].pt.x,
            conveyor.keyPoints[j].pt.y, probsString.c_str());
        }
      }
      #endif

      //!< Fill the probabilitiesVector2D with the rgb one
      for (int i = 0; i < rgbProbabilitiesVector2D.size(); i++)
      {
        std::vector<float> row;
        for (int j = 0; j < rgbProbabilitiesVector2D[i].size(); j++)
        {
          row.push_back(rgbProbabilitiesVector2D[i][j]);
        }
        probabilitiesVector2D.push_back(row);
      }
    }

    //!< If depth analysis is applicable
    if (Parameters::interpolation_method == 0)
    {
      //!< Initialize the depth probabilities 2D vector.
      //!< But first we need to know how many rows the vector will accomodate
      int depthActiveFilters = 0;

      if (Parameters::run_checker_depth_diff > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::run_checker_outline_of_rectangle > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::run_checker_depth_area > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::run_checker_brushfire_outline_to_rectangle > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::run_checker_depth_homogeneity > 0)
      {
        depthActiveFilters++;
      }

      if (depthActiveFilters > 0)
      {
        std::vector<std::vector<float> > depthProbabilitiesVector2D(
          depthActiveFilters,
          std::vector<float>(conveyor.keyPoints.size(), 0.0));

        //!< check holes for debugging purposes
        DepthFilters::checkHoles(
          conveyor,
          interpolatedDepthImage_,
          pointCloudXYZ_,
          holesMasksSetVector,
          inflatedRectanglesVector,
          inflatedRectanglesIndices,
          intermediatePointsSetVector,
          &depthProbabilitiesVector2D);

        #ifdef DEBUG_SHOW
        ROS_ERROR("-------------------------------------------");
        if (conveyor.keyPoints.size() > 0)
        {
          ROS_ERROR("Depth : Candidate Holes' probabilities");
          for (int j = 0; j < conveyor.keyPoints.size(); j++)
          {
            std::string probsString;
            for (int i = 0; i < depthActiveFilters; i++)
            {
              probsString += TOSTR(depthProbabilitiesVector2D[i][j]) + " | ";
            }

            ROS_ERROR("P_%d [%f %f]= %s", j, conveyor.keyPoints[j].pt.x,
              conveyor.keyPoints[j].pt.y, probsString.c_str());
          }
        }
        #endif

        //!< Fill the probabilitiesVector2D with the rgb one
        for (int i = 0; i < depthProbabilitiesVector2D.size(); i++)
        {
          std::vector<float> row;
          for (int j = 0; j < depthProbabilitiesVector2D[i].size(); j++)
          {
            row.push_back(depthProbabilitiesVector2D[i][j]);
          }
          probabilitiesVector2D.push_back(row);
        }
      }
    }
    //!< All filters have been applied, all probabilities produced


    #ifdef DEBUG_TIME
    Timer::tick("siftHoles");
    #endif
  }



  /**
    @brief Requests from the synchronizer to process a new point cloud
    @return void
   **/
  void HoleFusion::unlockSynchronizer()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Sending unlock message");
    #endif

    std_msgs::Empty unlockMsg;
    unlockPublisher_.publish(unlockMsg);
  }



  /**
    @brief Unpacks the the HolesConveyor struct for the
    candidate holes, the interpolated depth image and the point cloud
    from the vision_communications::CandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::CandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The output point cloud
    @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
    depth image
    @return void
   **/
  void HoleFusion::unpackMessage(
    const vision_communications::CandidateHolesVectorMsg& holesMsg,
    HolesConveyor* conveyor, cv::Mat* image, const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackMessage");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the image
    MessageConversions::extractImageFromMessageContainer(
      holesMsg,
      image,
      encoding);

    #ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
    #endif
  }



  /**
    @brief Tests the merging operations on artificial holes
    @param[out] dummy [HolesConveyor*] The hole candidates
    @return void
   **/
  void HoleFusion::testDummyHolesMerging(HolesConveyor* dummy)
  {
    #ifdef DEBUG_TIME
    Timer::start("testDummyHolesMerging", "processCandidateHoles");
    #endif

    //!< Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(20, 20), cv::Point2f(30, 30), 50, 50, 30, 30, dummy);

    //!< Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(80, 80), cv::Point2f(90, 90), 50, 50, 30, 30, dummy);


    //!< 0-th assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(370.0, 130.0), cv::Point2f(372.0, 132.0), 80, 80, 76, 76,
      dummy);

    //!< 0-th overlapper
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(372.0, 132.0), cv::Point2f(374.0, 134.0), 80, 80, 76, 76,
      dummy);

    //!< 0-th assimilable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(380.0, 140.0), cv::Point2f(382.0, 142.0), 20, 20, 16, 16,
      dummy);

    //!< 0-th amalgamatable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(420.0, 140.0), cv::Point2f(422.0, 142.0), 120, 40, 116, 36,
      dummy);

    //!< 0-th connectable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(510.0, 80.0), cv::Point2f(512.0, 82.0), 40, 40, 36, 36,
      dummy);


    //!< 1-st assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(300.0, 300.0), cv::Point2f(302.0, 302.0), 100, 100, 96, 96,
      dummy);

    //!< 1-st connectable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(410.0, 350.0), cv::Point2f(412.0, 352.0), 50, 50, 46, 46,
      dummy);


    HolesConveyorUtils::shuffle(dummy);

    ROS_ERROR("keypoints before: %d ", HolesConveyorUtils::size(*dummy));

    std::vector<std::string> msgs;
    Visualization::showHoles("before", interpolatedDepthImage_, *dummy,
      1, msgs);

    for (int i = 0; i < 3; i++)
    {
      HoleMerger::applyMergeOperation(
        dummy,
        interpolatedDepthImage_,
        pointCloudXYZ_,
        i);
    }

    ROS_ERROR("keypoints after: %d ", HolesConveyorUtils::size(*dummy));
    Visualization::showHoles("after", interpolatedDepthImage_, *dummy,
      1, msgs);

    #ifdef DEBUG_TIME
    Timer::tick("testDummyHolesMerging");
    #endif
  }

} // namespace pandora_vision
