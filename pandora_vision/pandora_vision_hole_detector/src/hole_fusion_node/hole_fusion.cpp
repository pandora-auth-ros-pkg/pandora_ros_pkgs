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

    // Calculate the histogram cv::MatND needed for texture comparing
    Histogram::getHistogram(&wallsHistogram_,
      Parameters::Histogram::secondary_channel);

    // Initialize the numNodesReady variable
    numNodesReady_ = 0;

    // Advertise the topic that the rgb_depth_synchronizer will be
    // subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>(
      Parameters::Topics::synchronizer_unlock_topic, 1000, true);

    // Advertise the topic that the yaw and pitch of the keypoints of the final,
    // valid holes will be published to
    validHolesPublisher_ = nodeHandle_.advertise
      <vision_communications::HolesDirectionsVectorMsg>(
        Parameters::Topics::hole_detector_output_topic, 1000, true);

    // Advertise the topic that information about the final holes,
    // will be published to
    enhancedHolesPublisher_ = nodeHandle_.advertise
      <vision_communications::EnhancedHolesVectorMsg>(
        Parameters::Topics::enhanced_holes_topic, 1000, true);

    // Subscribe to the topic where the depth node publishes
    // candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
      Parameters::Topics::depth_candidate_holes_topic, 1,
      &HoleFusion::depthCandidateHolesCallback, this);

    // Subscribe to the topic where the rgb node publishes
    // candidate holes
    rgbCandidateHolesSubscriber_= nodeHandle_.subscribe(
      Parameters::Topics::rgb_candidate_holes_topic, 1,
      &HoleFusion::rgbCandidateHolesCallback, this);

    // Subscribe to the topic where the synchronizer node publishes
    // the point cloud
    pointCloudSubscriber_= nodeHandle_.subscribe(
      Parameters::Topics::point_cloud_internal_topic, 1,
      &HoleFusion::pointCloudCallback, this);

    // The dynamic reconfigure (hole fusion's) parameter's callback
    server.setCallback(boost::bind(&HoleFusion::parametersCallback,
        this, _1, _2));


    ROS_INFO("HoleFusion node initiated");

    // Start the synchronizer
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
    @brief Runs candidate holes through selected filters.
    Probabilities for each candidate hole and filter
    are printed in the console, with an order specified by the
    hole_fusion_cfg of the dynamic reconfigure utility
    @param[in] conveyor [const HolesConveyor&] The conveyor
    containing candidate holes
    @return A two dimensional vector containing the probabilities of
    validity of each candidate hole. Each row of it pertains to a specific
    filter applied, each column to a particular hole
   **/
  std::vector<std::vector<float> > HoleFusion::checkHoles(
    const HolesConveyor& conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles", "processCandidateHoles");
    #endif

    // A vector of images that each one of them represents the corresponding
    // hole's mask: non-zero value pixels are within a hole's outline points
    std::vector<cv::Mat> holesMasksImageVector;

    // A vector of sets that each one of them contains indices of points
    // inside the hole's outline
    std::vector<std::set<unsigned int> > holesMasksSetVector;

    // A vector of vertices of each inflated bounding rectangle.
    std::vector<std::vector<cv::Point2f> > inflatedRectanglesVector;

    // Since inflated rectangle's vertices may go outside the image's bounds,
    // this vector stores the indices of the keypoints whose corresponding
    // inflated rectangle is in totality within the image's bounds
    std::vector<int> inflatedRectanglesIndices;

    // A vector of sets that each one of them contains indices of points
    // between the hole's outline and its respective bounding box
    std::vector<std::set<unsigned int> > intermediatePointsSetVector;

    // A vector of images that each one of them contains points
    // between the hole's outline and its respective bounding box
    std::vector<cv::Mat> intermediatePointsImageVector;

    // Construct the necessary vectors, depending on which filters
    // are to run in runtime and on the interpolation method used
    FiltersResources::createCheckerRequiredVectors(
      conveyor,
      interpolatedDepthImage_,
      Parameters::HoleFusion::rectangle_inflation_size,
      Parameters::Depth::interpolation_method,
      &holesMasksImageVector,
      &holesMasksSetVector,
      &inflatedRectanglesVector,
      &inflatedRectanglesIndices,
      &intermediatePointsImageVector,
      &intermediatePointsSetVector);


    // The overall 2D vector that contains the probabilities from
    // both the depth and rgb filtering regimes
    // Each column is a specific hole.
    // In each row there are values of probabilities of a specific filter
    std::vector<std::vector<float> > probabilitiesVector2D;


    // Initialize the rgb probabilities 2D vector. But first we need to know
    // how many rows the vector will accomodate
    int rgbActiveFilters = 0;

    if (Parameters::HoleFusion::run_checker_color_homogeneity > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::HoleFusion::run_checker_luminosity_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::HoleFusion::run_checker_texture_diff > 0)
    {
      rgbActiveFilters++;
    }
    if (Parameters::HoleFusion::run_checker_texture_backproject > 0)
    {
      rgbActiveFilters++;
    }

    if (rgbActiveFilters > 0)
    {
      std::vector<std::vector<float> > rgbProbabilitiesVector2D(
        rgbActiveFilters,
        std::vector<float>(conveyor.keyPoints.size(), 0.0));

      // check holes for debugging purposes
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

          ROS_ERROR("P_%d [%f %f] : %s", j, conveyor.keyPoints[j].pt.x,
            conveyor.keyPoints[j].pt.y, probsString.c_str());
        }
      }
      #endif

      // Fill the probabilitiesVector2D with the rgb one
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

    // If depth analysis is applicable,
    // determine the probabilities of validity of the candidate holes
    // by running the depth-based filters and append the respective 2D output
    // probabilities vector to the overall probabilities vector
    if (Parameters::Depth::interpolation_method == 0)
    {
      // Initialize the depth probabilities 2D vector.
      // But first we need to know how many rows the vector will accomodate
      int depthActiveFilters = 0;

      if (Parameters::HoleFusion::run_checker_depth_diff > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::HoleFusion::run_checker_outline_of_rectangle > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::HoleFusion::run_checker_depth_area > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle > 0)
      {
        depthActiveFilters++;
      }
      if (Parameters::HoleFusion::run_checker_depth_homogeneity > 0)
      {
        depthActiveFilters++;
      }

      if (depthActiveFilters > 0)
      {
        std::vector<std::vector<float> > depthProbabilitiesVector2D(
          depthActiveFilters,
          std::vector<float>(conveyor.keyPoints.size(), 0.0));

        // check holes for debugging purposes
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

            ROS_ERROR("P_%d [%f %f] : %s", j, conveyor.keyPoints[j].pt.x,
              conveyor.keyPoints[j].pt.y, probsString.c_str());
          }
        }
        #endif

        // Fill the probabilitiesVector2D with the depth one
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

    // All filters have been applied, all probabilities produced
    return probabilitiesVector2D;

    #ifdef DEBUG_TIME
    Timer::tick("checkHoles");
    #endif
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

    // Clear the current depthHolesConveyor struct
    // (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&depthHolesConveyor_);

    // Unpack the message
    unpackMessage(depthCandidateHolesVector,
      &depthHolesConveyor_,
      &interpolatedDepthImage_,
      sensor_msgs::image_encodings::TYPE_32FC1);

    numNodesReady_++;

    // If the RGB and the depth nodes are ready
    // and the point cloud has been delivered and interpolated,
    // unlock the rgb_depth_synchronizer and process the candidate holes
    // from both sources
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
    @param[in] inImage [const cv::Mat&] An image used for its size.
    It is needed if the wavelet method is used in the keypoints' extraction,
    in order to obtain the coherent shape of holes' outline points
    @return void
   **/
  void HoleFusion::fromCandidateHoleMsgToConveyor(
    const std::vector<vision_communications::CandidateHoleMsg>&
    candidateHolesVector,
    HolesConveyor* conveyor,
    const cv::Mat& inImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor");
    #endif

    // Normal mode
    if (Parameters::Image::image_representation_method == 0)
    {
      for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
      {
        // Recreate conveyor.keypoints
        cv::KeyPoint holeKeypoint;
        holeKeypoint.pt.x = candidateHolesVector[i].keypointX;
        holeKeypoint.pt.y = candidateHolesVector[i].keypointY;
        conveyor->keyPoints.push_back(holeKeypoint);

        // Recreate conveyor.rectangles
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

        // Recreate conveyor.outlines
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
    }
    // Wavelet mode
    else if (Parameters::Image::image_representation_method == 1)
    {
      for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
      {
        // Recreate conveyor.keypoints
        cv::KeyPoint holeKeypoint;
        holeKeypoint.pt.x = 2 * candidateHolesVector[i].keypointX;
        holeKeypoint.pt.y = 2 * candidateHolesVector[i].keypointY;
        conveyor->keyPoints.push_back(holeKeypoint);

        // Recreate conveyor.rectangles
        std::vector<cv::Point2f> renctangleVertices;
        for (unsigned int v = 0;
          v < candidateHolesVector[i].verticesX.size(); v++)
        {
          cv::Point2f vertex;
          vertex.x = 2 * candidateHolesVector[i].verticesX[v];
          vertex.y = 2 * candidateHolesVector[i].verticesY[v];
          renctangleVertices.push_back(vertex);
        }
        conveyor->rectangles.push_back(renctangleVertices);

        // Recreate conveyor.outlines
        std::vector<cv::Point2f> sparceOutlinePoints;
        for (unsigned int o = 0;
          o < candidateHolesVector[i].outlineX.size(); o++)
        {
          cv::Point2f outlinePoint;
          outlinePoint.x = 2 * candidateHolesVector[i].outlineX[o];
          outlinePoint.y = 2 * candidateHolesVector[i].outlineY[o];
          sparceOutlinePoints.push_back(outlinePoint);
        }

        std::vector<cv::Point2f> outlinePoints = sparceOutlinePoints;

        // Because the outline points do not constitute a coherent shape,
        // we need to draw them, connect them linearly and then the
        // points that are drawn will be the hole's outline points
        cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
        unsigned char* ptr = canvas.ptr();

        for(unsigned int a = 0; a < sparceOutlinePoints.size(); a++)
        {
          unsigned int ind =
            sparceOutlinePoints[a].x + inImage.cols * sparceOutlinePoints[a].y;

          ptr[ind] = 255;
        }

        // The easiest and most efficient way to obtain the same result as
        // if image_representation_method was 0 is to apply the raycast
        // algorithm
        std::vector<cv::Point2f> outline;
        BlobDetection::raycastKeypoint(holeKeypoint,
          &canvas,
          Parameters::Outline::raycast_keypoint_partitions,
          &outline);

        conveyor->outlines.push_back(outline);
      }
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
    @param[in,out] conveyor [HolesConveyor*] The conveyor of holes to be
    merged with one another, where applicable.
    @return void
   **/
  void HoleFusion::mergeHoles(HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("mergeHoles", "processCandidateHoles");
    #endif

    // Keep a copy of the initial (not merged) candidate holes for
    // debugging and exibition purposes
    HolesConveyor conveyorBeforeMerge;

    HolesConveyorUtils::copyTo(*conveyor, &conveyorBeforeMerge);


    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> canvases;
    std::vector<std::string> titles;

    if(Parameters::Debug::show_merge_holes)
    {
      // Push back the identifier of each keypoint
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


    // Try to merge holes that can be assimilated, amalgamated or connected
    if (Parameters::Depth::interpolation_method == 0)
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
    if(Parameters::Debug::show_merge_holes)
    {
      msgs.clear();
      // Push back the identifier of each keypoint
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
        canvases, titles, Parameters::Debug::show_merge_holes_size, 1);
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
    ROS_INFO("[Hole Fusion node] Parameters callback called");
    #endif

    // Debug
    Parameters::Debug::show_find_holes =
      config.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      config.show_find_holes_size;
    Parameters::Debug::show_denoise_edges =
      config.show_denoise_edges;
    Parameters::Debug::show_denoise_edges_size =
      config.show_denoise_edges_size;
    Parameters::Debug::show_connect_pairs =
      config.show_connect_pairs;
    Parameters::Debug::show_connect_pairs_size =
      config.show_connect_pairs_size;

    Parameters::Debug::show_get_shapes_clear_border  =
      config.show_get_shapes_clear_border;
    Parameters::Debug::show_get_shapes_clear_border_size =
      config.show_get_shapes_clear_border_size;

    Parameters::Debug::show_check_holes =
      config.show_check_holes;
    Parameters::Debug::show_check_holes_size =
      config.show_check_holes_size;

    Parameters::Debug::show_merge_holes =
      config.show_merge_holes;
    Parameters::Debug::show_merge_holes_size =
      config.show_merge_holes_size;


    // The interpolation method for noise removal
    // 0 for averaging the pixel's neighbor values
    // 1 for brushfire near
    // 2 for brushfire far
    Parameters::Depth::interpolation_method =
      config.interpolation_method;


    // Threshold parameters
    Parameters::Edge::denoised_edges_threshold =
      config.denoised_edges_threshold;


    // Histogram parameters
    Parameters::Histogram::number_of_hue_bins =
      config.number_of_hue_bins;
    Parameters::Histogram::number_of_saturation_bins =
      config.number_of_saturation_bins;
    Parameters::Histogram::number_of_value_bins =
      config.number_of_value_bins;
    Parameters::Histogram::secondary_channel =
      config.secondary_channel;



    // Show the holes that each of the depth and RGB nodes transmit to the
    // hole fusion node, on top of their respective origin images
    Parameters::HoleFusion::show_respective_holes =
      config.show_respective_holes;

    // The product of this package: valid holes
    Parameters::HoleFusion::show_final_holes =
     config.show_final_holes;

    // Hole checkers and their thresholds
    Parameters::HoleFusion::run_checker_depth_diff =
      config.run_checker_depth_diff;
    Parameters::HoleFusion::checker_depth_diff_threshold =
      config.checker_depth_diff_threshold;

    Parameters::HoleFusion::run_checker_outline_of_rectangle =
      config.run_checker_outline_of_rectangle;
    Parameters::HoleFusion::checker_outline_of_rectangle_threshold =
      config.checker_outline_of_rectangle_threshold;

    Parameters::HoleFusion::run_checker_depth_area =
      config.run_checker_depth_area;
    Parameters::HoleFusion::checker_depth_area_threshold =
      config.checker_depth_area_threshold;

    Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle =
      config.run_checker_brushfire_outline_to_rectangle;
    Parameters::HoleFusion::checker_brushfire_outline_to_rectangle_threshold =
      config.checker_brushfire_outline_to_rectangle_threshold;

    Parameters::HoleFusion::run_checker_depth_homogeneity =
      config.run_checker_depth_homogeneity;
    Parameters::HoleFusion::checker_depth_homogeneity_threshold =
      config.checker_depth_homogeneity_threshold;

    Parameters::HoleFusion::rectangle_inflation_size =
      config.rectangle_inflation_size;

    Parameters::HoleFusion::holes_gaussian_mean=
      config.holes_gaussian_mean;
    Parameters::HoleFusion::holes_gaussian_stddev=
      config.holes_gaussian_stddev;

    Parameters::HoleFusion::run_checker_color_homogeneity =
      config.run_checker_color_homogeneity;
    Parameters::HoleFusion::checker_color_homogeneity_threshold =
      config.checker_color_homogeneity_threshold;

    Parameters::HoleFusion::run_checker_luminosity_diff =
      config.run_checker_luminosity_diff;
    Parameters::HoleFusion::checker_luminosity_diff_threshold =
      config.checker_luminosity_diff_threshold;

    Parameters::HoleFusion::run_checker_texture_diff =
      config.run_checker_texture_diff;
    Parameters::HoleFusion::checker_texture_diff_threshold =
      config.checker_texture_diff_threshold;

    Parameters::HoleFusion::run_checker_texture_backproject =
      config.run_checker_texture_backproject;
    Parameters::HoleFusion::checker_texture_backproject_threshold =
      config.checker_texture_backproject_threshold;

    // Plane detection
    Parameters::HoleFusion::segmentation_method =
      config.segmentation_method;
    Parameters::HoleFusion::max_iterations =
      config.max_iterations;
    Parameters::HoleFusion::num_points_to_exclude =
      config.num_points_to_exclude;
    Parameters::HoleFusion::point_to_plane_distance_threshold =
      config.point_to_plane_distance_threshold;

    // Holes connection - merger
    Parameters::HoleFusion::connect_holes_min_distance =
      config.connect_holes_min_distance;
    Parameters::HoleFusion::connect_holes_max_distance =
      config.connect_holes_max_distance;

    // Texture parameters
    // The threshold for texture matching
    Parameters::HoleFusion::match_texture_threshold =
      config.match_texture_threshold;

    //Color homogeneity parameters
    Parameters::HoleFusion::num_bins_threshold =
      config.num_bins_threshold;
    Parameters::HoleFusion::non_zero_points_in_box_blob_histogram =
      config.non_zero_points_in_box_blob_histogram;

    // Holes validity thresholds
    // Normal : when depth analysis is applicable
    Parameters::HoleFusion::holes_validity_threshold_normal =
      config.holes_validity_threshold_normal;

    // Urgent : when depth analysis is not applicable, we can only rely
    // on RGB analysis
    Parameters::HoleFusion::holes_validity_threshold_urgent =
      config.holes_validity_threshold_urgent;


    // Depth and RGB image representation method.
    // 0 if the image used is the one obtained from the sensor,
    // unadulterated
    // 1 through wavelet representation
    Parameters::Image::image_representation_method =
      config.image_representation_method;

    // Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::Image::scale_method =
      config.scale_method;


    // The detection method used to obtain the outline of a blob
    // 0 for detecting by means of brushfire
    // 1 for detecting by means of raycasting
    Parameters::Outline::outline_detection_method =
      config.outline_detection_method;

    // When using raycast instead of brushfire to find the (approximate here)
    // outline of blobs, raycast_keypoint_partitions dictates the number of
    // rays, or equivalently, the number of partitions in which the blob is
    // partitioned in search of the blob's borders
    Parameters::Outline::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    // Loose ends connection parameters
    Parameters::Outline::AB_to_MO_ratio =
      config.AB_to_MO_ratio;

    Parameters::Outline::minimum_curve_points =
      config.minimum_curve_points;

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

    // Store the frame id and timestamp of the point cloud under processing
    frame_id_ = msg->header.frame_id;
    timestamp_ = msg->header.stamp;

    // Unpack the point cloud and store it in the member variable
    // pointCloudXYZ_
    MessageConversions::extractPointCloudXYZFromMessage(msg,
      &pointCloudXYZ_);

    // Extract the depth image from the point cloud message
    cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
      msg, CV_32FC1);

    // Interpolate the depthImage
    cv::Mat interpolatedDepthImage;
    NoiseElimination::performNoiseElimination(depthImage,
      &interpolatedDepthImage);

    // Set the interpolatedDepthImage's values as the depth values
    // of the point cloud
    setDepthValuesInPointCloud(interpolatedDepthImage, &pointCloudXYZ_);

    numNodesReady_++;

    // If the RGB and the depth nodes are ready
    // and the point cloud has been delivered and interpolated,
    // unlock the rgb_depth_synchronizer and process the candidate holes
    // from both sources
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

    #ifdef DEBUG_SHOW
    if (Parameters::HoleFusion::show_respective_holes)
    {
      std::vector<std::string> msgs;

      // Holes originated from analysis on the depth image,
      // on top of the depth image
      cv::Mat depthHolesImage =
        Visualization::showHoles("Holes originated from Depth analysis",
          interpolatedDepthImage_,
          depthHolesConveyor_,
          -1,
          msgs);

      // Holes originated from analysis on the RGB image,
      // on top of the RGB image
      cv::Mat rgbHolesImage =
        Visualization::showHoles("Holes originated from RGB analysis",
          rgbImage_,
          rgbHolesConveyor_,
          -1,
          msgs);

      // The two images
      std::vector<cv::Mat> imgs;
      imgs.push_back(depthHolesImage);
      imgs.push_back(rgbHolesImage);

      // The titles of the images
      std::vector<std::string> titles;
      titles.push_back("Holes originated from Depth analysis");
      titles.push_back("Holes originated from RGB analysis");

      Visualization::multipleShow("Respective keypoints", imgs, titles, 1280, 1);
    }
    #endif


    // Merge the conveyors from the RGB and Depth sources
    HolesConveyor rgbdHolesConveyor;
    HolesConveyorUtils::merge(depthHolesConveyor_, rgbHolesConveyor_,
      &rgbdHolesConveyor);

    /*//////////////////////////////////////////////////////////////////////////
    // Uncomment for testing artificial holes' merging process
    HolesConveyor dummy;
    testDummyHolesMerging(&dummy);
    return;
    //////////////////////////////////////////////////////////////////////////*/

    // Apply the {assimilation, amalgamation, connection} processes
    mergeHoles(&rgbdHolesConveyor);

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D;
    probabilitiesVector2D = checkHoles(rgbdHolesConveyor);

    // Which candidate holes are actually holes?
    // The probabilities obtained above need to be evaluated
    std::map<int, float> validHolesMap;
    validHolesMap = validateHoles(probabilitiesVector2D);

    // Publish the final holes
    publishValidHoles(rgbdHolesConveyor, &validHolesMap);

    // Publish the enhanced holes message
    publishEnhancedHoles(rgbdHolesConveyor,
      Parameters::Depth::interpolation_method);

    #ifdef DEBUG_SHOW
    if (Parameters::HoleFusion::show_final_holes)
    {
      // The holes conveyor containing only the valid holes
      HolesConveyor validHolesConveyor;

      // Contains the validity probability for each hole considered valid
      std::vector<std::string> msgs;

      for (std::map<int, float>::iterator it = validHolesMap.begin();
        it != validHolesMap.end(); it++)
      {
        HolesConveyorUtils::append(
          HolesConveyorUtils::getHole(rgbdHolesConveyor, it->first),
          &validHolesConveyor);

        msgs.push_back(TOSTR(it->second));
      }


      // Valid holes on top of the interpolated depth image
      cv::Mat depthValidHolesImage =
        Visualization::showHoles("Valid Holes",
          interpolatedDepthImage_,
          validHolesConveyor,
          -1,
          msgs);

      // Valid holes on top of the RGB image
      cv::Mat rgbValidHolesImage =
        Visualization::showHoles("ValidHoles",
          rgbImage_,
          validHolesConveyor,
          -1,
          msgs);

      // The two images
      std::vector<cv::Mat> imgs;
      imgs.push_back(depthValidHolesImage);
      imgs.push_back(rgbValidHolesImage);

      // The titles of the images
      std::vector<std::string> titles;
      titles.push_back("Valid Holes");
      titles.push_back("Valid Holes");

      Visualization::multipleShow("Valid Holes", imgs, titles, 1280, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Publishes the enhanced holes' information.
    @param[in] conveyor [const HolesConveyor&] The overall unique holes
    found by the depth and RGB nodes.
    @param[in] interpolationMethod [const int&] The interpolation method
    used. 0 if depth analysis is applicable, 1 or 2 for special cases,
    where the amount of noise in the depth image is overwhelming
    @return void
   **/
  void HoleFusion::publishEnhancedHoles (const HolesConveyor& conveyor,
    const int& interpolationMethod)
  {
    // The overall message of enhanced holes that will be published
    vision_communications::EnhancedHolesVectorMsg enhancedHolesMsg;

    // Set the frame element in the enhancedHolesMsg message to the rgb image
    MessageConversions::convertImageToMessage(
      rgbImage_,
      sensor_msgs::image_encodings::TYPE_8UC3,
      enhancedHolesMsg.frame);

    // Set whether depth analysis is applicable
    enhancedHolesMsg.isDepth = (interpolationMethod == 0);

    // Set the message's header
    enhancedHolesMsg.header.stamp = timestamp_;
    enhancedHolesMsg.header.frame_id = frame_id_;

    for (int i = 0; i < HolesConveyorUtils::size(conveyor); i++)
    {
      // The enhanced hole message. Used for one hole only
      vision_communications::EnhancedHoleMsg enhancedHoleMsg;

      // Set the hole's keypoint
      enhancedHoleMsg.keypointX = conveyor.keyPoints[i].pt.x;
      enhancedHoleMsg.keypointY = conveyor.keyPoints[i].pt.y;

      // Set the hole's bounding box vertices
      for (int r = 0; r < conveyor.rectangles[i].size(); r++)
      {
        enhancedHoleMsg.verticesX.push_back(conveyor.rectangles[i][r].x);
        enhancedHoleMsg.verticesY.push_back(conveyor.rectangles[i][r].y);
      }

      // Set the message's header
      enhancedHoleMsg.header.stamp = timestamp_;
      enhancedHoleMsg.header.frame_id = frame_id_;
    }

    // Publish the overall message
    enhancedHolesPublisher_.publish(enhancedHolesMsg);
  }



  /**
    @brief Publishes the valid holes' information.
    @param[in] conveyor [const HolesConveyor&] The overall unique holes
    found by the depth and RGB nodes.
    @param[in] map [std::map<int, float>*] A map containing the indices
    of valid holes and their respective probabilities of validity
    @return void
   **/
  void HoleFusion::publishValidHoles(const HolesConveyor& conveyor,
    std::map<int, float>* map)
  {
    // The depth sensor's horzontal and vertical field of view
    float hfov = Parameters::HoleFusion::horizontal_field_of_view;
    float vfov = Parameters::HoleFusion::vertical_field_of_view;

    // The frame's height and width
    int height = interpolatedDepthImage_.rows;
    int width = interpolatedDepthImage_.cols;

    // The overall valid holes found message
    vision_communications::HolesDirectionsVectorMsg holesVectorMsg;

    // Counter for the holes' identifiers
    int holeId = 0;

    for (std::map<int, float>::iterator it = map->begin();
      it != map->end(); it++)
    {
      // A single hole's message
      vision_communications::HoleDirectionMsg holeMsg;

      // The hole's keypoint coordinates relative to the center of the frame
      float x = conveyor.keyPoints[it->first].pt.x
        - static_cast<float>(width) / 2;
      float y = static_cast<float>(height) / 2
        - conveyor.keyPoints[it->first].pt.y;

      // The keypoint's yaw and pitch
      float yaw = atan(2 * x / width * tan(hfov / 2));
      float pitch = atan(2 * y / height * tan(vfov / 2));

      // Setup everything needed by the single hole's message
      // holeMsg.header.frame_id = frame_id_;
      // holeMsg.header.stamp = timestamp_;

      holeMsg.yaw = yaw;
      holeMsg.pitch = pitch;
      holeMsg.probability = it->second;
      holeMsg.holeId = holeId;

      // Fill the overall holes found message with the current hole message
      holesVectorMsg.holesDirections.push_back(holeMsg);

      holeId++;
    }

    // Publish the message containing the information about all holes found
    holesVectorMsg.header.stamp = timestamp_;
    holesVectorMsg.header.frame_id = frame_id_;

    validHolesPublisher_.publish(holesVectorMsg);
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

    // Clear the current rgbHolesConveyor struct
    // (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&rgbHolesConveyor_);

    // Unpack the message
    unpackMessage(rgbCandidateHolesVector,
      &rgbHolesConveyor_,
      &rgbImage_,
      sensor_msgs::image_encodings::TYPE_8UC3);

    numNodesReady_++;

    // If the RGB and the depth nodes are ready
    // and the point cloud has been delivered and interpolated,
    // unlock the rgb_depth_synchronizer and process the candidate holes
    // from both sources
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

    // If the inImage is not of type CV_32FC1, return
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
    HolesConveyor* conveyor,
    cv::Mat* image,
    const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackMessage");
    #endif

    // Unpack the image
    MessageConversions::extractImageFromMessageContainer(
      holesMsg,
      image,
      encoding);

    // Recreate the conveyor
    fromCandidateHoleMsgToConveyor(
      holesMsg.candidateHoles,
      conveyor,
      *image);

    #ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
    #endif
  }


  /**
    @brief Validates candidate holes, meaning that having a two dimensional
    array that is the product of a series of validity ascertainers that
    in their turn produce a probability hinting to the confidence level
    that a particular candidate hole is indeed a hole, this method fuses
    all the probabilities produced by various hole checkers and responds
    affirmatively to the question of the purpose of this package:
    which of the things that the image sensor of the pandora ugv
    locates as potential holes are indeed holes.
    @param[in] probabilitiesVector2D
    [const std::vector<std::vector<float> >&]
    A two dimensional vector containing the probabilities of
    validity of each candidate hole. Each row of it pertains to a specific
    filter applied, each column to a particular hole
    @return [std::map<int, float>] The indices of the valid holes and their
    respective validity probabilities
   **/
  std::map<int, float> HoleFusion::validateHoles(
    const std::vector<std::vector<float> >& probabilitiesVector2D)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateHoles", "processCandidateHoles");
    #endif

    // The map of holes' indices that are valid and
    // their respective validity probability that will be returned
    std::map<int, float> valid;

    for (int i = 0; i < probabilitiesVector2D[0].size(); i++)
    {
      int exponent = 0;
      float sum = 0.0;

      // The number of RGB-based checkers that are active.
      // This is needed as an offset for the location of the row in which a
      // depth filter is located inside the probabilitiesVector2D,
      // because, due to the fact that the depth sensor has a limited minimum
      // operational range, it may not be always possible to run depth-based
      // checkers. Depth-based checkers are hence run conditionally,
      // depending on the interpolation method used for the input depth image
      // and their presence is not always certain in the probabilitiesVector2D
      int rgbActiveFilters = 0;
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_color_homogeneity > 0)
        {
          rgbActiveFilters++;
        }
        if (Parameters::HoleFusion::run_checker_luminosity_diff > 0)
        {
          rgbActiveFilters++;
        }
        if (Parameters::HoleFusion::run_checker_texture_diff > 0)
        {
          rgbActiveFilters++;
        }
        if (Parameters::HoleFusion::run_checker_texture_backproject > 0)
        {
          rgbActiveFilters++;
        }
      }

      // Commence setting of priorities given to hole checkers.
      // Each priority given is not fixed, but there is an apparent hierarchy
      // witnessed here.
      // In order to reach a valid conclusion, an analytical method had to be
      // used, which is one analogous to the one presented in {insert link}

      // The depth homogeneity is considered the least confident measure of
      // a potential hole's validity. Hence,
      // Priotity{depth_homogeneity} = 0
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_depth_homogeneity > 0)
        {
          sum += pow(2, exponent) * probabilitiesVector2D[rgbActiveFilters
            + Parameters::HoleFusion::run_checker_depth_homogeneity - 1][i];

          exponent++;
        }
      }

      // Priotity{texture_diff} = 1
      if (Parameters::HoleFusion::run_checker_texture_diff > 0)
      {
        sum += pow(2, exponent) * probabilitiesVector2D[
          Parameters::HoleFusion::run_checker_texture_diff - 1][i];

        exponent++;
      }

      // Priotity{texture_backproject} = 2
      if (Parameters::HoleFusion::run_checker_texture_backproject > 0)
      {
        sum += pow(2, exponent) * probabilitiesVector2D[
          Parameters::HoleFusion::run_checker_texture_backproject - 1][i];

        exponent++;
      }

      // Priotity{color_homogeneity} = 3
      if (Parameters::HoleFusion::run_checker_color_homogeneity > 0)
      {
        sum += pow(2, exponent) * probabilitiesVector2D[
          Parameters::HoleFusion::run_checker_color_homogeneity - 1][i];

        exponent++;
      }

      // Priotity{luminosity_diff} = 4
      if (Parameters::HoleFusion::run_checker_luminosity_diff > 0)
      {
        sum += pow(2, exponent) * probabilitiesVector2D[
          Parameters::HoleFusion::run_checker_luminosity_diff - 1][i];

        exponent++;
      }

      // Priotity{brushfire_outline_to_rectangle} = 5
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle > 0)
        {
          sum += pow(2, exponent) * probabilitiesVector2D[rgbActiveFilters
            + Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle - 1][i];

          exponent++;
        }
      }

      // Priotity{outline_of_rectangle} = 6
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_outline_of_rectangle > 0)
        {
          sum += pow(2, exponent) * probabilitiesVector2D[rgbActiveFilters
            + Parameters::HoleFusion::run_checker_outline_of_rectangle - 1][i];

          exponent++;
        }
      }

      // Priotity{depth_diff} = 7
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_depth_diff > 0)
        {
          sum += pow(2, exponent) * probabilitiesVector2D[rgbActiveFilters
            + Parameters::HoleFusion::run_checker_depth_diff - 1][i];

          exponent++;
        }
      }

      // Priotity{depth_area} = 8
      if (Parameters::Depth::interpolation_method == 0)
      {
        if (Parameters::HoleFusion::run_checker_depth_area > 0)
        {
          sum += pow(2, exponent) * probabilitiesVector2D[rgbActiveFilters
            + Parameters::HoleFusion::run_checker_depth_area - 1][i];

          exponent++;
        }
      }

      sum /= (pow(2, exponent) - 1);

      // The validity acceptance threshold
      float threshold = 0.0;
      if (Parameters::Depth::interpolation_method == 0)
      {
        threshold = Parameters::HoleFusion::holes_validity_threshold_normal;
      }
      else
      {
        threshold = Parameters::HoleFusion::holes_validity_threshold_urgent;
      }

      if (sum > threshold)
      {
        valid[i] = sum;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("validateHoles");
    #endif

    return valid;
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

    // Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(20, 20), cv::Point2f(30, 30), 50, 50, 30, 30, dummy);

    // Invalid
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(80, 80), cv::Point2f(90, 90), 50, 50, 30, 30, dummy);


    // 0-th assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(370.0, 130.0), cv::Point2f(372.0, 132.0), 80, 80, 76, 76,
      dummy);

    // 0-th overlapper
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(372.0, 132.0), cv::Point2f(374.0, 134.0), 80, 80, 76, 76,
      dummy);

    // 0-th assimilable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(380.0, 140.0), cv::Point2f(382.0, 142.0), 20, 20, 16, 16,
      dummy);

    // 0-th amalgamatable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(420.0, 140.0), cv::Point2f(422.0, 142.0), 120, 40, 116, 36,
      dummy);

    // 0-th connectable
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(510.0, 80.0), cv::Point2f(512.0, 82.0), 40, 40, 36, 36,
      dummy);


    // 1-st assimilator - amalgamator - connector
    HolesConveyorUtils::appendDummyConveyor(
      cv::Point2f(300.0, 300.0), cv::Point2f(302.0, 302.0), 100, 100, 96, 96,
      dummy);

    // 1-st connectable
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
