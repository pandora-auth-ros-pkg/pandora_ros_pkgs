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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rgb_node/rgb.h"

PLUGINLIB_EXPORT_CLASS(pandora_vision::pandora_vision_hole::rgb::Rgb, nodelet::Nodelet)

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace rgb
{
  /**
    @brief Constructor
  **/
  Rgb::
  Rgb() {}

  /**
    @brief Destructor
   **/
  Rgb::
  ~Rgb()
  {
    NODELET_INFO("[%s] Terminated", nodeName_.c_str());
  }

  void
  Rgb::
  onInit()
  {
    state_manager::StateClientNodelet::onInit();
    nodeHandle_ = this->getNodeHandle();
    privateNodeHandle_ = this->getPrivateNodeHandle();
    nodeName_ = boost::to_upper_copy<std::string>(this->getName());
    serverPtr_.reset(new dynamic_reconfigure::Server<
        ::pandora_vision_hole::rgb_cfgConfig>(privateNodeHandle_));

    // Acquire the names of topics which the rgb node will be having
    // transactionary affairs with
    getTopicNames();

    // Calculate the vector of histograms of images of wooden walls:
    // the rgb node, depending on the procedure of edges extraction of
    // rgb image, is going to need it
    Histogram::getHistogram(&wallsHistogram_,
      Parameters::Histogram::secondary_channel);

    // Subscribe to the RGB image published by the
    // rgb_depth_synchronizer node
    rgbImageSubscriber_= nodeHandle_.subscribe(rgbImageTopic_, 1,
      &Rgb::inputRgbImageCallback, this);

    // Advertise the candidate holes found by the rgb node
    candidateHolesPublisher_ = nodeHandle_.advertise
      < ::pandora_vision_hole::CandidateHolesVectorMsg >(
      candidateHolesTopic_, 1);

    // The dynamic reconfigure (RGB) parameter's callback
    serverPtr_->setCallback(boost::bind(&Rgb::parametersCallback, this, _1, _2));

    clientInitialize();

    NODELET_INFO("[%s] Initiated", nodeName_.c_str());
  }

  /**
    @brief Completes the transition to a new state
    @param void
    @return void
   **/
  void Rgb::completeTransition(void)
  {
    NODELET_INFO("[%s] Transition Complete", nodeName_.c_str());
  }

  void Rgb::startTransition(int newState)
  {
    if (newState == state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD)
    {
      Parameters::Edge::edge_detection_method = 0;
    }
    else
    {
      Parameters::Edge::edge_detection_method = 1;
    }
    transitionComplete(newState);
  }


  /**
    @brief Callback for the rgb image received by the synchronizer node.

    The rgb image message received by the synchronizer node is unpacked
    in a cv::Mat image. Holes are then located inside this image and
    information about them, along with the rgb image, is then sent to the
    hole fusion node
    @param msg [const sensor_msgs::Image&] The rgb image message
    @return void
  **/
  void
  Rgb::
  inputRgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputRgbImageCallback", "", true);
    #endif

    // Obtain the rgb image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be CV_8UC3.
    cv::Mat rgbImage;
    MessageConversions::extractImageFromMessage(*msg, &rgbImage,
      sensor_msgs::image_encodings::BGR8);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_rgb_image)
    {
      Visualization::show("RGB image", rgbImage, 1);
    }
    #endif

    // Regardless of the image representation method, the RGB node
    // will publish the RGB image of original size to the Hole Fusion node
    cv::Mat rgbImageSent;
    rgbImage.copyTo(rgbImageSent);

    // A value of 1 means that the rgb image is subtituted by its
    // low-low, wavelet analysis driven, part
    if (Parameters::Image::image_representation_method == 1)
    {
      // Obtain the low-low part of the rgb image via wavelet analysis
      Wavelets::getLowLow(rgbImage, &rgbImage);
    }

    // Locate potential holes in the rgb image
    HolesConveyor conveyor = RgbHoleDetector::findHoles(rgbImage, wallsHistogram_);

    // Create the candidate holes message
    ::pandora_vision_hole::CandidateHolesVectorMsgPtr
      rgbCandidateHolesMsgPtr( new ::pandora_vision_hole::CandidateHolesVectorMsg );

    // Pack information about holes found and the rgb image inside a message.
    // This message will be published to and received by the hole fusion node
    MessageConversions::createCandidateHolesVectorMessage(
        conveyor,
        rgbImageSent,
        rgbCandidateHolesMsgPtr,
        sensor_msgs::image_encodings::TYPE_8UC3, *msg);

    // Publish the candidate holes message
    candidateHolesPublisher_.publish(rgbCandidateHolesMsgPtr);

    #ifdef DEBUG_TIME
    Timer::tick("inputRgbImageCallback");
    Timer::printAllMeansTree();
    #endif
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the rgb node
    @param void
    @return void
   **/
  void
  Rgb::
  getTopicNames()
  {
    // Read the name of the topic from where the rgb node acquires the
    // rgb image and store it in a private member variable
    if (!privateNodeHandle_.getParam("subscribed_topics/rgb_image_topic",
        rgbImageTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic rgb_image_topic", nodeName_.c_str());
      ROS_BREAK();
    }
    // Read the name of the topic to which the rgb node will be publishing
    // information about the candidate holes found and store it in a private
    // member variable
    if (!privateNodeHandle_.getParam("published_topics/candidate_holes_topic",
        candidateHolesTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic candidate_holes_topic", nodeName_.c_str());
      ROS_BREAK();
    }
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole::rgb_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void
  Rgb::
  parametersCallback(
      const ::pandora_vision_hole::rgb_cfgConfig& config,
      const uint32_t& level)
  {
    NODELET_INFO("[%s] Parameters callback called", nodeName_.c_str());
    //////////////////// Blob detection - specific parameters //////////////////

    Parameters::Blob::min_threshold =
      config.min_threshold;

    Parameters::Blob::max_threshold =
      config.max_threshold;

    Parameters::Blob::threshold_step =
      config.threshold_step;

    // In wavelet mode, the image shrinks by a factor of 4
    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Blob::min_area =
        config.min_area;

      Parameters::Blob::max_area =
        config.max_area;
    }
    else if (Parameters::Image::image_representation_method == 1)
    {
      Parameters::Blob::min_area =
        static_cast<int>(config.min_area / 4);

      Parameters::Blob::max_area =
        static_cast<int>(config.max_area / 4);
    }

    Parameters::Blob::min_convexity =
      config.min_convexity;

    Parameters::Blob::max_convexity =
      config.max_convexity;

    Parameters::Blob::min_inertia_ratio =
      config.min_inertia_ratio;

    Parameters::Blob::max_circularity =
      config.max_circularity;

    Parameters::Blob::min_circularity =
      config.min_circularity;

    Parameters::Blob::filter_by_color =
      config.filter_by_color;

    Parameters::Blob::filter_by_circularity =
      config.filter_by_circularity;


    ////////////////////////////// Debug parameters ////////////////////////////

    // Show the rgb image that arrives in the rgb node
    Parameters::Debug::show_rgb_image =
      config.show_rgb_image;

    Parameters::Debug::show_find_holes =
      config.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      config.show_find_holes_size;

    Parameters::Debug::show_produce_edges =
      config.show_produce_edges;
    Parameters::Debug::show_produce_edges_size =
      config.show_produce_edges_size;

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


    //////////////////// Parameters specific to the RGB node ///////////////////


    //------------------- Edge detection specific parameters -------------------

    // The opencv edge detection method:
    // 0 for the Canny edge detector
    // 1 for the Scharr edge detector
    // 2 for the Sobel edge detector
    // 3 for the Laplacian edge detector
    // 4 for mixed Scharr / Sobel edge detection
    Parameters::Edge::edge_detection_method =
      config.edge_detection_method;

    Parameters::Edge::denoised_edges_threshold =
      config.denoised_edges_threshold;

    // Canny parameters
    Parameters::Edge::canny_ratio =
      config.canny_ratio;

    Parameters::Edge::canny_kernel_size =
      config.canny_kernel_size;

    Parameters::Edge::canny_low_threshold =
      config.canny_low_threshold;

    Parameters::Edge::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;


    //------------- Parameters needed for histogram calculation ----------------

    Parameters::Histogram::number_of_hue_bins =
      config.number_of_hue_bins;

    Parameters::Histogram::number_of_saturation_bins =
      config.number_of_saturation_bins;

    Parameters::Histogram::number_of_value_bins =
      config.number_of_value_bins;

    Parameters::Histogram::secondary_channel =
      config.secondary_channel;


    //----------------- Outline discovery specific parameters ------------------

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


    //------------------- Loose ends connection parameters ---------------------

    Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    // In wavelet mode, the image shrinks by a factor of 4
    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Outline::minimum_curve_points =
        config.minimum_curve_points;
    }
    else if (Parameters::Image::image_representation_method == 1)
    {
      Parameters::Outline::minimum_curve_points =
        static_cast<int>(config.minimum_curve_points / 4);
    }


    // Selects the method for extracting a RGB image's edges.
    // Choices are via segmentation and via backprojection
    Parameters::Rgb::edges_extraction_method =
      config.edges_extraction_method;

    //------------------- RGB image segmentation parameters --------------------

    // Parameters specific to the pyrMeanShiftFiltering method
    Parameters::Rgb::spatial_window_radius =
      config.spatial_window_radius;
    Parameters::Rgb::color_window_radius =
      config.color_window_radius;
    Parameters::Rgb::maximum_level_pyramid_segmentation =
      config.maximum_level_pyramid_segmentation;

    // Term criteria for the pyrMeanShiftFiltering method
    Parameters::Image::term_criteria_max_iterations =
      config.term_criteria_max_iterations;
    Parameters::Image::term_criteria_max_epsilon =
      config.term_criteria_max_epsilon;

    // True to posterize the product of the segmentation
    Parameters::Rgb::posterize_after_segmentation =
      config.posterize_after_segmentation;

    // FloodFill options regarding minimum and maximum colour difference
    Parameters::Rgb::floodfill_lower_colour_difference =
      config.floodfill_lower_colour_difference;
    Parameters::Rgb::floodfill_upper_colour_difference =
      config.floodfill_upper_colour_difference;

    //------------ RGB image edges via backprojection parameters ---------------

    // The threshold applied to the backprojection of the RGB image
    // captured by the image sensor
    Parameters::Rgb::backprojection_threshold =
      config.backprojection_threshold;

    // Watershed-specific parameters
    Parameters::Rgb::watershed_foreground_dilation_factor =
      config.watershed_foreground_dilation_factor;
    Parameters::Rgb::watershed_foreground_erosion_factor =
      config.watershed_foreground_erosion_factor;
    Parameters::Rgb::watershed_background_dilation_factor =
      config.watershed_background_dilation_factor;
    Parameters::Rgb::watershed_background_erosion_factor =
      config.watershed_background_erosion_factor;
  }

}  // namespace rgb
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
