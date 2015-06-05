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
 * Authors: Vasilis Bosdelekidis, Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "hole_fusion_node/hole_fusion.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief The HoleFusion constructor
   **/
  HoleFusion::HoleFusion(void):
    pointCloud_(new PointCloud),
    nodeHandle_(""),
    generalNodeHandle_("~/general"),
    debugNodeHandle_("~/debug"),
    validityNodeHandle_("~/validation"),
    serverDebug(debugNodeHandle_),
    serverGeneral(generalNodeHandle_),
    serverValidity(validityNodeHandle_),
    imageTransport_(nodeHandle_)
  {
    // Initialize the parent frame_id to an empty string
    parent_frame_id_ = "";

    // Acquire the names of topics which the Hole Fusion node will be having
    // transactionary affairs with
    getTopicNames();

    // Initialize the numNodesReady variable.
    // It acts as a counter of nodes that have published their output to the
    // hole fusion node in each execution cycle.
    numNodesReady_ = 0;

    // Advertise the topic that the rgb_depth_synchronizer will be
    // subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>(
        unlockTopic_, 1000, true);

    // Advertise the topic where any external node(e.g. a functional test node)
    // will be subscribed to know that the hole node has finished processing
    // the current candidate holes as well as the result of the procedure.
    processEndPublisher_ = nodeHandle_.advertise<sensor_processor::ProcessorLogInfo>(
        processEndTopic_, 1000, true);

    // Advertise the topic that the yaw and pitch of the keypoints of the final,
    // valid holes will be published to
    validHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::HoleDirectionAlertVector>(
          validHolesTopic_, 10, true);

    // Advertise the topic that information about the final holes,
    // will be published to
    // Command line usage:
    // image_view /pandora_vision/hole_detector/debug_valid_holes_image
    // _image_transport:=compressed
    debugValidHolesPublisher_ = imageTransport_.advertise
      (debugValidHolesTopic_, 1, true);

    // Advertise the topic that information about holes found by the Depth
    // and RGB nodes will be published to
    // Command line usage:
    // image_view /pandora_vision/hole_detector/debug_respective_holes_image
    // _image_transport:=compressed
    debugRespectiveHolesPublisher_ = imageTransport_.advertise
      (debugRespectiveHolesTopic_, 1, true);

    // Advertise the topic that the image of the final holes,
    // will be published to
    enhancedHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::EnhancedImage>(
          enhancedHolesTopic_, 1000, true);

    // Advertise the topic where the Hole Fusion node requests from the
    // synchronizer node to subscribe to the input point cloud topic
    synchronizerSubscribeToInputPointCloudPublisher_ =
      nodeHandle_.advertise <std_msgs::Empty>(
          synchronizerSubscribeToInputPointCloudTopic_, 1000, true);

    // Advertise the topic where the Hole Fusion node requests from the
    // synchronizer node to leave its subscription to the
    // input point cloud topic
    synchronizerLeaveSubscriptionToInputPointCloudPublisher_=
      nodeHandle_.advertise <std_msgs::Empty>(
          synchronizerLeaveSubscriptionToInputPointCloudTopic_, 1000, true);

    // Subscribe to the topic where the synchronizer node publishes
    // the depth image
    depthImageSubscriber_= nodeHandle_.subscribe(
        depthImageTopic_, 1,
        &HoleFusion::depthImageCallback, this);

    // Subscribe to the topic where the depth node publishes
    // candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
        depthCandidateHolesTopic_, 1,
        &HoleFusion::depthCandidateHolesCallback, this);

    // Subscribe to the topic where the synchronizer node publishes
    // the rgb image
    rgbImageSubscriber_= nodeHandle_.subscribe(
        rgbImageTopic_, 1,
        &HoleFusion::rgbImageCallback, this);

    // Subscribe to the topic where the rgb node publishes
    // candidate holes
    rgbCandidateHolesSubscriber_= nodeHandle_.subscribe(
        rgbCandidateHolesTopic_, 1,
        &HoleFusion::rgbCandidateHolesCallback, this);

    // Subscribe to the topic where the thermal node publishes
    // candidate holes
    //thermalCandidateHolesSubscriber_= nodeHandle_.subscribe(
    //    thermalCandidateHolesTopic_, 1,
    //    &HoleFusion::thermalCandidateHolesCallback, this);

    // Subscribe to the topic where the synchronizer node publishes
    // the point cloud
    pointCloudSubscriber_= nodeHandle_.subscribe(
        pointCloudTopic_, 1,
        &HoleFusion::pointCloudCallback, this);


    // The dynamic reconfigure server for debugging parameters
    serverDebug.setCallback(
        boost::bind(&HoleFusion::parametersCallbackDebug,
          this, _1, _2));

    // The dynamic reconfigure server for general parameters
    serverGeneral.setCallback(
        boost::bind(&HoleFusion::parametersCallbackGeneral,
          this, _1, _2));

    // The dynamic reconfigure server for parameters pertaining to
    // the validity of holes
    serverValidity.setCallback(
        boost::bind(&HoleFusion::parametersCallbackValidity,
          this, _1, _2));

    // Set the initial on/off state of the Hole Detector package to off
    isOn_ = false;

    // Initialize the filtering mode variable to an invalid value
    filteringMode_ = -1;

    // Calculate the collective histogram of images of walls needed
    // for comparing against the one of images of the material surrounding
    // candidate holes
    //Histogram::getHistogram(&wallsHistogram_,
    //  Parameters::Histogram::secondary_channel);

    clientInitialize();

    ROS_INFO_NAMED(PKG_NAME, "[Hole Fusion node] Initiated");
  }



  /**
    @brief The HoleFusion deconstructor
   **/
  HoleFusion::~HoleFusion(void)
  {
    ros::shutdown();
    ROS_INFO_NAMED(PKG_NAME, "[Hole Fusion node] Terminated");
  }



  /**
    @brief Completes the transition to a new state
    @param void
    @return void
   **/
  void HoleFusion::completeTransition(void)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Hole Detector] : Transition Complete");
  }


  /**
    @brief Callback for a depth image via the synchronizer node.
    This method sets the interpolated depth image which will be used 
    for hole validation reasons, once there are any candidate holes
    from rgb or depth nodes.
    @param[in] image [const sensor_msgs::Image&] The depth image msg
    @return void
   **/
  void HoleFusion::depthImageCallback(
      const sensor_msgs::Image& image)
  {
    // Obtain the depth image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be TYPE_32FC1.
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
    interpolatedDepthImage_ = in_msg->image.clone();
  }


  /**
    @brief Callback for the candidate holes via the depth node.

    This method sets the candidate holes acquired from the depth node.
    If all the other callbacks (e.g. rgb, thermal) have done
    what must be, it resets the number of ready nodes, unlocks
    the synchronizer and calls for processing of the candidate
    holes.
    @param[in] depthCandidateHolesVector
    [const pandora_vision_msgs::RegionOfInterestVector&]
    The message containing the necessary information acquired through
    the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
      const pandora_vision_msgs::RegionOfInterestVector&
      depthCandidateHolesVector)
  {
#ifdef DEBUG_TIME
    Timer::start("depthCandidateHolesCallback", "", true);
#endif

    ROS_INFO_NAMED(PKG_NAME, "Hole Fusion Depth callback");

    // Clear the current depthHolesConveyor struct
    // (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&depthHolesConveyor_);

    // Unpack the message
    MessageConversions::unpackMessage(depthCandidateHolesVector,
        &depthHolesConveyor_,
        interpolatedDepthImage_,
        sensor_msgs::image_encodings::TYPE_32FC1);

    // The candidate holes acquired from the depth node and the interpolated
    // depth image are set
    numNodesReady_++;

    // If the RGB candidate holes and the RGB image are set
    // and the point cloud has been delivered and interpolated,
    // unlock the synchronizer and process the candidate holes from both sources
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
    @brief Callback for the candidate holes via the thermal node.

    This method sets the thermal image and the
    candidate holes acquired from the thermal node.
    If the depth and rgb callback counterparts have done
    what must be, it resets the number of ready nodes, unlocks
    the synchronizer and calls for processing of the candidate
    holes.
    @param[in] thermalCandidateHolesVector
    [const pandora_vision_msgs::CandidateHolesVectorMsg&]
    The message containing the necessary information acquired through
    the thermal node
    @return void
   **/
  //void HoleFusion::thermalCandidateHolesCallback(
  //    const pandora_vision_msgs::CandidateHolesVectorMsg&
  //    thermalCandidateHolesVector)
  //{
  //ifdef DEBUG_TIME
  //  Timer::start("thermalCandidateHolesCallback", "", true);
  //endif

  //  ROS_INFO_NAMED(PKG_NAME, "Hole Fusion Thermal callback");

  //  // Clear the current thermalHolesConveyor struct
  //  // (or else keyPoints, rectangles and outlines accumulate)
  //  HolesConveyorUtils::clear(&thermalHolesConveyor_);

  //  // Unpack the message
  //  MessageConversions::unpackHoleDetectorMessage(thermalCandidateHolesVector,
  //      &thermalHolesConveyor_,
  //      &interpolatedThermalImage_,
  //      sensor_msgs::image_encodings::TYPE_32FC1);

  //  // The candidate holes acquired from the thermal node and the
  //  // thermal image are set
  //  numNodesReady_++;

  //  // If the Thermal candidate holes and the Thermal image are set
  //  // and the point cloud has been delivered and interpolated,
  //  // unlock the synchronizer and process the candidate holes from all sources
  //  ROS_INFO("nodesReady %d \n", numNodesReady_);
  //  if (numNodesReady_ == 2)
  //  {
  //    numNodesReady_ = 0;

  //    unlockSynchronizer();

  //    processCandidateHoles();
  //  }

  //ifdef DEBUG_TIME
  //  Timer::tick("thermalCandidateHolesCallback");
  //  Timer::printAllMeansTree();
  //endif
  //}


  /**
    @brief Callback for a rgb image via the synchronizer node.
    This method sets the rgb image which will be used 
    for hole validation reasons, once there are any candidate holes
    from rgb or depth nodes.
    @param[in] image [const sensor_msgs::Image&] The rgb image msg
    @return void
   **/
  void HoleFusion::rgbImageCallback(
      const sensor_msgs::Image& image)
  {
    // Obtain the rgb image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be BGR8.
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    rgbImage_ = in_msg->image.clone();
  }


  /**
    @brief Callback for the candidate holes via the rgb node

    This method sets the candidate holes acquired
    from the rgb node.
    If the other callbacks (e.g. depth, thermal) have done
    what must be, it resets the number of ready nodes, unlocks
    the synchronizer and calls for processing of the candidate
    holes.
    @param[in] rgbCandidateHolesVector
    [const pandora_vision_msgs::RegionOfInterestVector&]
    The message containing the necessary information to filter hole
    candidates acquired through the rgb node
    @return void
   **/
  void HoleFusion::rgbCandidateHolesCallback(
      const pandora_vision_msgs::RegionOfInterestVector&
      rgbCandidateHolesVector)
  {
#ifdef DEBUG_TIME
    Timer::start("rgbCandidateHolesCallback", "", true);
#endif

    ROS_INFO_NAMED(PKG_NAME, "Hole Fusion RGB callback");

    // Clear the current rgbHolesConveyor struct
    // (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&rgbHolesConveyor_);

    // Unpack the message
    MessageConversions::unpackMessage(rgbCandidateHolesVector,
        &rgbHolesConveyor_,
        rgbImage_,
        sensor_msgs::image_encodings::TYPE_8UC3);

    // The candidate holes acquired from the rgb node and the rgb image are set
    numNodesReady_++;

    // If the depth candidate holes and the interpolated depth image are set
    // and the point cloud has been delivered and interpolated,
    // unlock the synchronizer and process the candidate holes from both sources
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
    @brief Retrieves the parent to the frame_id of the input point cloud,
    so as to set the frame_id of the output messages of valid holes.
    @param void
    @return void
   **/
  void HoleFusion::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";

    bool res = nodeHandle_.hasParam(model_param_name);

    std::string robot_description = "";

    // The parameter was not found.
    // Set the parent of the frame_id to a default value.
    if(!res || !nodeHandle_.getParam(model_param_name, robot_description))
    {
      ROS_ERROR_NAMED(PKG_NAME,
          "Robot description couldn't be retrieved from the parameter server.");

      parent_frame_id_ = "kinect_rgb_frame";

      return;
    }

    boost::shared_ptr<urdf::ModelInterface> model(
        urdf::parseURDF(robot_description));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> currentLink = model->getLink(frame_id_);

    boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();

    // Set the parent frame_id to the parent of the frame_id_
    parent_frame_id_ = parentLink->name;
  }



  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the Hole Fusion node
    @param void
    @return void
   **/
  void HoleFusion::getTopicNames ()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the Hole Fusion node acquires the
    // input point cloud
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/point_cloud_internal_topic",
          pointCloudTopic_))
    {
      // Make the topic's name absolute
      pointCloudTopic_ = ns + "/" + pointCloudTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the internal point cloud topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic point_cloud_internal_topic");
    }

    // Read the name of the topic from where the Hole Fusion node acquires the
    // depth images originated from the synchronizer node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/depth_image_topic",
          depthImageTopic_))
    {
      // Make the topic's name absolute
      depthImageTopic_ = ns + "/" + depthImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the Depth images topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic depth_image_topic");
    }

    // Read the name of the topic from where the Hole Fusion node acquires the
    // candidate holes originated from the Depth node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/depth_candidate_holes_topic",
          depthCandidateHolesTopic_))
    {
      // Make the topic's name absolute
      depthCandidateHolesTopic_ = ns + "/" + depthCandidateHolesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the Depth candidate holes topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic depth_candidate_holes_topic");
    }

    // Read the name of the topic from where the Hole Fusion node acquires the
    // rgb images originated from the synchronizer node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/rgb_image_topic",
          rgbImageTopic_))
    {
      // Make the topic's name absolute
      rgbImageTopic_ = ns + "/" + rgbImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the Rgb images topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic rgb_image_topic");
    }

    // Read the name of the topic from where the Hole Fusion node acquires the
    // candidate holes originated from the Rgb node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/rgb_candidate_holes_topic",
          rgbCandidateHolesTopic_))
    {
      // Make the topic's name absolute
      rgbCandidateHolesTopic_ = ns + "/" + rgbCandidateHolesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the Rgb candidate holes topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic rgb_candidate_holes_topic");
    }

    // Read the name of the topic from where the Hole Fusion node acquires the
    // candidate holes originated from the Thermal node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/subscribed_topics/thermal_candidate_holes_topic",
          thermalCandidateHolesTopic_))
    {

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Subscribed to the Thermal candidate holes topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic thermal_candidate_holes_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to unlock
    // the synchronizer node
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/published_topics/synchronizer_unlock_topic",
          unlockTopic_))
    {
      // Make the topic's name absolute
      unlockTopic_ = ns + "/" + unlockTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to the unlock topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic synchronizer_unlock_topic");
    }

    // Get the topic where the result of the hole processing will be
    // published.
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/published_topics/processor_log_topic",
          processEndTopic_))
    {
      // Make the topic's name absolute
      processEndTopic_ = ns + "/" + processEndTopic_;

      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to the Process End topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic Process end Topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // information about the valid holes found by the Hole Detector package
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/published_topics/hole_detector_output_topic",
          validHolesTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to the valid holes topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic hole_detector_output_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // additional information about the valid holes found by the
    // Hole Detector package
    if (nodeHandle_.getParam(
          ns + "/hole_fusion_node/published_topics/enhanced_holes_topic",
          enhancedHolesTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to the enhanced holes topic");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic enhanced_holes_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // messages so that the synchronizer node subscribes to the
    // input point cloud
    if (nodeHandle_.getParam(ns +
          "/hole_fusion_node/published_topics/make_synchronizer_subscribe_to_input",
          synchronizerSubscribeToInputPointCloudTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to topic where the synchronizer"
          " expects messages dictating its subscription to the input point cloud");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME, "[Hole Fusion Node] Could not find topic"
          " make_synchronizer_subscribe_to_input");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // messages so that the synchronizer node leaves its subscription to the
    // input point cloud
    if (nodeHandle_.getParam(ns +
          "/hole_fusion_node/published_topics/make_synchronizer_leave_subscription_to_input",
          synchronizerLeaveSubscriptionToInputPointCloudTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to topic where the synchronizer"
          " expects messages dictating its leave of subscription to the"
          " input point cloud");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME, "[Hole Fusion Node] Could not find topic"
          " make_synchronizer_leave_subscription_to_input");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // an image of holes found by the Depth and RGB nodes
    if (nodeHandle_.getParam(ns +
          "/hole_fusion_node/published_topics/debug_respective_holes_image",
          debugRespectiveHolesTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to topic where an image of the"
          " respective holes found is published");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic debug_respective_holes_image");
    }

    // Read the name of the topic that the Hole Fusion node uses to publish
    // an image of the valid holes found
    if (nodeHandle_.getParam(ns +
          "/hole_fusion_node/published_topics/debug_valid_holes_image",
          debugValidHolesTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
          "[Hole Fusion Node] Advertising to topic where an image of the"
          " valid holes is published");
    }
    else
    {
      ROS_INFO_NAMED (PKG_NAME,
          "[Hole Fusion Node] Could not find topic debug_valid_holes_image");
    }
  }



  /**
    @brief Computes the on/off state of the Hole Detector package,
    given a state
    @param[in] state [const int&] The robot's state
    @return [bool] True if the hole fusion node's state is set to "on"
   **/
  bool HoleFusion::isHoleDetectorOn(const int& state)
  {
    return (state ==
        state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE)
      || (state ==
          state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION)
      || (state ==
          state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD)
      || (state ==
          state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST);
  }



  ///**
  //  @brief The function called when a debugging parameter is changed
  //  @param[in] config
  //  [const pandora_vision_hole::debug_cfgConfig&]
  //  @param[in] level [const uint32_t]
  //  @return void
  // **/
  void HoleFusion::parametersCallbackDebug(
      const pandora_vision_hole_exploration::debug_cfgConfig &config,
      const uint32_t& level)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Hole Fusion node] Parameters callback called");

    //////////////////////////////// Debug parameters ////////////////////////////

    // Show the holes that each of the depth and RGB nodes transmit to the
    // hole fusion node, on top of their respective origin images
    Parameters::Debug::show_respective_holes =
      config.show_respective_holes;

    //  // Show all valid holes, from either the Depth or RGB source, or
    //  // the merges between them
    Parameters::Debug::show_valid_holes =
      config.show_valid_holes;

    //  // The product of this package: unique, valid holes
    //  Parameters::Debug::show_final_holes =
    //   config.show_final_holes;

    //  // In the terminal's window, show the probabilities of candidate holes
    Parameters::Debug::show_probabilities =
      config.show_probabilities;

    //  // Show the texture's watersheded backprojection
    //  Parameters::Debug::show_texture =
    //    config.show_texture;

    //  Parameters::Debug::show_find_holes =
    //    config.show_find_holes;
    //  Parameters::Debug::show_find_holes_size =
    //    config.show_find_holes_size;
    //  Parameters::Debug::show_denoise_edges =
    //    config.show_denoise_edges;
    //  Parameters::Debug::show_denoise_edges_size =
    //    config.show_denoise_edges_size;
    //  Parameters::Debug::show_connect_pairs =
    //    config.show_connect_pairs;
    //  Parameters::Debug::show_connect_pairs_size =
    //    config.show_connect_pairs_size;

    //  Parameters::Debug::show_get_shapes_clear_border  =
    //    config.show_get_shapes_clear_border;
    //  Parameters::Debug::show_get_shapes_clear_border_size =
    //    config.show_get_shapes_clear_border_size;

    //  Parameters::Debug::show_check_holes =
    //    config.show_check_holes;
    //  Parameters::Debug::show_check_holes_size =
    //    config.show_check_holes_size;

    //  Parameters::Debug::show_merge_holes =
    //    config.show_merge_holes;
    //  Parameters::Debug::show_merge_holes_size =
    //    config.show_merge_holes_size;
  }



  /**
    @brief The function called when a general parameter is changed
    @param[in] config
    [const pandora_vision_hole::general_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void HoleFusion::parametersCallbackGeneral(
      const pandora_vision_hole_exploration::general_cfgConfig &config,
      const uint32_t& level)
  {
    //  Parameters::Rgb::watershed_background_erosion_factor =
    //    config.watershed_background_erosion_factor;

  }


  /**
    @brief The function called when a validity parameter is changed
    @param[in] config
    [const pandora_vision_hole_exploration::validity_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void HoleFusion::parametersCallbackValidity(
      const pandora_vision_hole_exploration::validity_cfgConfig &config,
      const uint32_t& level)
  {
    Parameters::HoleFusion::merging_distance_thresh =
      config.merging_distance_thresh;
    Parameters::HoleFusion::valid_strong_probability =
      config.valid_strong_probability;
    Parameters::HoleFusion::valid_medium_probability =
      config.valid_medium_probability;
    Parameters::HoleFusion::valid_light_probability =
      config.valid_light_probability;
    Parameters::HoleFusion::max_depth_to_test_small_thresh =
      config.max_depth_to_test_small_thresh;
    Parameters::HoleFusion::min_depth_to_test_big_thresh =
      config.min_depth_to_test_big_thresh;
    Parameters::HoleFusion::small_rect_thresh =
      config.small_rect_thresh;
    Parameters::HoleFusion::big_rect_thresh =
      config.big_rect_thresh;
    Parameters::HoleFusion::rgb_distance_variance_thresh = 
      config.rgb_distance_variance_thresh;
    Parameters::HoleFusion::rgb_small_distance_variance_thresh = 
      config.rgb_small_distance_variance_thresh;
    Parameters::HoleFusion::hole_border_thresh = 
      config.hole_border_thresh;
    Parameters::HoleFusion::depth_difference_thresh = 
      config.depth_difference_thresh;
    Parameters::HoleFusion::remove_unstuffed_holes = 
      config.remove_unstuffed_holes;
    Parameters::HoleFusion::unstuffed_removal_method = 
      config.unstuffed_removal_method;
    Parameters::HoleFusion::difference_scanline_thresh = 
      config.difference_scanline_thresh;

  }



  /**
    @brief Callback for the point cloud that the synchronizer node
    publishes.

    This method interpolates the input point cloud so that depth-based
    filters using it have an integral input and sets it and header-related
    variables.
    If the depth and RGB callback counterparts have done
    what must be, it resets the number of ready nodes, unlocks
    the synchronizer and calls for processing of the candidate
    holes.
    @param[in] msg [const PointCloudPtr&] The message containing
    the point cloud
    @return void
   **/
  void HoleFusion::pointCloudCallback(const PointCloudPtr& msg)
  {
#ifdef DEBUG_TIME
    Timer::start("pointCloudCallback", "", true);
#endif

    ROS_INFO_NAMED(PKG_NAME, "Hole Fusion Point Cloud callback");

    // Convert the header of the point cloud message
    std_msgs::Header header;
    pcl_conversions::fromPCL(msg->header, header);

    // Store the frame_id and timestamp of the point cloud under processing.
    // The respective variables in the headers of the published messages will
    // be set to these values.
    // Because the frame_id will be used to retrieve its parent frame_id from
    // the robot's description, and the frame_id starts with a forward slash,
    // remove it, in order for the search to be successful.
    frame_id_ = header.frame_id.substr(1);
    timestamp_ = header.stamp;

    // The parent frame_id cannot be set in the constructor because the
    // frame_id is not known until the first point cloud message arrives.
    // In order not to parse the urdf file every time,
    // set the parent frame_id string once
    if (parent_frame_id_.compare("") == 0)
    {
      getParentFrameId();
    }

    // Because the input point cloud is marked as const,
    // and we need to interpolate the noise in it,
    // copy the input point cloud to a local one.
    pcl::copyPointCloud(*msg, *pointCloud_);

    // Extract the depth image from the point cloud message
    cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
        msg, CV_32FC1);

    // Interpolate the depthImage
    cv::Mat interpolatedDepthImage;
    depthImage.copyTo(interpolatedDepthImage);

    // The noise elimination method above defines the interpolation method.
    // Only in interpolation_method of zero can the depth filters through which
    // each candidate hole is passed be utilized: there is no valid depth
    // information available if the value of interpolation_method is set to
    // other than zero.
    //if (Parameters::Depth::interpolation_method == 0)
    //{
    //  filteringMode_ = RGBD_MODE;
    //}
    //else
    //{
    //  filteringMode_ = RGB_ONLY_MODE;
    //}

    // Set the interpolatedDepthImage's values as the depth values
    // of the point cloud
    setDepthValuesInPointCloud(interpolatedDepthImage, &pointCloud_);

    // The interpolated point cloud, frame_id and timestamp are set
    numNodesReady_++;

    // If the depth and RGB candidate holes, the interpolated depth image
    // and the RGB image are set,
    // unlock the synchronizer and process the candidate holes from all sources
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
    @brief Implements a strategy to combine information from both
    the depth, thermal and rgb image and holes sources in order to accurately
    find valid holes.

    Each probability is a measure of each candidate hole's validity: the more a value of a probability, the more a candidate hole is indeed a hole in space. Finally, information about the valid holes is published.
    @param void
    @return void
   **/
  void HoleFusion::processCandidateHoles()
  {
    ROS_INFO_NAMED(PKG_NAME, "Processing candidate holes");

#ifdef DEBUG_TIME
    Timer::start("processCandidateHoles", "", true);
#endif

#ifdef DEBUG_SHOW
    if (Parameters::Debug::show_respective_holes)
    {
      std::vector<std::string> msgs;

      // Holes originated from analysis on the depth image,
      // on top of the depth image
      cv::Mat depthHolesOnDepthImage =
        Visualization::showHoles(
            "Holes originated from Depth analysis, on the Depth image",
            interpolatedDepthImage_,
            depthHolesConveyor_,
            -1,
            msgs);

      // Holes originated from thermal analysis on the depth image,
      // on top of the thermal image
      //cv::Mat thermalHolesOnDepthImage =
      //  Visualization::showHoles(
      //      "Holes originated from Thermal analysis, on the Depth image",
      //      interpolatedDepthImage_,
      //      thermalHolesConveyor_,
      //      -1,
      //      msgs);

      // Holes originated from analysis on the RGB image,
      // on top of the RGB image
      cv::Mat rgbHolesOnRgbImage =
        Visualization::showHoles(
            "Holes originated from RGB analysis, on the RGB image",
            rgbImage_,
            rgbHolesConveyor_,
            -1,
            msgs);

      // Holes originated from analysis on the depth image,
      // on top of the RGB image
      cv::Mat depthHolesOnRgbImage =
        Visualization::showHoles(
            "Holes originated from Depth analysis, on the RGB image",
            rgbImage_,
            depthHolesConveyor_,
            -1,
            msgs);

      // Holes originated from analysis on the thermal image,
      // on top of the RGB image
      //cv::Mat thermalHolesOnRgbImage =
      //  Visualization::showHoles(
      //      "Holes originated from Thermal analysis, on the RGB image",
      //      rgbImage_,
      //      thermalHolesConveyor_,
      //      -1,
      //      msgs);

      // Holes originated from analysis on the RGB image,
      // on top of the Depth image
      cv::Mat rgbHolesOnDepthImage =
        Visualization::showHoles(
            "Holes originated from RGB analysis, on the Depth image",
            interpolatedDepthImage_,
            rgbHolesConveyor_,
            -1,
            msgs);

      // The six images
      std::vector<cv::Mat> imgs;
      imgs.push_back(depthHolesOnDepthImage);
      imgs.push_back(depthHolesOnRgbImage);
      //imgs.push_back(thermalHolesOnDepthImage);
      //imgs.push_back(thermalHolesOnRgbImage);
      imgs.push_back(rgbHolesOnRgbImage);
      imgs.push_back(rgbHolesOnDepthImage);

      // The titles of the images
      std::vector<std::string> titles;

      titles.push_back("Holes originated from Depth analysis, on the Depth image");
      titles.push_back("Holes originated from Depth analysis, on the RGB image");
      //titles.push_back("Holes originated from Thermal analysis, on the Thermal image");
      //titles.push_back("Holes originated from Thermal analysis, on the RGB image");
      titles.push_back("Holes originated from RGB analysis, on the RGB image");
      titles.push_back("Holes originated from RGB analysis, on the Depth image");

      Visualization::multipleShow("Respective keypoints", imgs, titles, 1280, 1);
    }
#endif
    double min, max;
    cv::minMaxLoc(interpolatedDepthImage_, &min, &max);
    //cv::minMaxLoc(interpolatedThermalImage_, &min, &max);
    HolesConveyor preValidatedHoles;
    std::map<int, float> validHolesMap;

    mergeHoles(
        &rgbHolesConveyor_, 
        &depthHolesConveyor_, 
        &thermalHolesConveyor_, 
        interpolatedDepthImage_, 
        rgbImage_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);

#ifdef DEBUG_SHOW
    if (Parameters::Debug::show_valid_holes && validHolesMap.size() > 0)
    {
      // A vector containing images, one per valid hole found.
      std::vector<cv::Mat> holesImages;

      // A vector of validity probabilities per valid hole found.
      std::vector<std::string> msgs;

      // Iterate over the map of valid holes to their validity probabilities
      for (std::map<int, float>::iterator it = validHolesMap.begin();
          it != validHolesMap.end(); it++)
      {
        // A vector containing one entry: the validity probability of the
        // it->first-th hole
        std::vector<std::string> msg;

        msg.push_back(TOSTR(it->second));

        msgs.push_back(TOSTR(it->second));
        HolesConveyor temp;
        temp.rectangle.push_back(preValidatedHoles.rectangle[it -> first]);
        temp.keypoint.push_back(preValidatedHoles.keypoint[it -> first]);

        // Project this valid hole onto the rgb image
        holesImages.push_back(
            Visualization::showHoles("",
              rgbImage_,
              temp,
              -1,
              msg));
      }

      // Show all valid holes in one window
      Visualization::multipleShow("Valid Holes", holesImages, msgs, 1280, 1);
    }
#endif

    sensor_processor::ProcessorLogInfo resultMsg;
    resultMsg.success = validHolesMap.size() > 0;

    // If there are valid holes, publish them
    if (resultMsg.success)
    {
      publishValidHoles(preValidatedHoles, &validHolesMap);
    }
    processEndPublisher_.publish(resultMsg);

    // Publish the enhanced holes message
    // regardless of the amount of valid holes
    publishEnhancedHoles(preValidatedHoles, &validHolesMap);

    //#ifdef DEBUG_TIME
    //Timer::tick("processCandidateHoles");
    //Timer::printAllMeansTree();
    //#endif
  }



  //void HoleFusion::produceDataset(
  //  const HolesConveyor& conveyor,
  //  const std::vector<std::vector<float> >& probabilities)
  //{
  //  // Open the dataset
  //  std::ofstream dataset;
  //  dataset.open ("dataset_urgent.csv", std::ios_base::app | std::ios_base::out);

  //  for (int i = 0; i < conveyor.size(); i++)
  //  {
  //    // True holes
  //    if (conveyor.holes[i].keypoint.pt.x >  0
  //      && conveyor.holes[i].keypoint.pt.x < 1
  //      && conveyor.holes[i].keypoint.pt.y > 0
  //      && conveyor.holes[i].keypoint.pt.y < 1)
  //    {
  //      dataset << probabilities[0][i] << ", "
  //        << probabilities[1][i] << ", "
  //        << probabilities[2][i] << ", "
  //        << probabilities[3][i] << ", "
  //        //<< probabilities[4][i] << ", "
  //        //<< probabilities[5][i] << ", "
  //        //<< probabilities[6][i] << ", "
  //        //<< probabilities[7][i] << ", "
  //        << 1 << "\n";
  //    }
  //    else
  //    {
  //      dataset << probabilities[0][i] << ", "
  //        << probabilities[1][i] << ", "
  //        << probabilities[2][i] << ", "
  //        << probabilities[3][i] << ", "
  //        //<< probabilities[4][i] << ", "
  //        //<< probabilities[5][i] << ", "
  //        //<< probabilities[6][i] << ", "
  //        //<< probabilities[7][i] << ", "
  //        << 0 << "\n";
  //    }
  //  }
  //}


  /**
    @brief Publishes the holes' enhanced information.
    @param[in] conveyor [const HolesConveyor&]
    The overall valid holes found by the depth, thermal and RGB nodes.
    @param[in] validHolesMap [std::map<int, float>*]
    A map containing the indices of the valid holes inside the conveyor
    and their respective validity probabilities
    @return void
   **/
  void HoleFusion::publishEnhancedHoles (const HolesConveyor& conveyor,
      std::map<int, float>* validHolesMap)
  {
    // The overall message of enhanced holes that will be published
    pandora_vision_msgs::EnhancedImage enhancedHolesMsg;

    // Set the rgbImage in the enhancedHolesMsg message to the rgb image
    enhancedHolesMsg.rgbImage = MessageConversions::convertImageToMessage(
        rgbImage_,
        sensor_msgs::image_encodings::TYPE_8UC3,
        enhancedHolesMsg.rgbImage);

    // Set the depthImage in the enhancedHolesMsg message to the depth image
    enhancedHolesMsg.depthImage = MessageConversions::convertImageToMessage(
        Visualization::scaleImageForVisualization(interpolatedDepthImage_,
          Parameters::Image::scale_method),
        sensor_msgs::image_encodings::TYPE_8UC1,
        enhancedHolesMsg.depthImage);

    // Set the thermalImage in the enhancedHolesMsg message to the thermal image
    //enhancedHolesMsg.thermalImage = MessageConversions::convertImageToMessage(
    //    Visualization::scaleImageForVisualization(interpolatedThermalImage_,
    //      Parameters::Image::scale_method),
    //    sensor_msgs::image_encodings::TYPE_8UC1,
    //    enhancedHolesMsg.thermalImage);

    // Set whether depth analysis is applicable
    enhancedHolesMsg.isDepth = (filteringMode_ == RGBD_MODE);

    // Set the message's header
    enhancedHolesMsg.header.stamp = timestamp_;
    enhancedHolesMsg.header.frame_id = frame_id_;

    for (std::map<int, float>::iterator it = validHolesMap->begin();
        it != validHolesMap->end(); it++)
    {
      // The enhanced hole message. Used for one hole only
      pandora_vision_msgs::RegionOfInterest enhancedHoleMsg;

      // Set the hole's keypoint
      enhancedHoleMsg.center.x = conveyor.keypoint[it->first].x;
      enhancedHoleMsg.center.y = conveyor.keypoint[it->first].y;

      // Set the hole's bounding box width and height
      enhancedHoleMsg.width = conveyor.rectangle[it->first].width;
      enhancedHoleMsg.height = conveyor.rectangle[it->first].height;

      // Push back into the enhancedHolesMsg message
      enhancedHolesMsg.regionsOfInterest.push_back(enhancedHoleMsg);
    }

    // Publish the overall message
    enhancedHolesPublisher_.publish(enhancedHolesMsg);
  }

  /**
    @brief Publishes the valid holes' information.
    @param[in] conveyor [const HolesConveyor&] The overall valid and merged holes
    found by the depth and RGB nodes.
    @param[in] map [std::map<int, float>*] A map containing the indices
    of valid holes inside the conveyor and their respective
    probabilities of validity
    @return void
   **/
  void HoleFusion::publishValidHoles(const HolesConveyor& conveyor,
      std::map<int, float>* map)
  {
    // The depth sensor's horizontal and vertical field of view
    float hfov = Parameters::Image::horizontal_field_of_view;
    float vfov = Parameters::Image::vertical_field_of_view;

    // The frame's height and width
    int height = interpolatedDepthImage_.rows;
    int width = interpolatedDepthImage_.cols;

    // The overall valid holes found message
    pandora_vision_msgs::HoleDirectionAlertVector holesVectorMsg;

    // Counter for the holes' identifiers
    int holeId = 0;

    for (std::map<int, float>::iterator it = map->begin();
        it != map->end(); it++)
    {
      // A single hole's message
      pandora_vision_msgs::HoleDirectionAlert holeMsg;

      // The hole's keypoint coordinates relative to the center of the frame
      float x = conveyor.keypoint[it -> first].x
        - static_cast<float>(width) / 2;
      float y = static_cast<float>(height) / 2
        - conveyor.keypoint[it -> first].y;

      // The keypoint's yaw and pitch
      float yaw = atan(2 * x / width * tan(hfov / 2));
      float pitch = atan(2 * y / height * tan(vfov / 2));

      // Setup everything needed by the single hole's message
      holeMsg.info.yaw = yaw;
      holeMsg.info.pitch = pitch;
      holeMsg.info.probability = it->second;
      holeMsg.holeId = holeId;

      // Fill the overall holes found message with the current hole message
      holesVectorMsg.holesDirections.push_back(holeMsg);

      holeId++;
    }

    // Publish the message containing the information about all holes found
    holesVectorMsg.header.stamp = timestamp_;
    holesVectorMsg.header.frame_id = parent_frame_id_;

    validHolesPublisher_.publish(holesVectorMsg);

    // Publish an image with the valid holes found

    // The holes conveyor containing only the valid holes
    HolesConveyor validHolesConveyor;

    // Contains the validity probability for each hole considered valid
    std::vector<std::string> msgs;

    for (std::map<int, float>::iterator it = map->begin();
        it != map->end(); it++)
    {
      validHolesConveyor.rectangle.push_back(conveyor.rectangle[it -> first]);
      validHolesConveyor.keypoint.push_back(conveyor.keypoint[it -> first]);

      msgs.push_back(TOSTR(it->second));
    }

    // Valid holes on top of the RGB image
    cv::Mat rgbValidHolesImage; //=
    Visualization::showHoles("Valid Holes",
        rgbImage_,
        validHolesConveyor,
        -1,
        msgs);

    // Convert the image into a message
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->header = holesVectorMsg.header;
    msgPtr->encoding = sensor_msgs::image_encodings::BGR8;
    msgPtr->image = rgbValidHolesImage;

    // Publish the image message
    debugValidHolesPublisher_.publish(*msgPtr->toImageMsg());
  }


  /**
    @brief Applies a validation and merging operation of holes
    @details validate contours based on size and distance from image sensor, validate contours based on variance of depth, merge overlapping contours.
    @param[in,out] rgbHolesConveyor [HolesConveyor*] The rgb
    candidate holes conveyor
    @param[in,out] depthHolesConveyor [HolesConveyor*] The depth
    candidate holes conveyor
    @param[in,out] thermalHolesConveyor [HolesConveyor*] The thermal
    candidate holes conveyor
    @param[in] depthImage [const cv::Mat&] Depth Image used for validation purposes
    @param[in] rgbImage [const cv::Mat&] Rgb Image used for validation purposes
    @param[in] pointCloud [const PointCloudPtr] An interpolated point
    cloud used in the connection operation; it maybe will be used to obtain real world
    distances between holes
    @param[in,out] preValidatedHoles [HolesConveyor*] The valid and non double registered holes to publish
    @param[in,out] validHolesMap [std::map<int, float>*] Holes indexes with probabilities
    @return void
   **/
  void HoleFusion::mergeHoles(
      HolesConveyor* rgbHolesConveyor,
      HolesConveyor* depthHolesConveyor,
      HolesConveyor* thermalHolesConveyor,
      const cv::Mat& depthImage,
      const cv::Mat& rgbImage,
      const PointCloudPtr& pointCloud, 
      HolesConveyor* preValidatedHoles,
      std::map<int, float>* validHolesMap)
  {
#ifdef DEBUG_TIME
    Timer::start("applyMergeOperation", "mergeHoles");
#endif
    std::vector<bool> realRgbContours(rgbHolesConveyor -> rectangle.size(), true);
    std::vector<bool> realDepthContours(depthHolesConveyor -> rectangle.size(), true);
    std::vector<bool> realThermalContours(thermalHolesConveyor -> rectangle.size(), true);
    // Small contours at a small distance are not valid. Moreover, big contours at a big distance are not valid.
    validateDistance(depthImage, &(*rgbHolesConveyor), &realRgbContours, pointCloud);
    validateDistance(depthImage, &(*depthHolesConveyor), &realDepthContours, pointCloud);

    // eliminate RGB contours with small distance variance or very big distance variance (considering them as unstuffed)
    // Do not bother for small depth variance if the holes are too close to the sensor
    float sum = 0;
    for(int row = 0; row < depthImage.rows; row ++)
      for(int col = 0; col < depthImage.cols; col ++)
        sum += depthImage.at<float>(row, col);
    float avg = sum / (depthImage.rows * depthImage.cols);

    for(int i = 0; i < rgbHolesConveyor -> rectangle.size(); i++)
    {
      if(realRgbContours[i])
      {
        cv::Scalar mean;
        cv::Scalar stddev;
        cv::Mat ROI = depthImage(rgbHolesConveyor -> rectangle[i]);
        cv::meanStdDev (ROI, mean, stddev);
        float distanceVariance = static_cast<float>(stddev.val[0]); 
        if(avg > Depth::min_valid_depth)
        {
          if(distanceVariance < Parameters::HoleFusion::rgb_distance_variance_thresh 
              || (Parameters::HoleFusion::remove_unstuffed_holes 
                && distanceVariance > Parameters::HoleFusion::depth_difference_thresh))
            realRgbContours[i] = false;
        }
        else
          if(distanceVariance < Parameters::HoleFusion::rgb_small_distance_variance_thresh)
            realRgbContours[i] = false;

      }
    }


    // eliminate unstuffed rgb holes
    if(Parameters::HoleFusion::remove_unstuffed_holes)
    {
      if(Parameters::HoleFusion::unstuffed_removal_method == 0)
      {
        for(int i = 0; i < rgbHolesConveyor -> rectangle.size(); i++)
        {
          if(realRgbContours[i])
          {
            cv::Scalar mean;
            cv::Scalar stddev;
            cv::Mat ROI = depthImage(rgbHolesConveyor -> rectangle[i]);
            cv::meanStdDev (ROI, mean, stddev);
            float distanceVariance = static_cast<float>(stddev.val[0]); 
            if(distanceVariance > Parameters::HoleFusion::depth_difference_thresh)
              realRgbContours[i] = false;

          }
        }
      }
      else
      {
        // use a vertical and a horizontal line which pass from the middle of each box. 
        // Find an average of differences from initial pixel to the keypoint        
        for(int i = 0; i < rgbHolesConveyor -> rectangle.size(); i++)
        {
          if(realRgbContours[i])
          {
            float startY = rgbHolesConveyor -> rectangle[i].y;
            float startX = rgbHolesConveyor -> rectangle[i].x;
            float sumV = 0.0;
            float sumH = 0.0;
            int sumVPixels = 0;
            int sumHPixels = 0;
            for(int j = startY; j < rgbHolesConveyor -> keypoint[i].y; j ++) 
            {
              if(depthImage.at<float>(j, rgbHolesConveyor -> keypoint[i].x) != 0)
              {
                sumV += 
                  std::abs(depthImage.at<float>(startY, rgbHolesConveyor -> keypoint[i].x) 
                      - depthImage.at<float>(j, rgbHolesConveyor -> keypoint[i].x));
                sumVPixels++;
              }
            }
            for(int j = startX; j < rgbHolesConveyor -> keypoint[i].x; j ++) 
            {
              if(depthImage.at<float>(rgbHolesConveyor -> keypoint[i].y, j) != 0)
              {
                sumH += 
                  std::abs(depthImage.at<float>(rgbHolesConveyor -> keypoint[i].y, startX) 
                      - depthImage.at<float>(rgbHolesConveyor -> keypoint[i].y, j));
                sumHPixels++;
              }
            }
            float avg = (sumV / sumVPixels + sumH / sumHPixels) / 2;
            if(avg > Parameters::HoleFusion::difference_scanline_thresh)
              realRgbContours[i] = false;
          }
        }
      }
    }
    // keep only the valid contours till this point in a temporary conveyor
    HolesConveyor conveyorTemp;

    for(int i = 0; i < rgbHolesConveyor -> rectangle.size(); i++)
      if(realRgbContours[i])
      {
        conveyorTemp.keypoint.push_back((*rgbHolesConveyor).keypoint[i]);
        conveyorTemp.rectangle.push_back((*rgbHolesConveyor).rectangle[i]);
      }
    // Replace RGB conveyor with the valid holes
    (*rgbHolesConveyor).rectangle = conveyorTemp.rectangle;
    (*rgbHolesConveyor).keypoint = conveyorTemp.keypoint;
    conveyorTemp.rectangle.clear();
    conveyorTemp.keypoint.clear();
    realRgbContours.clear();
    for(int i = 0; i < rgbHolesConveyor -> rectangle.size(); i++)
      realRgbContours.push_back(true);

    // eliminate unstuffed depth holes
    if(Parameters::HoleFusion::remove_unstuffed_holes)
    {
      if(Parameters::HoleFusion::unstuffed_removal_method == 0)
      {
        for(int i = 0; i < depthHolesConveyor -> rectangle.size(); i++)
        {
          if(realDepthContours[i])
          {
            cv::Scalar mean;
            cv::Scalar stddev;
            cv::Mat ROI = depthImage(depthHolesConveyor -> rectangle[i]);
            cv::meanStdDev (ROI, mean, stddev);
            float distanceVariance = static_cast<float>(stddev.val[0]); 
            if(distanceVariance > Parameters::HoleFusion::depth_difference_thresh)
              realDepthContours[i] = false;

          }
        }
      }
      else
      {
        // use a vertical and a horizontal line which pass from the middle of each box. 
        // Find an average of differences from initial pixel to the keypoint        
        for(int i = 0; i < depthHolesConveyor -> rectangle.size(); i++)
        {
          if(realDepthContours[i])
          {
            float startY = depthHolesConveyor -> rectangle[i].y;
            float startX = depthHolesConveyor -> rectangle[i].x;
            float sumV = 0.0;
            float sumH = 0.0;
            int sumVPixels = 0;
            int sumHPixels = 0;
            for(int j = startY; j < depthHolesConveyor -> keypoint[i].y; j ++) 
            {
              if(depthImage.at<float>(j, depthHolesConveyor -> keypoint[i].x) != 0)
              {
                sumV += 
                  std::abs(depthImage.at<float>(startY, depthHolesConveyor -> keypoint[i].x) 
                      - depthImage.at<float>(j, depthHolesConveyor -> keypoint[i].x));
                sumVPixels++;
              }
            }
            for(int j = startX; j < depthHolesConveyor -> keypoint[i].x; j ++) 
            {
              if(depthImage.at<float>(depthHolesConveyor -> keypoint[i].y, j) != 0)
              {
                sumH += 
                  std::abs(depthImage.at<float>(depthHolesConveyor -> keypoint[i].y, startX) 
                      - depthImage.at<float>(depthHolesConveyor -> keypoint[i].y, j));
                sumHPixels++;
              }
            }
            float avg = (sumV / sumVPixels + sumH / sumHPixels) / 2;
            if(avg > Parameters::HoleFusion::difference_scanline_thresh)
              realDepthContours[i] = false;
          }
        }
      }
    }


    for(int i = 0; i < depthHolesConveyor -> rectangle.size(); i++)
      if(realDepthContours[i])
      {
        conveyorTemp.keypoint.push_back((*depthHolesConveyor).keypoint[i]);
        conveyorTemp.rectangle.push_back((*depthHolesConveyor).rectangle[i]);
      }
    // Replace Depth conveyor with the valid holes
    (*depthHolesConveyor).rectangle = conveyorTemp.rectangle;
    (*depthHolesConveyor).keypoint = conveyorTemp.keypoint;
    conveyorTemp.rectangle.clear();
    conveyorTemp.keypoint.clear();
    realDepthContours.clear();
    for(int i = 0; i < depthHolesConveyor -> rectangle.size(); i++)
      realDepthContours.push_back(true);


    // all thermal contours are valid
    for(int i = 0; i < thermalHolesConveyor -> rectangle.size(); i++)
      realThermalContours.push_back(true);
    // If there are no candidate holes from both RGB and Depth,
    // there is no meaning to this operation
    if (rgbHolesConveyor -> rectangle.size() == 0 || depthHolesConveyor -> rectangle.size() == 0)
    {
    }

    // merge contours of the same kind
    mergeSameNeighbors(rgbImage, &realRgbContours, &(*rgbHolesConveyor));
    mergeSameNeighbors(rgbImage, &realDepthContours, &(*depthHolesConveyor));
    mergeSameNeighbors(rgbImage, &realThermalContours, &(*thermalHolesConveyor));

    // counter for the number of valid holes
    int vi = 0;
    // merge overlapping contours. 
    // merge rgb contours with all others
    mergeDifferentNeighbors(
        rgbImage,
        &realRgbContours, 
        &realDepthContours, 
        &realThermalContours, 
        (*rgbHolesConveyor),
        (*depthHolesConveyor),
        (*thermalHolesConveyor),
        &(*preValidatedHoles),
        &(*validHolesMap),
        &vi);

    // merge depth contours with all others. This happens only if these contours
    // did not get merged at the previous step 
    mergeDifferentNeighbors(
        rgbImage,
        &realDepthContours, 
        &realRgbContours, 
        &realThermalContours, 
        (*depthHolesConveyor),
        (*rgbHolesConveyor),
        (*thermalHolesConveyor),
        &(*preValidatedHoles),
        &(*validHolesMap),
        &vi);

    // merge thermal contours with all others. This happens only if these contours
    // did not get merged at the previous step 
    mergeDifferentNeighbors(
        rgbImage,
        &realThermalContours, 
        &realDepthContours, 
        &realRgbContours, 
        (*thermalHolesConveyor),
        (*depthHolesConveyor),
        (*rgbHolesConveyor),
        &(*preValidatedHoles),
        &(*validHolesMap),
        &vi);

#ifdef DEBUG_TIME
    Timer::tick("applyMergeOperation");
#endif
  }

  /**
    @brief Applies a validation based on size and distance 
    @details Small contours in small distance or big contours at big distance are not valid
    @param[in] image [const cv::Mat&] Depth Image used for validation purposes
    @param[in,out] holesConveyor [HolesConveyor*] The rgb or depth conveyor 
    @param[in,out] realContours [std::vector<bool>*] A vector which show for each hole if it is valid or not
    @param[in] pointCloud [const PointCloudPtr] An interpolated point
    cloud 
    @return void
   **/
  void HoleFusion::validateDistance(
      const cv::Mat& image,
      HolesConveyor* holesConveyor,
      std::vector<bool>* realContours,
      const PointCloudPtr& pointCloud)
  {
    for(int i = 0; i < holesConveyor -> rectangle.size(); i ++)
    {
      float sumDepths = 0.0;
      int upperX = static_cast<int>(holesConveyor -> keypoint[i].x - holesConveyor -> rectangle[i].width / 2);
      int upperY = static_cast<int>(holesConveyor -> keypoint[i].y - holesConveyor -> rectangle[i].height / 2);
      int lowerX = static_cast<int>(holesConveyor -> keypoint[i].x + holesConveyor -> rectangle[i].width / 2);
      int lowerY = static_cast<int>(holesConveyor -> keypoint[i].y + holesConveyor -> rectangle[i].height / 2);
      if(upperX < 0)
        upperX = 0;
      if(lowerX > image.cols)
        lowerX = image.cols;
      if(upperY < 0)
        upperY = 0;
      if(lowerY > image.rows)
        lowerY = image.rows;
      for(int col = upperX; col < lowerX; col ++)
        for(int row = upperY; row < lowerY; row ++)
        {
          sumDepths += image.at<float>(row, col);
        }
      float avgDepth = sumDepths / ((lowerX - upperX) * (lowerY - upperY));
      if(holesConveyor -> rectangle[i].width 
          * holesConveyor -> rectangle[i].height <  Parameters::HoleFusion::small_rect_thresh  
          && avgDepth < Parameters::HoleFusion::max_depth_to_test_small_thresh)
      {
        (*realContours)[i] = false;
      }
      else if(holesConveyor -> rectangle[i].width 
          * holesConveyor -> rectangle[i].height >  Parameters::HoleFusion::big_rect_thresh 
          && avgDepth > Parameters::HoleFusion::min_depth_to_test_big_thresh)
      {
        (*realContours)[i] = false;
      }
    }
  }

  /**
    @brief Applies merging of contours of same type, e.g. RGB - RGB 
    @details We will consider distance and homogenity features for the merging
    @param[in] image [const cv::Mat&] RGB Image used for homogenity extraction
    @param[in,out] realContours [std::vector<bool>*] A vector which shows for 
    each hole if it is valid or not
    @param[in,out] holesConveyor [HolesConveyor*] The rgb, thermal or depth 
    conveyor 
    cloud 
    @return void
   **/

  void HoleFusion::mergeSameNeighbors(
      const cv::Mat& image,
      std::vector<bool>* realContours,
      HolesConveyor* holesConveyor)
  {
    HolesConveyor conveyorTemp;
    std::vector<std::vector<float> > 
      edMatrix(holesConveyor -> rectangle.size(), std::vector<float>(holesConveyor -> rectangle.size()));
    // first step of contour merging; merge contours of the same kind
    for(int contouri = 0; contouri < holesConveyor -> rectangle.size(); contouri ++)
      for(int contourj = 0; contourj < holesConveyor -> rectangle.size(); contourj ++)
      {
        float keypointDistance = 
          std::sqrt(
              std::pow(holesConveyor -> keypoint[contouri].x - holesConveyor -> keypoint[contourj].x, 2 )
              + std::pow(holesConveyor -> keypoint[contouri].y - holesConveyor -> keypoint[contourj].y, 2 ));
        float minBorderDistanceX = 
          std::min(std::abs(holesConveyor -> rectangle[contouri].x - holesConveyor -> rectangle[contourj].x 
                - holesConveyor -> rectangle[contourj].width),
              std::abs(holesConveyor -> rectangle[contouri].x + holesConveyor -> rectangle[contouri].width 
                - holesConveyor -> rectangle[contourj].x));
        float minBorderDistanceY = 
          std::min(std::abs(holesConveyor -> rectangle[contouri].y - holesConveyor -> rectangle[contourj].y 
                - holesConveyor -> rectangle[contourj].height),
              std::abs(holesConveyor -> rectangle[contouri].y + holesConveyor -> rectangle[contouri].height 
                - holesConveyor -> rectangle[contourj].y));
        float bordersDistance = 
          std::sqrt(std::pow(minBorderDistanceX, 2) + std::pow(minBorderDistanceY, 2));
        edMatrix[contouri][contourj] = std::min(keypointDistance, bordersDistance);
      }
    // Merge same type step

    for(int contouri = 0; contouri < holesConveyor -> rectangle.size(); contouri ++)
    {
      if((*realContours)[contouri])
      {
        float sumKX = holesConveyor -> keypoint[contouri].x;
        float sumKY = holesConveyor -> keypoint[contouri].y;
        int overlapsSum = 1;
        int upperX = holesConveyor -> rectangle[contouri].x;
        int upperY = holesConveyor -> rectangle[contouri].y;
        int lowerX = holesConveyor -> rectangle[contouri].x + holesConveyor -> rectangle[contouri].width;
        int lowerY = holesConveyor -> rectangle[contouri].y + holesConveyor -> rectangle[contouri].height;

        for(int contourj = contouri + 1; contourj < holesConveyor -> rectangle.size(); contourj ++)
        {
          if(edMatrix[contouri][contourj] < Parameters::HoleFusion::merging_distance_thresh)
          {
            overlapsSum++;
            sumKX += holesConveyor -> keypoint[contourj].x;
            sumKY += holesConveyor -> keypoint[contourj].y;
            upperX = std::min(upperX, holesConveyor -> rectangle[contourj].x);
            upperY = std::min(upperY, holesConveyor -> rectangle[contourj].y);
            lowerX = 
              std::max(lowerX, holesConveyor -> rectangle[contourj].x 
                  + holesConveyor -> rectangle[contourj].width);
            lowerY = 
              std::max(lowerY, holesConveyor -> rectangle[contourj].y 
                  + holesConveyor -> rectangle[contourj].height);
            (*realContours)[contourj] = false;
          }
        }
        cv::Point2f mergedKeypoint(sumKX / overlapsSum, sumKY / overlapsSum);
        conveyorTemp.keypoint.push_back(mergedKeypoint);
        cv::Rect mergedRect(upperX, upperY, lowerX - upperX, lowerY - upperY);
        conveyorTemp.rectangle.push_back(mergedRect);
        
      }
    }
    // Replace conveyor with the merged holes
    (*holesConveyor).rectangle = conveyorTemp.rectangle;
    (*holesConveyor).keypoint = conveyorTemp.keypoint;
    conveyorTemp.rectangle.clear();
    conveyorTemp.keypoint.clear();

    (*realContours).clear();
    for(int i = 0; i < (*holesConveyor).rectangle.size(); i ++)
      (*realContours).push_back(true);

  }


  /**
    @brief Applies merging of contours of different types, 
    e.g. RGB-Depth-Thermal  
    @details We will consider distance and homogenity features for the merging.
    Firstly, store in 2D vectors the euclidean distance of all contours of one 
    type, against the contours of all other types. Finally, find the contours 
    with small distance and merge them.
    @param[in] image [const cv::Mat&] RGB Image used for homogenity extraction
    @param[in,out] realContours1 [std::vector<bool>*] A vector which shows for 
    each hole of first type if it is valid or not
    @param[in,out] realContours2 [std::vector<bool>*] A vector which shows for 
    each hole of second type if it is valid or not
    @param[in,out] realContours3 [std::vector<bool>*] A vector which shows for 
    each hole of third type if it is valid or not
    @param[in,out] holesConveyor1 [const HolesConveyor&] The rgb, thermal or depth 
    conveyor different from the other types
    @param[in,out] holesConveyor2 [const HolesConveyor&] The rgb, thermal or depth 
    conveyor different from the other types
    @param[in,out] holesConveyor3 [const HolesConveyor&] The rgb, thermal or depth 
    conveyor different from the other types
    @param[in,out] preValidatedHoles [HolesConveyor*] The valid and non double registered holes to publish
    @param[in,out] validHolesMap [std::map<int, float>*] Holes indexes with probabilities
    @param[in,out] vi [int*] Current valid holes index. The valid holes are stored in one vector
    @return void
   **/
  void HoleFusion::mergeDifferentNeighbors(
      const cv::Mat& image,
      std::vector<bool>* realContours1,
      std::vector<bool>* realContours2,
      std::vector<bool>* realContours3,
      const HolesConveyor& holesConveyor1,
      const HolesConveyor& holesConveyor2,
      const HolesConveyor& holesConveyor3,
      HolesConveyor* preValidatedHoles,
      std::map<int, float>* validHolesMap,
      int* vi)
  {
    std::vector<std::vector<float> > 
      edMatrix12(holesConveyor1.rectangle.size(), std::vector<float>(holesConveyor2.rectangle.size()));
    std::vector<std::vector<float> > 
      edMatrix13(holesConveyor1.rectangle.size(), std::vector<float>(holesConveyor3.rectangle.size()));
    std::vector<std::vector<float> > 
      edMatrix23(holesConveyor2.rectangle.size(), std::vector<float>(holesConveyor3.rectangle.size()));
    // Calculate distances for all combinations between the contours sources 
    findContoursDistance(holesConveyor1, holesConveyor2, &edMatrix12);
    findContoursDistance(holesConveyor1, holesConveyor3, &edMatrix13);
    findContoursDistance(holesConveyor2, holesConveyor3, &edMatrix23);
    // Merge step

    for(int contouri = 0; contouri < holesConveyor1.rectangle.size(); contouri ++)
    {
      if((*realContours1)[contouri])
      {
        int merges = 0;
        bool foundOne1 = false;
        bool foundOne2 = false;
        float sumKX = holesConveyor1.keypoint[contouri].x;
        float sumKY = holesConveyor1.keypoint[contouri].y;
        int overlapsSum = 1;
        int upperX = holesConveyor1.rectangle[contouri].x;
        int upperY = holesConveyor1.rectangle[contouri].y;
        int lowerX = holesConveyor1.rectangle[contouri].x + holesConveyor1.rectangle[contouri].width;
        int lowerY = holesConveyor1.rectangle[contouri].y + holesConveyor1.rectangle[contouri].height;

        for(int contourj = 0; contourj < holesConveyor2.rectangle.size(); contourj ++)
        {
          bool mergable = false;
          calculateMergingProbability(
              image, 
              holesConveyor1.rectangle[contouri],
              holesConveyor2.rectangle[contourj],
              &mergable);
          if(edMatrix12[contouri][contourj] < Parameters::HoleFusion::merging_distance_thresh
              && (*realContours2)[contourj] && mergable)
          {
            mergable = false;
            (*realContours2)[contourj] = false;
            if(!foundOne1)
              merges++;
            foundOne1 = true;
            overlapsSum++;
            sumKX += holesConveyor2.keypoint[contourj].x;
            sumKY += holesConveyor2.keypoint[contourj].y;
            upperX = std::min(upperX, holesConveyor2.rectangle[contourj].x);
            upperY = std::min(upperY, holesConveyor2.rectangle[contourj].y);
            lowerX = std::max(lowerX, holesConveyor2.rectangle[contourj].x
                + holesConveyor2.rectangle[contourj].width);
            lowerY = std::max(lowerY, holesConveyor2.rectangle[contourj].y
                + holesConveyor2.rectangle[contourj].height);
            for(int contourk = 0; contourk < holesConveyor3.rectangle.size(); contourk ++)
            {
              bool mergable1 = false;
              bool mergable2 = false;
              calculateMergingProbability(
                  image, 
                  holesConveyor1.rectangle[contouri],
                  holesConveyor3.rectangle[contourk],
                  &mergable1);
              calculateMergingProbability(
                  image, 
                  holesConveyor2.rectangle[contourj],
                  holesConveyor3.rectangle[contourk],
                  &mergable2);
              if(((edMatrix13[contouri][contourk] < Parameters::HoleFusion::merging_distance_thresh
                      && mergable1) 
                    || (edMatrix23[contourj][contourk] < Parameters::HoleFusion::merging_distance_thresh 
                      && mergable2))
                  && (*realContours3)[contourk])
              {
                (*realContours3)[contourk] = false;
                mergable1 = false;
                mergable2 = false;
                overlapsSum++;
                sumKX += holesConveyor3.keypoint[contourk].x;
                sumKY += holesConveyor3.keypoint[contourk].y;
                if(!foundOne2)
                  merges++;
                foundOne2 = true;
                upperX = std::min(upperX, holesConveyor3.rectangle[contourk].x);
                upperY = std::min(upperY, holesConveyor3.rectangle[contourk].y);
                lowerX = std::max(lowerX, holesConveyor3.rectangle[contourk].x 
                    + holesConveyor3.rectangle[contourk].width);
                lowerY = 
                  std::max(lowerY, holesConveyor3.rectangle[contourk].y 
                      + holesConveyor3.rectangle[contourk].height);
              }
            }
          }
        }

        if(merges == 0)
          (*validHolesMap)[(*vi)] = Parameters::HoleFusion::valid_light_probability;
        else if(merges == 1)
          (*validHolesMap)[(*vi)] = Parameters::HoleFusion::valid_medium_probability;
        else
          (*validHolesMap)[(*vi)] = Parameters::HoleFusion::valid_strong_probability;
        (*realContours1)[contouri] = false;
        cv::Point2f mergedKeypoint(sumKX / overlapsSum, sumKY / overlapsSum);
        (*preValidatedHoles).keypoint.push_back(mergedKeypoint);
        cv::Rect mergedRect(upperX, upperY, lowerX - upperX, lowerY - upperY);
        (*preValidatedHoles).rectangle.push_back(mergedRect);
        (*vi)++;
      }
    }
  }


  /**
    @brief Finds distance between contours of two sources 
    @param[in,out] holesConveyor1 [HolesConveyor&] The rgb, thermal or depth
    conveyor 
    @param[in,out] holesConveyor2 [HolesConveyor&] The rgb, thermal or depth 
    conveyor
    @param[in,out] edMatrix [std::vector<std::vector<float> >*] The matrix
    where distances are stored, first source against second source
    @return void
   **/
  void HoleFusion::findContoursDistance(
      const HolesConveyor& holesConveyor1,
      const HolesConveyor& holesConveyor2,
      std::vector<std::vector<float> >* edMatrix)
  {
    for(int contouri = 0; contouri < holesConveyor1.rectangle.size(); contouri ++)
    {
      for(int contourj = 0; contourj < holesConveyor2.rectangle.size(); contourj ++)
      {
        cv::Rect overlap = holesConveyor1.rectangle[contouri] & holesConveyor2.rectangle[contourj];
        // check if two rectangles intersect, if yes set a very small distance, otherwise set the smallest distance 
        // between the two rectangles 
        if( cv::Rect(holesConveyor1.rectangle[contouri]) == cv::Rect() )
        {
          float keypointDistance = 
            std::sqrt(
                std::pow(holesConveyor1.keypoint[contouri].x - holesConveyor2.keypoint[contourj].x, 2 )
                + std::pow(holesConveyor1.keypoint[contouri].y - holesConveyor2.keypoint[contourj].y, 2 ));
          float minBorderDistanceX = 
            std::min(std::abs(holesConveyor1.rectangle[contouri].x - holesConveyor2.rectangle[contourj].x 
                  - holesConveyor2.rectangle[contourj].width),
                std::abs(holesConveyor1.rectangle[contouri].x + holesConveyor1.rectangle[contouri].width 
                  - holesConveyor2.rectangle[contourj].x));
          float minBorderDistanceY = 
            std::min(std::abs(holesConveyor1.rectangle[contouri].y - holesConveyor2.rectangle[contourj].y 
                  - holesConveyor2.rectangle[contourj].height),
                std::abs(holesConveyor1.rectangle[contouri].y + holesConveyor1.rectangle[contouri].height 
                  - holesConveyor2.rectangle[contourj].y));
          float bordersDistance = 
            std::sqrt(std::pow(minBorderDistanceX, 2) + std::pow(minBorderDistanceY, 2));
          (*edMatrix)[contouri][contourj] = std::min(keypointDistance, bordersDistance);
        }
        else
        {
          // If there is an intersection then they surely need to get merged
          (*edMatrix)[contouri][contourj] = 1;
        }
      }
    }
  }

  /**
    @brief Sets the depth values of a point cloud according to the
    values of a depth image.

    Needed by the depth-based filters that employ a point cloud analysis.
    The values of the point cloud published by the synchronizer node to
    the hole fusion node contain the unadulterated values of the point
    cloud directly obtained by the depth sensor,
    hence they contain NaNs and zero-value pixels that
    would otherwise, if not for this method, obstruct the function of the
    above filters.
    @param[in] inImage [const cv::Mat&] The depth image in CV_32FC1 format
    @param[out] pointCloudPtr [PointCloudPtr*] The point cloud
    @return void
   **/
  void HoleFusion::setDepthValuesInPointCloud(const cv::Mat& inImage,
      PointCloudPtr* pointCloudPtr)
  {
#ifdef DEBUG_TIME
    Timer::start("setDepthValuesInPointCloud", "pointCloudCallback");
#endif

    // If the inImage is not of type CV_32FC1, return
    if(inImage.type() != CV_32FC1)
    {
      return;
    }

    for (unsigned int row = 0; row < (*pointCloudPtr)->height; ++row)
    {
      for (unsigned int col = 0; col < (*pointCloudPtr)->width; ++col)
      {
        (*pointCloudPtr)->points[col + (*pointCloudPtr)->width * row].z =
          inImage.at<float>(row, col);
      }
    }

#ifdef DEBUG_TIME
    Timer::tick("setDepthValuesInPointCloud");
#endif
  }

  /**
    @brief Calculates homogenity between two rectangles and returns an 
    indication if the holes inside these rectangles are mergable or not.

    @param[in] image [const cv::Mat&] The image where homogenity will be
    calculated
    @param[in] rectangle1 [cv::Rect&] First rectangle
    @param[in] rectangle2 [cv::Rect&] Second rectangle
    @param[out] mergable [bool*] The indication, true means that the holes
    are mergable
    @return void
   **/
  void HoleFusion::calculateMergingProbability(
      const cv::Mat& image,
      const cv::Rect& rectangle1,
      const cv::Rect& rectangle2,
      bool* mergable)
  {
    float upperX = std::min(rectangle1.x, rectangle2.x);
    float upperY = std::min(rectangle1.y, rectangle2.y);
    float lowerX = 
      std::max(rectangle1.x + rectangle1.width,
          rectangle2.x + rectangle2.width);
    float lowerY = 
      std::max(rectangle1.y + rectangle1.height,
          rectangle2.y + rectangle2.height);
    float homogRectWidth = std::abs(lowerX - upperX);
    float homogRectHeight = std::abs(lowerY - upperY);
    if(homogRectWidth < Rgb::homog_rect_dims_thresh)
      homogRectWidth = std::max(rectangle1.width, rectangle2.width);
    if(homogRectHeight < Rgb::homog_rect_dims_thresh)
      homogRectHeight = std::max(rectangle1.height, rectangle2.height);
    if(homogRectWidth == 0 || homogRectHeight == 0)
      return;
    //if(upperX < 0)
    //  upperX = 0;
    //if(upperY < 0)
    //  upperY = 0;
    //if(lowerX > image.cols)
    //  lowerX = image.cols;
    //if(lowerY > image.rows)
    //  lowerY = image.rows;
    cv::Mat ROI = image(cv::Rect(upperX, upperY, homogRectWidth, homogRectHeight));
    HaralickFeaturesExtractor haralickFeaturesDetector_;
    haralickFeaturesDetector_.findHaralickFeatures(ROI);
    std::vector<double> haralickFeatures = haralickFeaturesDetector_.getFeatures();
    if(haralickFeatures[0] > Rgb::homogenity_thresh)
    {
      (*mergable) = true;
    }


  }

  /**
    @brief The node's state manager.

    If the current state is set to off, the synchronizer node is not
    subscribed to the input point cloud. Otherwise, it is. In the event
    of an "off" state instruction while "on", this method instructs the
    synchronizer to leave its subscription from the input point cloud.
    In the event of an "on" state instruction while "off", the this method
    instructs the synchronizer node to subscribe to the input point cloud
    and, if no callback of the three is called on the hole fusion node,
    unlocks the synchronizer, so that normal processing can start/continue.
    @param[in] newState [const int&] The robot's new state
    @return void
   **/
  void HoleFusion::startTransition(int newState)
  {
    // The new on/off state of the Hole Detector package
    bool toBeOn = isHoleDetectorOn(newState);

    // off -> on
    if (!isOn_ && toBeOn)
    {
      // The on/off state of the Hole Detector Package is off, so the
      // synchronizer is not subscribed to the input point cloud.
      // Make him subscribe to it now
      std_msgs::Empty msg;
      synchronizerSubscribeToInputPointCloudPublisher_.publish(msg);

      // Set the Hole Detector's on/off state to the new one.
      // In this case, it has to be before the call to unlockSynchronizer
      isOn_ = toBeOn;

      // If all three callbacks have finished execution and they are waiting
      // a new input while the state changed, the synchronizer needs to be
      // unclocked manually here.
      // By contrast, if the number of nodes ready is non-zero,
      // while maybe impossible due to the halted state of the Hole Detector,
      // the last callback that finishes execution will attempt to unlock the
      // synchonizer, thus a manual unlock is not needed.
      if (numNodesReady_ == 0)
      {
        unlockSynchronizer();
      }
    }
    // on -> off
    else if (isOn_ && !toBeOn)
    {
      isOn_ = toBeOn;

      // The on/off state of the Hole Detector package is on and is to be off.
      // The synchronizer node is subscribed to the input point cloud and now
      // it should leave its subscription to it so that the processing
      // resources of the robot's computer pertaining to the Hole Detector
      // package are minimized
      std_msgs::Empty msg;
      synchronizerLeaveSubscriptionToInputPointCloudPublisher_.publish(msg);
    }
    else
    {
      isOn_ = toBeOn;
    }

    transitionComplete(newState);
  }



  /**
    @brief Requests from the synchronizer to process a new point cloud
    @return void
   **/
  void HoleFusion::unlockSynchronizer()
  {
    // The Hole Fusion node can request from the synchronizer node to process
    // a new point cloud only if the on/off state of the Hole Detector package
    // is set to on
    if (isOn_)
    {
      ROS_INFO_NAMED(PKG_NAME, "Sending unlock message");

      std_msgs::Empty unlockMsg;
      unlockPublisher_.publish(unlockMsg);
    }
  }

} // namespace pandora_vision
