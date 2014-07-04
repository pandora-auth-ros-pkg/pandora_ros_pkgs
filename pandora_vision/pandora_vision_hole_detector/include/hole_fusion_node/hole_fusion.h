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

#ifndef HOLE_FUSION_NODE_HOLE_FUSION_H
#define HOLE_FUSION_NODE_HOLE_FUSION_H

#include <dirent.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <urdf_parser/urdf_parser.h>
#include <image_transport/image_transport.h>
#include "state_manager/state_client.h"
#include "vision_communications/CandidateHolesVectorMsg.h"
#include "vision_communications/CandidateHoleMsg.h"
#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/HoleDirectionMsg.h"
#include "vision_communications/EnhancedHolesVectorMsg.h"
#include "vision_communications/EnhancedHoleMsg.h"
#include "utils/defines.h"
#include "utils/histogram.h"
#include "utils/message_conversions.h"
#include "utils/noise_elimination.h"
#include "utils/parameters.h"
#include "utils/visualization.h"
#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/filters.h"
#include "hole_fusion_node/filters_resources.h"
#include "hole_fusion_node/rgb_filters.h"
#include "hole_fusion_node/hole_merger.h"
#include "hole_fusion_node/hole_validation.h"


/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleFusion
    @brief Provides functionalities and methods for fusing holes obtained
    through Depth and RGB analysis
   **/
  class HoleFusion : public StateClient
  {
    private:

      // The main ROS nodehandle
      ros::NodeHandle nodeHandle_;

      // The ROS nodehandle needed by the general_cfg
      ros::NodeHandle generalNodeHandle_;

      // The ROS nodehandle needed by the debug_cfg
      ros::NodeHandle debugNodeHandle_;

      // The ROS nodehandle needed by the filters_order_cfg
      ros::NodeHandle filtersOrderNodeHandle_;

      // The ROS nodehandle needed by the filters_thresholds_cfg
      ros::NodeHandle filtersThresholdsNodeHandle_;

      // The ROS nodehandle needed by the validity_cfg
      ros::NodeHandle validityNodeHandle_;

      // The image_transport nodehandle
      image_transport::ImageTransport imageTransport_;

      // The ROS publisher that will be used for unlocking the
      // synchronizer_node
      ros::Publisher unlockPublisher_;

      // The name of the topic where the Hole Fusion node will publish messages
      // in order to unlock the Synchronizer node
      std::string unlockTopic_;

      // The ROS publisher that will be used to publish the final valid holes
      // produced by this package
      ros::Publisher validHolesPublisher_;

      // The name of the topic where the Hole Fusion node will publish
      // information pertaining to the valid holes found by the Hole Detector
      // package
      std::string validHolesTopic_;

      // The ROS publisher that will be used to publish the enhanced valid holes
      // produced by this package
      ros::Publisher enhancedHolesPublisher_;

      // The name of the topic where the Hole Fusion node will publish
      // additional information, in respect to the valid holes topic,
      // pertaining to the valid holes found by the Hole Detector package
      std::string enhancedHolesTopic_;

      // The ROS publisher that will be used to publish an image depicting
      // the keypoint, outline points and bounding rectangle of holes found
      // by the Depth and RGB nodes
      image_transport::Publisher debugRespectiveHolesPublisher_;

      // The name of the topic where the Hole Fusion node puplishes
      // an image depicting the keypoint, outline points and
      // bounding rectangle of holes found by the Depth and RGB nodes
      std::string debugRespectiveHolesTopic_;

      // The ROS publisher that will be used to publish an image depicting
      // the keypoint, outline points and bounding rectangle for all
      // valid holes found
      image_transport::Publisher debugValidHolesPublisher_;

      // The name of the topic where the Hole Fusion node puplishes
      // an image depicting the keypoint, outline points and
      // bounding rectangle for all valid holes found
      std::string debugValidHolesTopic_;

      // The publisher that the Hole Fusion node uses to request from the
      // synchronizer node to subscribe to the input point cloud
      ros::Publisher synchronizerSubscribeToInputPointCloudPublisher_;

      // The name of the topic that the Hole Fusion node uses to request from
      // the synchronizer node to subscribe to the input point cloud
      std::string synchronizerSubscribeToInputPointCloudTopic_;

      // The publisher that the Hole Fusion node uses to request from the
      // synchronizer node to leave its subscription to the input point cloud
      ros::Publisher synchronizerLeaveSubscriptionToInputPointCloudPublisher_;

      // The name of the topic that the Hole Fusion node uses to request from
      // the synchronizer node to leave its subscription to the
      // input point cloud
      std::string synchronizerLeaveSubscriptionToInputPointCloudTopic_;

      // The ROS subscriber for acquisition of candidate holes originated
      // from the depth node
      ros::Subscriber depthCandidateHolesSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // for acquiring information about candidate holes found by the Depth
      // node
      std::string depthCandidateHolesTopic_;

      // The ROS subscriber for acquisition of candidate holes originated
      // from the rgb node
      ros::Subscriber rgbCandidateHolesSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // for acquiring information about candidate holes found by the Rgb
      // node
      std::string rgbCandidateHolesTopic_;

      // The ROS subscriber for acquisition of the point cloud originated
      // from the synchronizer node
      ros::Subscriber pointCloudSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // where it acquires the point cloud from the Synchronizer node
      std::string pointCloudTopic_;

      // The timestamp of the point cloud
      ros::Time timestamp_;

      // The frame_id of the point cloud
      std::string frame_id_;

      // The parent frame_id of the point cloud
      std::string parent_frame_id_;

      // Indicates how many of the depth_node and rgb_node nodes have
      // received hole candidates and are ready to send them for processing
      int numNodesReady_;

      // The rgb received by the RGB node
      cv::Mat rgbImage_;

      // The point cloud received by the synchronizer node
      PointCloudPtr pointCloud_;

      // The interpolated depth image received by the depth node
      cv::Mat interpolatedDepthImage_;

      // The conveyor of hole candidates received by the depth node
      HolesConveyor depthHolesConveyor_;

      // The conveyor of hole candidates received by the rgb node
      HolesConveyor rgbHolesConveyor_;

      // Indicates whether RGB-based only
      // or both the RGB and Depth based filtering is applicable
      int filteringMode_;

      // A histogramm for the texture of walls
      cv::MatND wallsHistogram_;

      // The on/off state of the Hole Detector package
      bool isOn_;


      // The dynamic reconfigure server for debugging parameters
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        debug_cfgConfig> serverDebug;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        debug_cfgConfig>::CallbackType f_debug;


      // The dynamic reconfigure server for parameters pertaining to the order
      // of filters
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        filters_order_cfgConfig> serverFiltersOrder;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        filters_order_cfgConfig>::CallbackType f_filters_order;


      // The dynamic reconfigure server for parameters pertaining to
      // thresholds of filters
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        filters_thresholds_cfgConfig> serverFiltersThresholds;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        filters_thresholds_cfgConfig>::CallbackType f_filters_thresholds;


      // The dynamic reconfigure server for general parameters
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        general_cfgConfig> serverGeneral;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        general_cfgConfig>::CallbackType f_general;


      // The dynamic reconfigure server for parameters pertaining to
      // the validity of holes
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        validity_cfgConfig> serverValidity;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::
        validity_cfgConfig>::CallbackType f_validity;


      /**
        @brief Callback for the candidate holes via the depth node.

        This method sets the interpolated depth image and the
        candidate holes acquired from the depth node.
        If the rgb and point cloud callback counterparts have done
        what must be, it resets the number of ready nodes, unlocks
        the synchronizer and calls for processing of the candidate
        holes.
        @param[in] depthCandidateHolesVector
        [const vision_communications::CandidateHolesVectorMsg&]
        The message containing the necessary information acquired through
        the depth node
        @return void
       **/
      void depthCandidateHolesCallback(
        const vision_communications::CandidateHolesVectorMsg&
        depthCandidateHolesVector);

      /**
        @brief Runs candidate holes through selected filters.
        Probabilities for each candidate hole and filter
        are printed in the console, with an order specified by the
        hole_fusion_cfg of the dynamic reconfigure utility
        @param[in] conveyor [const HolesConveyor&] The conveyor
        containing candidate holes that are to be checked against selected
        filters
        @return [std::vector<std::vector<float> >]
        A two dimensional vector containing the probabilities of
        validity of each candidate hole. Each row of it pertains to a specific
        filter applied, each column to a particular hole
       **/
      std::vector<std::vector<float> > filterHoles(
        const HolesConveyor& conveyor);

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
      void fromCandidateHoleMsgToConveyor(
        const std::vector<vision_communications::CandidateHoleMsg>&
        candidateHolesVector,
        HolesConveyor* conveyor,
        const cv::Mat& inImage);

      /**
        @brief Retrieves the parent to the frame_id of the input point cloud,
        so as to set the frame_id of the output messages of valid holes.
        @param void
        @return void
       **/
      void getParentFrameId();

      /**
        @brief Acquires topics' names needed to be subscribed to and advertise
        to by the Hole Fusion node
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief Computes the on/off state of the Hole Detector package,
        given a state
        @param[in] state [const int&] The robot's state
        @return [bool] True if the hole fusion node's state is set to "on"
       **/
      bool isHoleDetectorOn(const int& state);

      /**
        @brief Takes as input a container of holes and a map of indices
        of holes referring to the container to validity probabilities.
        It outputs the most probable unique valid holes and a map
        adjusted to fit the altered container of holes.

        Inside the allHoles container, reside holes that have originated
        from the Depth and RGB nodes, plus any merges between them. Having
        acquired the validity probability of each one of them, this method
        locates valid holes that refer to the same physical hole in space
        inside the allHoles container and picks the one with the largest
        validity probability.
        @param[in,out] allHoles [HolesConveyor*] The conveyor of holes.
        @param[in,out] validHolesMap [std::map<int, float>*] The std::map
        that maps holes inside the allHoles conveyor to their validity
        probability
        @return void
       **/
      void makeValidHolesUnique(HolesConveyor* allHoles,
        std::map<int, float>* validHolesMap);

      /**
        @brief The function called when a debugging parameter is changed
        @param[in] config
        [const pandora_vision_hole_detector::debug_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackDebug(
        const pandora_vision_hole_detector::debug_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief The function called when a parameter regarding the order
        of filters is changed
        @param[in] config
        [const pandora_vision_hole_detector::filters_order_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackFiltersOrder(
        const pandora_vision_hole_detector::filters_order_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief The function called when a parameter regarding thresholds
        of filters is changed
        @param[in] config
        [const pandora_vision_hole_detector::debug_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackFiltersThresholds(
        const pandora_vision_hole_detector::filters_thresholds_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief The function called when a general parameter is changed
        @param[in] config
        [const pandora_vision_hole_detector::general_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackGeneral(
        const pandora_vision_hole_detector::general_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief The function called when a parameter regarding the validity of
        holes is changed
        @param[in] config
        [const pandora_vision_hole_detector::debug_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackValidity(
        const pandora_vision_hole_detector::validity_cfgConfig& config,
        const uint32_t& level);

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
      void pointCloudCallback(const PointCloudPtr& msg);

      /**
        @brief Implements a strategy to combine information from both
        the depth and rgb image and holes sources in order to accurately
        find valid holes.

        It first assimilates all the holes that can be assimilated into other
        ones, amalgamates holes that can be amalgamated with others and
        connectes nearby holes with each other. Then, it passes each of the
        resulting holes through a series of depth-based (if depth analysis
        is possible) filters and rgb-based filters in order to extract a series
        of probabilities for each hole. Each probability is a measure of each
        candidate hole's validity: the more a value of a probability, the more
        a candidate hole is indeed a hole in space. Next, a selection regime
        is implemented in order to assess a hole's validity in the totality
        of the filters it has been through. Finally, information about the
        valid holes is published, along with enhanced information about them.
        @param void
        @return void
       **/
      void processCandidateHoles();

      void produceDataset(
        const HolesConveyor& conveyor,
        const std::vector<std::vector<float> >& probabilities);

      /**
        @brief Publishes the holes' enhanced information.
        @param[in] conveyor [const HolesConveyor&]
        The overall valid holes found by the depth and RGB nodes.
        @param[in] validHolesMap [std::map<int, float>*]
        A map containing the indices of the valid holes inside the conveyor
        and their respective validity probabilities
        @return void
       **/
      void publishEnhancedHoles (const HolesConveyor& conveyor,
        std::map<int, float>* validHolesMap);

      /**
        @brief Publishes an image showing holes found from the Depth node
        and the RGB node.
        @param void
        @return void
       **/
      void publishRespectiveHolesFound();

      /**
        @brief Publishes the valid holes' information.
        @param[in] conveyor [const HolesConveyor&] The overall unique holes
        found by the depth and RGB nodes.
        @param[in] map [std::map<int, float>*] A map containing the indices
        of valid holes inside the conveyor and their respective
        probabilities of validity
        @return void
       **/
      void publishValidHoles(const HolesConveyor& conveyor,
        std::map<int, float>* map);

      /**
        @brief Callback for the candidate holes via the rgb node

        This method sets the RGB image and the candidate holes acquired
        from the rgb node.
        If the depth and point cloud callback counterparts have done
        what must be, it resets the number of ready nodes, unlocks
        the synchronizer and calls for processing of the candidate
        holes.
        @param[in] rgbCandidateHolesVector
        [const vision_communications::CandidateHolesVectorMsg&]
        The message containing the necessary information to filter hole
        candidates acquired through the rgb node
        @return void
       **/
      void rgbCandidateHolesCallback(
        const vision_communications::CandidateHolesVectorMsg&
        rgbCandidateHolesVector);

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
      void setDepthValuesInPointCloud(const cv::Mat& inImage,
        PointCloudPtr* pointCloudPtr);

      /**
        @brief Requests from the synchronizer to process a new point cloud
        @return void
       **/
      void unlockSynchronizer();

      /**
        @brief Unpacks the the HolesConveyor struct for the
        candidate holes, the interpolated depth image and the point cloud
        from the vision_communications::CandidateHolesVectorMsg message
        @param[in] holesMsg
        [vision_communications::CandidateHolesVectorMsg&] The input
        candidate holes message obtained through the depth node
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
        depth image
        @return void
       **/
      void unpackMessage(
        const vision_communications::CandidateHolesVectorMsg& holesMsg,
        HolesConveyor* conveyor,
        cv::Mat* image,
        const std::string& encoding);


    public:

      /**
        @brief The HoleFusion constructor
       **/
      HoleFusion(void);

      /**
        @brief The HoleFusion deconstructor
       **/
      ~HoleFusion(void);

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
      void startTransition(int newState);

      /**
        @brief Completes the transition to a new state
        @param void
        @return void
       **/
      void completeTransition(void);

  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_HOLE_FUSION_H
