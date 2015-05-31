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

#ifndef HOLE_FUSION_NODE_HOLE_FUSION_H
#define HOLE_FUSION_NODE_HOLE_FUSION_H

#include <dirent.h>
#include <ros/package.h>
#include <urdf_parser/urdf_parser.h>
#include <image_transport/image_transport.h>
#include "state_manager/state_client.h"
#include "utils/defines.h"
#include "utils/message_conversions.h"
#include "utils/parameters.h"
#include "utils/depth_parameters.h"
#include "utils/visualization.h"


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

      //// The ROS nodehandle needed by the filters_priority_cfg
      //ros::NodeHandle filtersPriorityNodeHandle_;

      //// The ROS nodehandle needed by the filters_thresholds_cfg
      //ros::NodeHandle filtersThresholdsNodeHandle_;

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

      // The publisher used to inform the world that the hole fusion node
      // has finished processing the hole data.
      ros::Publisher processEndPublisher_;

      // The topic where the process end will be advertised.
      std::string processEndTopic_;

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
      // pertaining to the valid holes found by the Hole Exploration package
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

      // The ROS subscriber for acquisition of depth images originated 
      // from the synchronizer node
      ros::Subscriber depthImageSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // acquire depth images from synchronizer node
      std::string depthImageTopic_;

      // The ROS subscriber for acquisition of rgb images originated 
      // from the synchronizer node
      ros::Subscriber rgbImageSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // acquire rgb images from synchronizer node
      std::string rgbImageTopic_;

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

      // The ROS subscriber for acquisition of candidate holes originated
      // from the thermal node
      ros::Subscriber thermalCandidateHolesSubscriber_;

      // The name of the topic where the Hole Fusion node is subscribed to
      // for acquiring information about candidate holes found by the thermal
      // node
      std::string thermalCandidateHolesTopic_;

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

      // The interpolated thermal image received by the thermal node
      cv::Mat interpolatedThermalImage_;

      // The conveyor of hole candidates received by the depth node
      HolesConveyor depthHolesConveyor_;

      // The conveyor of hole candidates received by the rgb node
      HolesConveyor rgbHolesConveyor_;

      // The conveyor of hole candidates received by the thermal node
      HolesConveyor thermalHolesConveyor_;

      // Indicates whether RGB-based only
      // or both the RGB and Depth based filtering is applicable
      int filteringMode_;

      // A vector of histograms for the texture of walls
      std::vector<cv::MatND> wallsHistogram_;

      // The on/off state of the Hole Detector package
      bool isOn_;


      // The dynamic reconfigure server for debugging parameters
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        debug_cfgConfig> serverDebug;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        debug_cfgConfig>::CallbackType f_debug;

      // The dynamic reconfigure server for general parameters
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        general_cfgConfig> serverGeneral;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        general_cfgConfig>::CallbackType f_general;


      // The dynamic reconfigure server for parameters pertaining to
      // the validity of holes
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        validity_cfgConfig> serverValidity;

      // The dynamic reconfigure callback type for the above server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::
        validity_cfgConfig>::CallbackType f_validity;

      /**
        @brief Callback for a depth image via the synchronizer node.
        This method sets the interpolated depth image which will be used 
        for hole validation reasons, once there are any candidate holes
        from rgb or depth nodes.
        @param[in] image [const sensor_msgs::Image&] The depth image msg
        @return void
       **/
      void depthImageCallback(
          const sensor_msgs::Image& image);


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
      void depthCandidateHolesCallback(
          const pandora_vision_msgs::RegionOfInterestVector&
          depthCandidateHolesVector);


      /**
        @brief Callback for a rgb image via the synchronizer node.
        This method sets the rgb image which will be used 
        for hole validation reasons, once there are any candidate holes
        from rgb or depth nodes.
        @param[in] image [const sensor_msgs::Image&] The rgb image msg
        @return void
       **/
      void rgbImageCallback(
          const sensor_msgs::Image& image);

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
      //void thermalCandidateHolesCallback(
      //    const pandora_vision_msgs::CandidateHolesVectorMsg&
      //    thermalCandidateHolesVector);


      /**
        @brief Callback for the candidate holes via the rgb node

        This method sets the RGB image and the candidate holes acquired
        from the rgb node.
        If the depth callback counterpart has done
        what must be, it resets the number of ready nodes, unlocks
        the synchronizer and calls for processing of the candidate
        holes.
        @param[in] rgbCandidateHolesVector
        [const pandora_vision_msgs::RegionOfInterestVector&]
        The message containing the necessary information to filter hole
        candidates acquired through the rgb node
        @return void
       **/
      void rgbCandidateHolesCallback(
          const pandora_vision_msgs::RegionOfInterestVector&
          rgbCandidateHolesVector);


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
        @brief The function called when a debugging parameter is changed
        @param[in] config
        [const pandora_vision_hole::debug_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackDebug(
          const pandora_vision_hole_exploration::debug_cfgConfig& config,
          const uint32_t& level);


      /**
        @brief The function called when a general parameter is changed
        @param[in] config
        [const pandora_vision_hole::general_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackGeneral(
          const pandora_vision_hole_exploration::general_cfgConfig& config,
          const uint32_t& level);

      /**
        @brief The function called when a validity parameter is changed
        @param[in] config
        [const pandora_vision_hole_exploration::validity_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackValidity(
          const pandora_vision_hole_exploration::validity_cfgConfig &config,
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
    public:

      /**
        @brief Implements a strategy to combine information from both
        the depth and rgb image and holes sources in order to accurately
        find valid holes.

        Each probability is a measure of each candidate hole's validity: the more a value of a probability, the more a candidate hole is indeed a hole in space. Finally, information about the valid holes is published.
        @param void
        @return void
       **/
      void processCandidateHoles();

      /**
        @brief Publishes the holes' enhanced information.
        @param[in] conveyor [const HolesConveyor&]
        The overall valid holes found by the depth, thermal and RGB nodes.
        @param[in] validHolesMap [std::map<int, float>*]
        A map containing the indices of the valid holes inside the conveyor
        and their respective validity probabilities
        @return void
       **/
      void publishEnhancedHoles (const HolesConveyor& conveyor,
          std::map<int, float>* validHolesMap);

      /**
        @brief Publishes the valid holes' information.
        @param[in] conveyor [const HolesConveyor&] The overall valid and merged holes found by the depth and RGB nodes.
        @param[in] map [std::map<int, float>*] A map containing the indices
        of valid holes inside the conveyor and their respective
        probabilities of validity
        @return void
       **/
      void publishValidHoles(const HolesConveyor& conveyor,
          std::map<int, float>* map);


      /**
        @brief Applies a validation and merging operation of holes
        @details validate contours based on size and distance from image sensor, validate contours based on variance of depth, merge overlapping contours.
        @param[in,out] rgbHolesConveyor [HolesConveyor*] The rgb
        candidate holes conveyor
        @param[in,out] depthHolesConveyor [HolesConveyor*] The depth
        candidate holes conveyor
        @param[in] image [const cv::Mat&] Depth Image used for validation purposes
        @param[in] pointCloud [const PointCloudPtr] An interpolated point
        cloud used in the connection operation; it maybe will be used to obtain real world
        distances between holes
        @param[in,out] preValidatedHoles [HolesConveyor*] The valid and non double registered holes to publish
        @param[in,out] validHolesMap [std::map<int, float>*] Holes indexes with probabilities
        @return void
       **/
      static void mergeHoles(
          HolesConveyor* rgbHolesConveyor,
          HolesConveyor* depthHolesConveyor,
          HolesConveyor* thermalHolesConveyor,
          const cv::Mat& image,
          const PointCloudPtr& pointCloud,
          HolesConveyor* preValidatedHoles,
          std::map<int, float>* validHolesMap);

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
      static void validateDistance(
          const cv::Mat& image,
          HolesConveyor* holesConveyor,
          std::vector<bool>* realContours,
          const PointCloudPtr& pointCloud);

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
