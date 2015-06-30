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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Angelos Triantafyllidis
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H
#define PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H

#include "utils/message_conversions.h"
#include <utils/defines.h>
#include <utils/parameters.h>
#include <std_msgs/Empty.h>
#include "pandora_vision_msgs/SynchronizedMsg.h"
#include "pandora_vision_msgs/EnhancedImage.h"
#include "pandora_vision_msgs/IndexedThermal.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class RgbDepthSynchronizer
    @brief Responsible for the dissemination of the Depth and RGB images to
    their respective responsible nodes, extracted from a coloured point cloud.
   **/
  class RgbDepthSynchronizer
  {
    private:
      // The ROS node handle
      ros::NodeHandle nodeHandle_;

      // The subscriber to the Rgbd-T synchronizer node that aquires
      // the SynchronizedMsg (pointcloud and thermal synchronized info)
      ros::Subscriber inputSynchronizedSubscriber_;

      // The name of the topic from where the Thermal
      // and pointcloud info is acquired
      std::string inputSynchronizedTopic_;

      // The subscriber to the topic where the hole_fusion node publishes
      // lock/unlock messages concerning the rgb_depth_synchronizer's
      // behaviour
      ros::Subscriber unlockSubscriber_;

      // The subscriber to a topic that the synchronizer is subscribed to,
      // where it is dictated to him explicitly to subscribe to the input
      // point cloud.
      ros::Subscriber subscribeToInputPointCloudSubscriber_;

      // The name of the topic where the Hole Fusion node dictates to the
      // synchronizer node to subscribe to the input point cloud topic
      std::string subscribeToInputPointCloudTopic_;

      // The subscriber to a topic that the synchronizer is subscribed to,
      // where it is dictated to him explicitly to leave the subscription
      // to the input point cloud
      ros::Subscriber leaveSubscriptionToInputPointCloudSubscriber_;

      // The name of the topic where the Hole Fusion node dictates to the
      // synchronizer node to leave its subscription to the input point cloud
      // topic
      std::string leaveSubscriptionToInputPointCloudTopic_;

      // The name of the topic that the Hole Fusion node publishes messages to
      // in order to unlock the synchronizer node
      std::string unlockTopic_;

      // The subscriber to thermal node.
      ros::Subscriber unlockThermalProcedureSubscriber_;

      // The name of the topic that thermal node publishes unlock information
      // to synchronizer node
      std::string unlockThermalProcedureTopic_;

      // The publishers which will advertise the
      // synchronized point cloud, depth and rgb images extracted from the
      // point cloud
      ros::Publisher synchronizedPointCloudPublisher_;
      ros::Publisher synchronizedDepthImagePublisher_;
      ros::Publisher synchronizedRGBImagePublisher_;

      // The publisher which will advertise the synchronized rgb and depth
      // images extracted from the point cloud to thermal cropper node.
      ros::Publisher synchronizedRgbDepthCropperImagesPublisher_;

      // The publisher which will advertise the
      // synchronized thermal information from flir camera
      ros::Publisher synchronizedThermalImagePublisher_;

      // The names of the topic to which the synchronizer node publishes the
      // synchronized point cloud, depth and rgb images extracted from the
      // point cloud
      std::string synchronizedPointCloudTopic_;
      std::string synchronizedDepthImageTopic_;
      std::string synchronizedRgbImageTopic_;

      // The name of the topic to which the synchronizer node publishes the
      // synchronized rgb and depth images to thermal cropper node.
      std::string synchronizedRgbDepthCropperImagesTopic_;

      // The name of the topic to which the synchronizer node publisher the
      // synchronized thermal images
      std::string synchronizedThermalImageTopic_;

      // Here we copy the incoming messages from the Rgbd-T synchronizer node
      PointCloudPtr copiedPc_;

      // Booleans that tell the synchronizer where to publish.
      bool isLocked_;
      bool thermalLocked_;

      // The mode in which the package is running
      // If true Thermal process is enabled, else only Rgb-D.
      bool mode_;

      // Records the time for each synchronizer invocation
      double invocationTime_;

      // Mean invocation interval of time
      double meanProcessingTime_;

      // Amount of synchronizer's invocations
      int ticks_;

      /**
        @brief Variables regarding the point cloud are needed to be set in
        simulation mode: the point cloud's heigth, width and point step.
        This method reads them and sets their respective Parameters.
        @param void
        @return void
       **/
      void getSimulationDimensions();

      /**
        @brief Acquires topics' names needed to be subscribed to and advertise
        to by the synchronizer node
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief The synchronized callback for the point cloud
        obtained by the depth sensor and the thermal info by
        flir camera.

        If the synchronizer node is unlocked, it extracts a depth image from
        the input point cloud's depth measurements, an RGB image from the colour
        measurements of the input point cloud and thermal info.
        Then publishes these images and thermal info
        to their respective recipients. Finally, the input point cloud is
        published directly to the hole fusion node.
        @param[in] synchronizedMessage [const pandora_vision_msgs::SynchronizedMsg&]
        The input synchronized thermal and pc info
        @return void
       **/
      void inputPointCloudThermalCallback(
        const pandora_vision_msgs::SynchronizedMsg& SynchronizedMessage);

      /**
        @brief The callback executed when the Hole Fusion node requests
        from the synchronizer node to leave its subscription to the
        input pointcloud2 and flir topics.
        This happens when the state of the hole detector package is set
        to "off" so as to minimize processing resources.
        @param[in] msg [const std_msgs::Empty&] An empty message used to
        trigger the callback
        @return void
       **/
      void leaveSubscriptionToInputPointCloudCallback(const std_msgs::Empty&
        msg);

      /**
        @brief The callback executed when the Hole Fusion node requests
        from the synchronizer node to subscribe to the input pointcloud2 and flir.
        This happens when the hole detector is in an "off" state, where the
        synchronizer node is not subscribed to the input point cloud(and flir) and
        transitions to an "on" state, where the synchronizer node needs to be
        subscribed to the input point cloud topic in order for the hole detector
        to function.
        @param[in] msg [const std_msgs::Empty&] An empty message used to
        trigger the callback
        @return void
       **/
      void subscribeToInputPointCloudCallback(const std_msgs::Empty& msg);

      /**
        @brief The callback for the hole_fusion node request for the
        lock/unlock of the rgb_depth_synchronizer node.

        The synchronizer node, when subscribed to the input point cloud topic,
        receives in a second as many callbacks as the number of depth sensor's
        point cloud publications per second (currently 25). Because its
        function is to synchronize the input of the depth and rgb nodes,
        it needs to stay locked for the time that these nodes process their
        input. When both these nodes have finished processing their input
        images and the hole fusion node has processed the point cloud sent
        to him by the synchronizer node, the synchronizer
        (who has been locked all this time), can now be unlocked and therefore
        receive a new point cloud.
        @param[in] lockMsg [const std_msgs::Empty] An empty message used to
        trigger the callback
        @return void
       **/
      void unlockCallback(const std_msgs::Empty& lockMsg);

      /**
        @brief The callback from thermal node. Set's a lock variable that is
        responsible for the message that is sent to thermal node from
        synchronizer node.
        @paramp[in] lockMsg [const std_msgs::Empty&] An empty message used to
        trigger the callback for thermal procedure.
       **/
      void unlockThermalProcessCallback(const std_msgs::Empty& lockMsg);

    public:
      /**
        @brief The constructor
       **/
      RgbDepthSynchronizer(void);

      /**
        @brief The default constructor
       **/
      ~RgbDepthSynchronizer(void);
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H
