/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include "thermal_node/thermal_cropper.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  ThermalCropper::ThermalCropper(void)
  {
    // Set counter to zero when the node starts
    counter_ = 0;

    // Acquire the names of topics which the thermal_cropper node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the thermal poi published by thermal node
    thermalPoiSubscriber_ = nodeHandle_.subscribe(thermalPoiTopic_, 1,
      &ThermalCropper::inputThermalPoiCallback, this);

    // Subscribe to rgb and depth images published by synchronizer node.
    rgbDepthImagesSubscriber_ = nodeHandle_.subscribe(rgbDepthImagesTopic_, 1,
      &ThermalCropper::inputRgbDepthImagesCallback, this);

    // Advertise enhanced message to victim node
    victimThermalPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::EnhancedImage>(victimThermalTopic_, 1000);

    // Advertise empty message to synchronizer node so the thermal process
    // circle will start again
    unlockThermalProcedurePublisher_ = nodeHandle_.advertise
      <std_msgs::Empty>(unlockThermalProcedureTopic_, 1000, true);

    // When the node starts from launch file dictates thermal procedure to start
    unlockThermalProcedure();

    ROS_INFO_NAMED(PKG_NAME, "[ThermalCropper node] Initiated");
  }

  /**
    @brief Default destructor
    @return void
   **/
  ThermalCropper::~ThermalCropper(void)
  {
    ROS_INFO_NAMED(PKG_NAME, "[ThermalCropper node] Terminated");
  }

  /**
    @brief Callback for the thermal point of interest received 
    by the thermal node.

    The thermal poi message received by the thermal node is unpacked.
    A counter is set. When this counter reach 2 it means both rgb Depth and
    thermal poi message have been subscribed and are ready to be sent to victim. 
    @param msg [const pandora_vision_hole::CandidateHolesVectorMsg&]
    The thermal image message
    @return void
   **/
  void ThermalCropper::inputThermalPoiCallback(
    const pandora_vision_hole::CandidateHolesVectorMsg& msg)
  {
    ROS_INFO("[ThermalCropper node], ThermalPoi callback called");

    // Clear the current thermalHolesConveyor struct
    // (or else keyPoints, rectangles and outlines accumulate)
    HolesConveyorUtils::clear(&thermalHolesConveyor_);

    // Unpack the message and store the acquired information in private variable
    unpackThermalMsg(msg.candidateHoles);

    counter_++;

    if(counter_ == 2)
    {
      // Set counter to zero to restart the process
      counter_ = 0;
 
      unlockThermalProcedure();

      // Fill and publish enhanced message to victim node
      publishEnhancedMsg();
    }
  }


  /**
    @brief Callback for the synchronized rgb and depth images message 
    received by synchronizer node.

    The message received by the synchronizer node is stored in private variable.
    A counter is set. When this counter reaches 2 it means both rgb Depth and
    thermal poi message have been subscribed and are ready to be sent to victim. 
    @param msg [const pandora_vision_msgs::EnhancedImage&]
    The input synchronized rgb and depth images message
    @return void
   **/
  void ThermalCropper::inputRgbDepthImagesCallback(
    const pandora_vision_msgs::EnhancedImage& msg)
  {
    ROS_INFO("[ThermalCropper node], EnhancedImage callback called");

    // Unpack the message and store the acquired information in private variable
    unpackEnhancedMsgFromSynchronizer(msg);

    counter_++;

    if(counter_ == 2)
    {
      // Set counter to zero to restart the process
      counter_ = 0;
 
      unlockThermalProcedure();

      // Fill and publish enhanced message to victim node
      publishEnhancedMsg();
    }
  }

  /**
    @brief When CandidateHolesMsg arrives from thermal node it must be unpacked, 
    in order to be further used. These information is stored in
    private member variable for further use.
    @param[in] msg [const std::vector<pandora_vision_hole::CandidateHoleMsg>&]
    the input candidate holes
    @return void
   **/
  void ThermalCropper::unpackThermalMsg(
    const std::vector<pandora_vision_hole::CandidateHoleMsg>&
    candidateHolesVector)
  {
    for(unsigned int i = 0; i < candidateHolesVector.size(); i++)
    {
      // A singe hole
      HoleConveyor hole;

      // The hole's keypoint
      hole.keypoint.pt.x = candidateHolesVector[i].keypointX;
      hole.keypoint.pt.y = candidateHolesVector[i].keypointY;

      // The hole's rectangle points 
      std::vector<cv::Point2f> renctangleVertices;

      for (unsigned int v = 0;
        v < candidateHolesVector[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = candidateHolesVector[i].verticesX[v];
        vertex.y = candidateHolesVector[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }

      hole.rectangle = renctangleVertices;

      // Push hole back into conveyor
      thermalHolesConveyor_.holes.push_back(hole);
    }
  }

  /**
    @brief When EnhancedMsg arrives from synchronizer node it must be unpacked, 
    in order to be further used. The message that arrives consists of a depth
    image, an rgb image, a thermal image and a boolean variable. 
    These information is stored in private member variables for further use.
    @param[in] msg [const pandora_vision_msgs::EnhancedImage&] the input message
    from synchronizer node.
    @return void
   **/
  void ThermalCropper::unpackEnhancedMsgFromSynchronizer(
    const pandora_vision_msgs:: EnhancedImage& msg)
  {
    headerStamp_ = msg.header.stamp;
    depthImage_ = msg.depthImage;
    rgbImage_ = msg.rgbImage;
    thermalImage_ = msg.thermalImage;

  }

  /**
    @brief When both messages arrive this function is called,fills the 
    final EnhancedMsg and publishes it to victim node.
    @param void
    @return void
   **/
  void ThermalCropper::publishEnhancedMsg()
  {
    // The enhanced message 
    pandora_vision_msgs::EnhancedImage enhancedMsg;

    enhancedMsg.header.stamp = headerStamp_;

    // Victim node needs the interpolated depth image
    // From interpolation we also acquire the isDepth boolean
    enhancedMsg.depthImage = interpolateDepthImage(depthImage_);
    enhancedMsg.rgbImage = rgbImage_;
    enhancedMsg.thermalImage = thermalImage_;
    enhancedMsg.isDepth = isDepth_; 

    // Find keypoint, width and height of each region of interest
    enhancedMsg.regionsOfInterest = 
      findRegionsOfInterest(thermalHolesConveyor_);

    // Publish the enhanced message to victim node
    victimThermalPublisher_.publish(enhancedMsg); 
  }

  /**
    @brief The enhanced messages that is sent to victim node must have the 
    interpolated depth image. So this fuction must extract the image from the 
    message, interpolate it and convert it again to sensor_msgs/Image type.
    @param[in] depthImage [const sensor_msgs::Image&] The input depthImage
    @return [sensor_msgs::Image]
    The interpolated depth image.
   **/
  sensor_msgs::Image ThermalCropper::interpolateDepthImage(
    const sensor_msgs::Image& depthImage)
  {
    cv::Mat depthImageMat;

    // Obtain the depth image. Since the image is in a format of
    // // sensor_msgs::Image, it has to be transformed into a cv format in order
    // // to be processed. Its cv format will be CV_32FC1.
    MessageConversions::extractImageFromMessage(depthImage, &depthImageMat,
      sensor_msgs::image_encodings::TYPE_32FC1);

    // Perform noise elimination on the depth image.
    // Every pixel of noise will be eliminated and substituted by an
    // appropriate non-zero value, depending on the amount of noise present
    // in the input depth image.
    cv::Mat interpolatedDepthImage;
    NoiseElimination::performNoiseElimination(depthImageMat,
      &interpolatedDepthImage);

    // When the depth image is interpolated, we also acquire the interpolation
    // method. Check if depth analysis is applicable.
    if(Parameters::Depth::interpolation_method == 0)
    {
      isDepth_ = true;
    }
    else
    {
      isDepth_ = false;
    }

    // Convert the cv::Mat to sensor_msgs/Image type
    return MessageConversions::convertImageToMessage(interpolatedDepthImage, 
      sensor_msgs::image_encodings::TYPE_32FC1, depthImage);

  }

  /**
    @brief This function finds the keypoint, the width and height of each region
    of interest found by the thermal node.
    @param[in] thermalHoles [const HolesConveyor&] The thermal holes from
    thermal node.
    @return [std::vector<pandora_vision_msgs::RegionOfInterest>]
    The vector of the regions of interest of each hole.
   **/
  std::vector<pandora_vision_msgs::RegionOfInterest> 
    ThermalCropper::findRegionsOfInterest(const HolesConveyor& thermalHoles)
  {
    // The vector of regions of interest that is going to be returned
    std::vector<pandora_vision_msgs::RegionOfInterest> regions;

    for(unsigned int i = 0; i < thermalHoles.size(); i++)
    {
      // The enhanced hole message. Used for one hole only
      pandora_vision_msgs::RegionOfInterest enhancedHoleMsg;

      // Set the keypoint
      enhancedHoleMsg.center.x = thermalHoles.holes[i].keypoint.pt.x;
      enhancedHoleMsg.center.y = thermalHoles.holes[i].keypoint.pt.y;

      // Set the bounding box width and height
      int minx = thermalHoles.holes[i].rectangle[0].x;
      int maxx = thermalHoles.holes[i].rectangle[0].x;
      int miny = thermalHoles.holes[i].rectangle[0].y;
      int maxy = thermalHoles.holes[i].rectangle[0].y;

      for (unsigned int r = 1; r < thermalHoles.holes[i].rectangle.size(); r++)
      {
        int xx = thermalHoles.holes[i].rectangle[r].x;
        int yy = thermalHoles.holes[i].rectangle[r].y;

        minx = xx < minx ? xx : minx;
        maxx = xx > maxx ? xx : maxx;
        miny = yy < miny ? yy : miny;
        maxy = yy > maxy ? yy : maxy;
      }

      enhancedHoleMsg.width = maxx - minx;
      enhancedHoleMsg.height = maxy - miny;

      // Push back into the vector
      regions.push_back(enhancedHoleMsg);
    }
    return regions;
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the thermal_cropper node
    @param void
    @return void
   **/
  void ThermalCropper::getTopicNames ()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the thermalcropper node acquires
    // the thermal poi message and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/subscribed_topics/thermal_poi_topic",
        thermalPoiTopic_ ))
    {
    
      // Make topic's name absolute  
      thermalPoiTopic_ = ns + "/" + thermalPoiTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Subscribed to the input thermal Poi");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_Poi_topic");
    }

    // Read the name of the topic from where the thermalcropper node acquires
    // the synchronized rgb and depth images and store it in 
    // a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/subscribed_topics/rgb_depth_images_topic",
        rgbDepthImagesTopic_ ))
    {
    
      // Make topic's name absolute  
      rgbDepthImagesTopic_ = ns + "/" + rgbDepthImagesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Subscribed to the input Rgb and depth images");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic rgb_depth_images_topic");
    }

     //Read the name of the topic to which the thermal node will be publishing
     //information directly to victim node about the candidate holes found 
     //and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/published_topics/thermal_victim_node_topic",
        victimThermalTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Advertising to the Thermal-victim node topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_victim_node_topic");
    }

     //Read the name of the topic to which the thermal node will be publishing
     //information directly to synchronizer node.
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/published_topics/thermal_unlock_synchronizer_topic",
        unlockThermalProcedureTopic_))
    {
      // Make the topic's name absolute
      unlockThermalProcedureTopic_ = ns + "/" + unlockThermalProcedureTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Advertising to the Synchronizer node topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_unlock_synchronizer_topic");
    }
  }

  /**
    @brief Sends an empty message to dictate synchronizer node to unlock 
    the thermal procedure.
    @param void
    @return void
   **/
  void ThermalCropper::unlockThermalProcedure()
  {
    // Send message to synchronizer in order for thermal procedure to start.
    std_msgs::Empty unlockThermalProcedure;
    unlockThermalProcedurePublisher_.publish(unlockThermalProcedure);
  }

} // namespace pandora_vision
