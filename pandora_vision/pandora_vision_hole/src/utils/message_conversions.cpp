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

#include "utils/message_conversions.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  /**
    @brief Converts a cv::Mat image into a sensor_msgs::Image message
    @param[in] image [const cv::Mat&] The image
    @param[in] encoding [const std::string&] The image message's encoding
    @param[in] msg [const sensor_msgs::Image&] A message needed for
    setting the output message's header by extracting its header
    @return [sensor_msgs::Image] The output image message
   **/
  sensor_msgs::Image MessageConversions::convertImageToMessage(
    const cv::Mat& image, const std::string& encoding,
    const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->header = msg.header;
    msgPtr->encoding = encoding;
    msgPtr->image = image;

    return *msgPtr->toImageMsg();
  }



  /**
    @brief Extracts an image from a point cloud message
    @param pointCloud[in] [const PointCloudPtr&]
    The input point cloud message
    @param[in] id [const int&] The enconding of the converted image.
    CV_32FC1 for a depth image, CV_8UC3 for a rgb image
    @return cv::Mat The output image
   **/
  cv::Mat MessageConversions::convertPointCloudMessageToImage(
    const PointCloudPtr& pointCloud, const int& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("convertPointCloudMessageToImage");
    #endif

    // Prepare the output image
    cv::Mat image(pointCloud->height, pointCloud->width, encoding);

    // For the depth image
    if (encoding == CV_32FC1)
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloud->width; ++col)
        {
          image.at<float>(row, col) =
            pointCloud->points[col + pointCloud->width * row].z;

          // if element is nan make it a zero
          if (image.at<float>(row, col) != image.at<float>(row, col))
          {
            image.at<float>(row, col) = 0.0;
          }
        }
      }
    }
    else if (encoding == CV_8UC3)  // For the rgb image
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloud->width; ++col)
        {
          image.at<unsigned char>(row, 3 * col + 2) =
            pointCloud->points[col + pointCloud->width * row].r;
          image.at<unsigned char>(row, 3 * col + 1) =
            pointCloud->points[col + pointCloud->width * row].g;
          image.at<unsigned char>(row, 3 * col + 0) =
            pointCloud->points[col + pointCloud->width * row].b;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("convertPointCloudMessageToImage");
    #endif

    return image;
  }



  /**
    @brief Constructs a pandora_vision_msgs/CandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[out] candidateHolesVector
    [std::vector<::pandora_vision_hole::CandidateHolesVectorMsg>*]
    The vector containing the conveyor's holes in
    ::pandora_vision_hole::CandidateHolesVectorMsg format
    @return void
   **/
  void MessageConversions::createCandidateHolesVector(
    const HolesConveyor& conveyor,
    std::vector< ::pandora_vision_hole::CandidateHoleMsg >* candidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVector");
    #endif

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // candidateHoles vector
    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      ::pandora_vision_hole::CandidateHoleMsg holeMsg;

      // Push back the keypoint
      holeMsg.keypointX = conveyor.holes[i].keypoint.pt.x;
      holeMsg.keypointY = conveyor.holes[i].keypoint.pt.y;

      // Push back the bounding rectangle's vertices
      for (int v = 0; v < conveyor.holes[i].rectangle.size(); v++)
      {
        holeMsg.verticesX.push_back(conveyor.holes[i].rectangle[v].x);
        holeMsg.verticesY.push_back(conveyor.holes[i].rectangle[v].y);
      }

      // Push back the blob's outline points
      for (int o = 0; o < conveyor.holes[i].outline.size(); o++)
      {
        holeMsg.outlineX.push_back(conveyor.holes[i].outline[o].x);
        holeMsg.outlineY.push_back(conveyor.holes[i].outline[o].y);
      }

      // Push back one hole to the holes vector message
      candidateHolesVector->push_back(holeMsg);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVector");
    #endif
  }



  /**
    @brief Constructs a pandora_vision_msgs/CandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[in] image [cv::Mat&] The image to be packed in the message
    @param[out] candidateHolesVectorMsg
    [::::pandora_vision_hole::CandidateHolesVectorMsg*] The output message
    @param[in] encoding [std::string&] The image's encoding
    @param[in] msg [const sensor_msgs::Image&] Needed to extract
    its header and place it as the header of the output message
    @return void
   **/
  void MessageConversions::createCandidateHolesVectorMessage(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    ::pandora_vision_hole::CandidateHolesVectorMsg* candidateHolesVectorMsg,
    const std::string& encoding,
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVectorMessage");
    #endif

    // Fill the ::::pandora_vision_hole::CandidateHolesVectorMsg's
    // candidateHoles vector
    std::vector< ::pandora_vision_hole::CandidateHoleMsg > candidateHolesVector;
    createCandidateHolesVector(conveyor, &candidateHolesVector);

    candidateHolesVectorMsg->candidateHoles = candidateHolesVector;

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // image
    candidateHolesVectorMsg->image =
      convertImageToMessage(image, encoding, msg);

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // header
    candidateHolesVectorMsg->header = msg.header;

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVectorMessage");
    #endif
  }



  /**
    @brief Extracts a cv::Mat image from a ROS image message
    @param[in] msg [const sensor_msgs::Image&] The input ROS image
    message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
    const sensor_msgs::Image& msg,
    cv::Mat* image,
    const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractImageFromMessage");
    #endif

    cv_bridge::CvImagePtr in_msg;

    in_msg = cv_bridge::toCvCopy(msg, encoding);

    *image = in_msg->image.clone();

    #ifdef DEBUG_TIME
    Timer::tick("extractImageFromMessage");
    #endif
  }



  /**
    @brief Extracts a cv::Mat image from a custom ROS message of type
    ::::pandora_vision_hole::CandidateHolesVectorMsg
    containing the interpolated depth image
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessageContainer(
    const ::pandora_vision_hole::CandidateHolesVectorMsg& msg,
    cv::Mat* image, const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractDepthImageFromMessageContainer");
    #endif

    sensor_msgs::Image imageMsg = msg.image;
    extractImageFromMessage(imageMsg, image, encoding);

    #ifdef DEBUG_TIME
    Timer::tick("extractDepthImageFromMessageContainer");
    #endif
  }



  /**
    @brief Recreates the HolesConveyor struct for the candidate holes
    from the pandora_vision_hole::CandidateHolerMsg message
    @param[in] candidateHolesVector
    [const std::vector<::pandora_vision_hole::CandidateHoleMsg>&]
    The input candidate holes
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[in] inImage [const cv::Mat&] An image used for its size.
    It is needed if the wavelet method is used in the keypoints' extraction,
    in order to obtain the coherent shape of holes' outline points
    @param[in] representationMethod [const int&] The @param inImage
    representation method. 0 for normal mode, 1 for wavelet mode
    @param[in] raycastKeypointPartitions [const int&] The number of rays
    used, if @param representationMethod = 1, in order to recreate a
    blob's outline
    @return void
   **/
  void MessageConversions::fromCandidateHoleMsgToConveyor(
    const std::vector< ::pandora_vision_hole::CandidateHoleMsg >&
    candidateHolesVector,
    HolesConveyor* conveyor,
    const cv::Mat& inImage,
    const int& representationMethod,
    const int& raycastKeypointPartitions)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor", "unpackMessage");
    #endif

    // Normal mode
    if (representationMethod == 0)
    {
      for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
      {
        // A single hole
        HoleConveyor hole;

        // Recreate the hole's keypoint
        hole.keypoint.pt.x = candidateHolesVector[i].keypointX;
        hole.keypoint.pt.y = candidateHolesVector[i].keypointY;

        // Recreate the hole's rectangle points
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

        // Recreate the hole's outline points
        std::vector<cv::Point2f> outlinePoints;
        for (unsigned int o = 0;
          o < candidateHolesVector[i].outlineX.size(); o++)
        {
          cv::Point2f outlinePoint;
          outlinePoint.x = candidateHolesVector[i].outlineX[o];
          outlinePoint.y = candidateHolesVector[i].outlineY[o];
          outlinePoints.push_back(outlinePoint);
        }
        hole.outline= outlinePoints;

        // Push hole back into the conveyor
        conveyor->holes.push_back(hole);
      }
    }
    // Wavelet mode
    else if (representationMethod == 1)
    {
      for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
      {
        // A single hole
        HoleConveyor hole;

        // Recreate conveyor.keypoints
        hole.keypoint.pt.x = 2 * candidateHolesVector[i].keypointX;
        hole.keypoint.pt.y = 2 * candidateHolesVector[i].keypointY;

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
        hole.rectangle = renctangleVertices;

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

        for (unsigned int a = 0; a < sparceOutlinePoints.size(); a++)
        {
          unsigned int ind =
            sparceOutlinePoints[a].x + inImage.cols * sparceOutlinePoints[a].y;

          ptr[ind] = 255;
        }

        // The easiest and most efficient way to obtain the same result as
        // if image_representation_method was 0 is to apply the raycast
        // algorithm
        float area = 0.0;
        OutlineDiscovery::raycastKeypoint(hole.keypoint,
          &canvas,
          raycastKeypointPartitions,
          false,
          &hole.outline,
          &area);


        // Push hole back into the conveyor
        conveyor->holes.push_back(hole);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("fromCandidateHoleMsgToConveyor");
    #endif
  }



  /**
    @brief Unpacks the the HolesConveyor struct for the
    candidate holes, the interpolated depth image or the RGB image
    from the ::::pandora_vision_hole::CandidateHolesVectorMsg message
    @param[in] holesMsg
    [::::pandora_vision_hole::CandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HolesConveyor*] The output conveyor
    struct
    @param[out] image [cv::Mat*] The output image
    @param[in] representationMethod [const int&] The @param inImage
    representation method. 0 for normal mode, 1 for wavelet mode
    @param[in] encoding [const std::string&] The encoding used for
    @param[in] raycastKeypointPartitions [const int&] The number of rays
    used, if @param representationMethod = 1, in order to recreate a
    blob's outline
    @return void
   **/
  void MessageConversions::unpackMessage(
    const ::pandora_vision_hole::CandidateHolesVectorMsg& holesMsg,
    HolesConveyor* conveyor,
    cv::Mat* image,
    const int& representationMethod,
    const std::string& encoding,
    const int& raycastKeypointPartitions)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackMessage");
    #endif

    // Unpack the image
    extractImageFromMessageContainer(holesMsg, image, encoding);

    // Recreate the conveyor
    fromCandidateHoleMsgToConveyor(
      holesMsg.candidateHoles,
      conveyor,
      *image,
      representationMethod,
      raycastKeypointPartitions);

    #ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
    #endif
  }

  /**
    @brief Convert the Float32MultiArray data to cv::Mat.
    Its cv format  will be CV_8UC1.
    @param[in] inArray [const std_msgs::Float32MultiArray&]
    The input MultiArray
    @return cv::Mat
   **/
  cv::Mat MessageConversions::convertFloat32MultiArrayToMat(
    const std_msgs::Float32MultiArray& inArray)
  {
    // The width and height of the input temperature multiarray
    int width = inArray.layout.dim[1].size;
    int height = inArray.layout.dim[0].size;

    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

    for (unsigned int i = 0; i < height; i++)
    {
      for (unsigned int j = 0; j < width; j++)
      {
        image.data[i * width + j] = inArray.data[i * width + j];
      }
    }
    return image;
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
