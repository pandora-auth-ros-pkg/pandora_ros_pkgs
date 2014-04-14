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

#include "utils/message_conversions.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
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
    @param pointCloud[in] [const sensor_msgs::PointCloud2ConstPtr&]
    The input point cloud message
    @param[in] id [const int&] The enconding of the converted image.
    CV_32FC1 for depth image, CV_8UC3 for rgb image
    @return cv::Mat The output image
   **/
  cv::Mat MessageConversions::convertPointCloudMessageToImage(
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage,
    const int& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("convertPointCloudMessageToImage");
    #endif

    PointCloud pointCloud;

    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr
    //!< to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*pointCloudMessage, pointCloud);

    //!< prepare the output image
    cv::Mat image(pointCloud.height, pointCloud.width,
      encoding);

    if (encoding == CV_32FC1)
    {
      //!< convert the point cloud from
      //!< pcl::PCLPointCloud2 to pcl::PointCloudXYZ
      PointCloudXYZPtr pointCloudXYZ (new PointCloudXYZ);
      pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZ);

      for (unsigned int row = 0; row < pointCloudXYZ->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloudXYZ->width; ++col)
        {
          image.at<float>(row, col) =
            pointCloudXYZ->points[col + pointCloudXYZ->width * row].z;

          //!< if element is nan make it a zero
          if (image.at<float>(row, col) !=
            image.at<float>(row, col))
          {
            image.at<float>(row, col) = 0.0;
          }
        }
      }
    }
    else if (encoding == CV_8UC3)
    {
      //!< convert the point cloud from
      //!< pcl::PCLPointCloud2 to pcl::PointCloudXYZRGB
      PointCloudXYZRGBPtr pointCloudXYZRGB (new PointCloudXYZRGB);
      pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZRGB);

      for (unsigned int row = 0; row < pointCloudXYZRGB->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloudXYZRGB->width; ++col)
        {
          image.at<unsigned char>(row, 3 * col + 2) =
            pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].r;
          image.at<unsigned char>(row, 3 * col + 1) =
            pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].g;
          image.at<unsigned char>(row, 3 * col + 0) =
            pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].b;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("convertPointCloudMessageToImage");
    #endif

    return image;
  }



  /**
    @brief Converts a point cloud of type PointCloudXYZPtr to
    a point cloud of type PointCloud and packs it in a message
    @param[in] pointCloudXYZ [const PointCloudXYZPtr&] The point cloud to be
    converted
    @param[out] pointCloud [sensor_msgs::PointCloud2*]
    The converted point cloud message
    @return void
   **/
  void MessageConversions::convertPointCloudXYZToMessage(
    const PointCloudXYZPtr& pointCloudXYZPtr,
    sensor_msgs::PointCloud2* pointCloudMsg)
  {
    #ifdef DEBUG_TIME
    Timer::start("convertPointCloudXYZToMessage");
    #endif

    PointCloud pointCloud;

    //!< Convert the pcl::PointCloud<pcl::PointXYZ>::Ptr aka PointCloudXYZPtr
    //!< to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*pointCloudXYZPtr, pointCloud);

    //!< Pack the point cloud to a ROS message
    pcl_conversions::fromPCL(pointCloud, *pointCloudMsg);

    #ifdef DEBUG_TIME
    Timer::tick("convertPointCloudXYZToMessage");
    #endif
  }



  /**
    @brief Constructs a vision_communications/CandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[in] image [cv::Mat&] The image to be packed in the message
    @param[out] candidateHolesVectorMsg
    [vision_communications::CandidateHolesVectorMsg*] The output message
    @param[in] encoding [std::string&] The image's encoding
    @param[in] msg [const sensor_msgs::Image&] Needed to extract
    its header and place it as the header of the output message
    @return void
   **/
  void MessageConversions::createCandidateHolesVectorMessage(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    vision_communications::CandidateHolesVectorMsg* candidateHolesVectorMsg,
    const std::string& encoding,
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVectorMessage");
    #endif

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< candidateHoles vector
    std::vector<vision_communications::CandidateHoleMsg> candidateHolesVector;
    createCandidateHolesVector(conveyor, &candidateHolesVector);

    candidateHolesVectorMsg->candidateHoles = candidateHolesVector;

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< image
    candidateHolesVectorMsg->image =
      convertImageToMessage(image, encoding, msg);

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< header
    candidateHolesVectorMsg->header = msg.header;

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVectorMessage");
    #endif
  }



  /**
    @brief Constructs a vision_communications/CandidateHolesVectorMsg
    message
    @param[in] conveyor [const HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[in] image [const sensor_msgs::Image&] The image to be packed
    in the message
    @param[out] candidateHolesVectorMsg
    [vision_communications::CandidateHolesVectorMsg*] The output message
    @param[in] msg [const sensor_msgs::Image&] Needed to extract
    its header and place it as the header of the output message
    @return void
   **/
  void MessageConversions::createCandidateHolesVectorMessage(
    const HolesConveyor& conveyor,
    const sensor_msgs::Image& image,
    vision_communications::CandidateHolesVectorMsg* candidateHolesVectorMsg,
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVectorMessage");
    #endif

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< candidateHoles vector
    std::vector<vision_communications::CandidateHoleMsg> candidateHolesVector;
    createCandidateHolesVector(conveyor, &candidateHolesVector);

    candidateHolesVectorMsg->candidateHoles = candidateHolesVector;

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< image
    candidateHolesVectorMsg->image = image;

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< header
    candidateHolesVectorMsg->header = msg.header;

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesMessage");
    #endif
  }



  /**
    @brief Constructs a vision_communications/CandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[out] candidateHolesVector
    [std::vector<vision_communications::CandidateHolesVectorMsg>*]
    The vector containing the conveyor's holes in
    vision_communications::CandidateHolesVectorMsg format
    @return void
   **/
  void MessageConversions::createCandidateHolesVector(
    const HolesConveyor& conveyor,
    std::vector<vision_communications::CandidateHoleMsg>* candidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVector");
    #endif

    //!< Fill the vision_communications::CandidateHolesVectorMsg's
    //!< candidateHoles vector
    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      vision_communications::CandidateHoleMsg holeMsg;

      //!< Push back the keypoint
      holeMsg.keypointX = conveyor.keyPoints[i].pt.x;
      holeMsg.keypointY = conveyor.keyPoints[i].pt.y;

      //!< Push back the bounding rectangle's vertices
      for (int v = 0; v < conveyor.rectangles[i].size(); v++)
      {
        holeMsg.verticesX.push_back(conveyor.rectangles[i][v].x);
        holeMsg.verticesY.push_back(conveyor.rectangles[i][v].y);
      }

      //!< Push back the blob's outline points
      for (int o = 0; o < conveyor.outlines[i].size(); o++)
      {
        holeMsg.outlineX.push_back(conveyor.outlines[i][o].x);
        holeMsg.outlineY.push_back(conveyor.outlines[i][o].y);
      }

      //!< Push back one hole to the holes vector message
      candidateHolesVector->push_back(holeMsg);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVector");
    #endif
  }



  /**
    @brief Extracts a PointCloudXYZPtr (see defines.h)
    from a point cloud message
    @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The input point
    cloud message
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The extracted point cloud
    @return void
   **/
  void MessageConversions::extractPointCloudXYZFromMessage(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    PointCloudXYZPtr* pointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractPointCloudXYZFromMessage");
    #endif

    PointCloud pointCloud;

    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr
    //!< to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*msg, pointCloud);

    //!< Convert the pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>::Ptr
    //!< aka PointCloudXYZPtr
    pcl::fromPCLPointCloud2 (pointCloud, *(*pointCloudXYZ));

    #ifdef DEBUG_TIME
    Timer::tick("extractPointCloudXYZFromMessage");
    #endif
  }



  /**
    @brief Extracts a cv::Mat image from a ROS image message pointer
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS image
    message pointer
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
    const sensor_msgs::ImageConstPtr& msg,
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
    vision_communications::CandidateHolesVectorMsg
    containing the interpolated depth image
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessageContainer(
    const vision_communications::CandidateHolesVectorMsg& msg,
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

} // namespace pandora_vision
