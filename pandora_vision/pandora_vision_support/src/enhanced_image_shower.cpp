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
 * Authors: Marios Protopapas
 *********************************************************************/
 
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "pandora_vision_msgs/EnhancedImage.h"


 /**
 * @brief Callback for the requested image topic. Shows the image and
 * provides the keybinding 's' to save the image
 **/
void imageCallback(const pandora_vision_msgs::EnhancedImageConstPtr& msg)
{
    cv_bridge::CvImagePtr rgb_msg, depth_msg;
    cv::Mat temp, temp1;
    std::vector<cv::Mat> imgs;
    std::vector<cv::Rect> bounding_boxes;
    std::vector<cv::KeyPoint> keypoints;


  //!< Current frame to be processed
  if(msg->isDepth)
  {
    depth_msg = cv_bridge::toCvCopy(msg->depthImage);
    if( msg->depthImage.encoding == "8UC1" || msg->depthImage.encoding == "mono8")
    {
      cv::cvtColor(depth_msg->image, temp, CV_GRAY2RGB);
    }

    else if (msg->depthImage.encoding == "16UC1" || msg->depthImage.encoding == "32FC1")
    {
      double min, max;
      cv::minMaxLoc(depth_msg->image, &min, &max);
      cv::Mat img_scaled_8u;
      cv::Mat(depth_msg->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
      cv::cvtColor(img_scaled_8u, temp, CV_GRAY2RGB);
    }

    imgs.push_back(temp);
    rgb_msg = cv_bridge::toCvCopy(msg->rgbImage,
    sensor_msgs::image_encodings::TYPE_8UC3);
    imgs.push_back(rgb_msg->image);

  }
  else
  {
    rgb_msg = cv_bridge::toCvCopy(msg->rgbImage,
    sensor_msgs::image_encodings::TYPE_8UC3);
    // cv::cvtColor(rgb_msg->image, temp, CV_BGR2RGB);
    imgs.push_back(rgb_msg->image);
  }
  for (unsigned int i = 0 ; i < msg->regionsOfInterest.size(); i++)
   {
    cv::Rect rect(msg->regionsOfInterest[i].center.x - msg->regionsOfInterest[i].width / 2,
        msg->regionsOfInterest[i].center.y - msg->regionsOfInterest[i].height / 2, msg->regionsOfInterest[i].width,
        msg->regionsOfInterest[i].height);
     bounding_boxes.push_back(rect);
     keypoints.push_back(cv::KeyPoint(msg->regionsOfInterest[i].center.x,
                            msg->regionsOfInterest[i].center.y, 10));
  }

  for (int j = 0; j < imgs.size(); j++)
  {
    cv::drawKeypoints(imgs[j], keypoints, imgs[j],
          CV_RGB(255, 100, 0),
          cv::DrawMatchesFlags::DEFAULT);
        for (unsigned int i = 0 ; i < bounding_boxes.size() ; i++)
       {
         cv::rectangle(imgs[j], bounding_boxes[i],
           CV_RGB(255, 100, 0));
       }
  }
  if(imgs.size()> 1)
  {
    cv::Mat displayImage(imgs[0].rows, imgs[0].cols * 2, imgs[0].type());
    imgs[0].copyTo(displayImage(cv::Rect(0, 0, imgs[0].cols, imgs[0].rows)));
    imgs[1].copyTo(displayImage(cv::Rect(imgs[1].cols, 0, imgs[1].cols, imgs[1].rows)));
    cv::imshow("EnhancedImage", displayImage);
    ROS_INFO_STREAM("displayImage" << displayImage.type());
    cv::waitKey(10);
  }
  else
  {
    cv::Mat displayImage = imgs[0].clone();
    cv::imshow("EnhancedImage", displayImage);
    ROS_INFO_STREAM("displayImage" << displayImage.type());
    cv::waitKey(10);
  }

}

/**
  @brief Main function of the image_save_by_topic_node
  @param argc [int] Number of input arguments
  @param argv [char**] The input arguments
  @return int : 0 for success
 **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "enhanced_image_shower_node");
  ros::NodeHandle n;
  if(argc != 2 )
  {
    ROS_ERROR("You haven't provided correct input. \nUsage:\n\trosrun \
pandora_vision_support enhanced_image_shower $image_topic \nExiting...");
    exit(0);
  }
  //! Argument parsing
  std::string topic(argv[1]);
  //! Topic subscription
  ROS_WARN("Subscribing to topic %s", topic.c_str());
  ros::Subscriber sub = n.subscribe(topic.c_str(), 1, imageCallback);
  ros::spin();
  return 0;
}
