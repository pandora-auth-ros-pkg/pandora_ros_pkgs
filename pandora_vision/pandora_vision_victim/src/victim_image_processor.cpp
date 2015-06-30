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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include <string>
#include <vector>

#include "pandora_vision_victim/victim_image_processor.h"

namespace pandora_vision
{
  VictimImageProcessor::VictimImageProcessor(const std::string& ns,
    sensor_processor::Handler* handler)
    : sensor_processor::Processor<EnhancedImageStamped,
      POIsStamped>(ns, handler)
  {
    params_.configVictim(*this->accessPublicNh());

    _debugVictimsPublisher = image_transport::ImageTransport(
      *this->accessProcessorNh()).advertise(params_.victimDebugImg, 1, true);

    rgbSvmValidatorPtr_.reset(new SvmValidator(*this->accessPublicNh(), "rgb", "svm"));
    depthSvmValidatorPtr_.reset(new SvmValidator(*this->accessPublicNh(), "depth", "svm"));

    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
      this->accessProcessorNh()->getNamespace());
  }

  VictimImageProcessor::VictimImageProcessor()
    : sensor_processor::Processor<EnhancedImageStamped, POIsStamped>()
  {
  }

  VictimImageProcessor::~VictimImageProcessor()
  {
    ROS_DEBUG("[victim_node] : Destroying Victim Image Processor instance");
  }

  std::vector<VictimPOIPtr> VictimImageProcessor::detectVictims(
    const EnhancedImageStampedConstPtr& input)
  {
    if (params_.debug_img || params_.debug_img_publisher)
    {
      input->getRgbImage().copyTo(debugImage);
      rgb_svm_keypoints.clear();
      depth_svm_keypoints.clear();
      rgb_svm_bounding_boxes.clear();
      depth_svm_bounding_boxes.clear();
      rgb_svm_p.clear();
      depth_svm_p.clear();
    }

    bool depthEnable = input->getDepth();

    std::vector<VictimPOIPtr> final_victims = victimFusion(input, depthEnable);

    /// Message alert creation
    for (int i = 0;  i < final_victims.size() ; i++)
    {
      if (final_victims[i]->getClassLabel() == 1)
      {
        /// Debug purposes
        if (params_.debug_img || params_.debug_img_publisher)
        {
          cv::Point victimPOI = final_victims[i]->getPoint();
          victimPOI.x -= final_victims[i]->getWidth() / 2;
          victimPOI.y -= final_victims[i]->getHeight() / 2;
          cv::KeyPoint kp(final_victims[i]->getPoint(), 10);
          cv::Rect re(victimPOI, cv::Size(final_victims[i]->getWidth(),
            final_victims[i]->getHeight()));

          switch (final_victims[i]->getSource())
          {
            case RGB_SVM:
              rgb_svm_keypoints.push_back(kp);
              rgb_svm_bounding_boxes.push_back(re);
              rgb_svm_p.push_back(final_victims[i]->getProbability());
              break;
            case DEPTH_RGB_SVM:
              depth_svm_keypoints.push_back(kp);
              depth_svm_bounding_boxes.push_back(re);
              depth_svm_p.push_back(final_victims[i]->getProbability());
              break;
          }
        }
      }
    }

    /// Debug image
    if (params_.debug_img || params_.debug_img_publisher)
    {
      cv::drawKeypoints(debugImage, rgb_svm_keypoints, debugImage,
        CV_RGB(0, 100, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for (unsigned int i = 0 ; i < rgb_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, rgb_svm_bounding_boxes[i],
          CV_RGB(0, 100, 255));
        {
          std::ostringstream convert;
          convert << rgb_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            rgb_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
        }
      }

      cv::drawKeypoints(debugImage, depth_svm_keypoints, debugImage,
        CV_RGB(0, 255, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for (unsigned int i = 0 ; i < depth_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, depth_svm_bounding_boxes[i],
          CV_RGB(0, 255, 255));
        {
          std::ostringstream convert;
          convert << depth_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            depth_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
        }
      }

      {
        std::ostringstream convert;
        convert << "RGB_SVM : "<< rgb_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 60),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "DEPTH_SVM : "<< depth_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 80),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
      }
    }

    if (params_.debug_img_publisher)
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = input->getHeader();
      msgPtr->encoding = sensor_msgs::image_encodings::BGR8;
      msgPtr->image = debugImage;
      // Publish the image message
      _debugVictimsPublisher.publish(*msgPtr->toImageMsg());
    }

    if (params_.debug_img)
    {
      cv::imshow("Victim Image processor", debugImage);
      cv::waitKey(30);
    }
    return final_victims;
  }

  std::vector<VictimPOIPtr> VictimImageProcessor::victimFusion(const EnhancedImageStampedConstPtr& input,
    bool depthEnable)
  {
    std::vector<VictimPOIPtr> finalProbability;

    VictimPOIPtr rgbSvmProbability(new VictimPOI);
    VictimPOIPtr depthSvmProbability(new VictimPOI);
    cv::Point p;

    // VictimPOIPtr temp(new VictimPOI);
    float probability, classLabel;
    p.x = input->getRgbImage().rows/2;
    p.y = input->getRgbImage().cols/2;
    rgbSvmValidatorPtr_->calculatePredictionProbability(input->getRgbImage(), &classLabel, &probability);
    rgbSvmProbability->setProbability(params_.rgb_svm_weight * probability);
    rgbSvmProbability->setProbability(probability);
    rgbSvmProbability->setClassLabel(classLabel);
    rgbSvmProbability->setPoint(p);  // center of frame???
    rgbSvmProbability->setWidth(input->getRgbImage().rows);
    rgbSvmProbability->setHeight(input->getRgbImage().cols);
    rgbSvmProbability->setSource(RGB_SVM);

    if (depthEnable)
    {
      p.x = input->getDepthImage().rows/2;
      p.y = input->getDepthImage().cols/2;
      depthSvmValidatorPtr_->calculatePredictionProbability(input->getDepthImage(), &classLabel, &probability);
      depthSvmProbability->setProbability(params_.depth_svm_weight * probability);
      depthSvmProbability->setProbability(probability);
      depthSvmProbability->setClassLabel(classLabel);
      depthSvmProbability->setPoint(p);  // center of frame???]
      depthSvmProbability->setWidth(input->getDepthImage().rows);
      depthSvmProbability->setHeight(input->getDepthImage().cols);
      depthSvmProbability->setSource(DEPTH_RGB_SVM);
    }

    finalProbability.push_back(rgbSvmProbability);
    if (depthEnable)
    {
      finalProbability.push_back(depthSvmProbability);
    }

    // ......
    return finalProbability;  // vector??
  }

  bool VictimImageProcessor::process(const EnhancedImageStampedConstPtr& input,
    const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getRgbImage().cols;
    output->frameHeight = input->getRgbImage().rows;

    std::vector<VictimPOIPtr> victims = detectVictims(input);

    for (int ii = 0; ii < victims.size(); ii++)
    {
      if (ii == 0)
      {
        output->pois.clear();
      }
      if (victims[ii]->getProbability() > 0.0001)
      {
        output->pois.push_back(victims[ii]);
      }
    }

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision
