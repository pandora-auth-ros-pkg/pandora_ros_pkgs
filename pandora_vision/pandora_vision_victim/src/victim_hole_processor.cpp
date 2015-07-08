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

#include <vector>
#include <string>

#include "pandora_vision_victim/victim_hole_processor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  void
  VictimHoleProcessor::initialize(const std::string& ns,
    sensor_processor::Handler* handler)
  {
    sensor_processor::Processor<EnhancedImageStamped,
      POIsStamped>::initialize(ns, handler);

    ros::NodeHandle processor_nh = this->getProcessorNodeHandle();

    paramsPtr_.reset( new VictimParameters(processor_nh) );
    paramsPtr_->configVictim(processor_nh);

    _debugVictimsPublisher = image_transport::ImageTransport(processor_nh).
      advertise(paramsPtr_->victimDebugImg, 1, true);

    interpolatedDepthPublisher_ = image_transport::ImageTransport(processor_nh).
      advertise(paramsPtr_->interpolatedDepthImg, 1, true);

    std::string rgbClassifierType;
    if (!processor_nh.getParam("rgb_classifier", rgbClassifierType))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : " +
          "Could not retrieve the RGB classifier Type from the yaml file!");
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : "
          + "Shutting Down now!");
      ROS_BREAK();
    }

    std::string depthClassifierType;
    if (!processor_nh.getParam("depth_classifier", depthClassifierType))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : " +
          "Could not retrieve the Depth Classifier Type from the yaml file!");
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : " +
          "Shutting Down now!");
      ROS_BREAK();
    }

    std::string rgbdClassifierType;
    if (!processor_nh.getParam("rgbd_classifier", rgbdClassifierType))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : " +
          "Could not retrieve the RGBD Classifier Type from the yaml file!");
      ROS_ERROR_STREAM("[" + this->getName() + "] processor nh processor : " +
          "Shutting Down now!");
      ROS_BREAK();
    }

    validatorFactoryPtr_.reset(new ValidatorFactory);
    rgbValidatorPtr_.reset(validatorFactoryPtr_->createValidator(processor_nh,
          rgbClassifierType, "rgb"));
    depthValidatorPtr_.reset(validatorFactoryPtr_->createValidator(processor_nh,
          depthClassifierType, "depth"));
    rgbdValidatorPtr_.reset(validatorFactoryPtr_->createValidator(processor_nh,
          rgbdClassifierType, "rgbd"));

    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
      processor_nh.getNamespace());
  }

  VictimHoleProcessor::VictimHoleProcessor() : sensor_processor::Processor<EnhancedImageStamped,
    POIsStamped>() {}

  /**
  @brief This method check in which state we are, according to
  the information sent from hole_detector_node
  @return void
  **/
  std::vector<VictimPOIPtr> VictimHoleProcessor::detectVictims(
    const EnhancedImageStampedConstPtr& input)
  {
    if (paramsPtr_->debug_img || paramsPtr_->debug_img_publisher)
    {
      debugImage.clear();
      debugImage.push_back(input->getRgbImage());
      if (input->getDepth())
        debugImage.push_back(input->getDepthImage());
      rgb_svm_keypoints.clear();
      depth_svm_keypoints.clear();
      rgbd_svm_keypoints.clear();
      rgb_svm_bounding_boxes.clear();
      depth_svm_bounding_boxes.clear();
      rgbd_svm_bounding_boxes.clear();
      holes_bounding_boxes.clear();
      rgb_svm_p.clear();
      depth_svm_p.clear();
      rgbd_svm_p.clear();
    }

    DetectionImages imgs;
    int stateIndicator = 2;  //  * input->getDepth() + (input->getRegions().size() > 0) + 1;
    DetectionMode detectionMode;
    switch (stateIndicator)
    {
      case 1:
        detectionMode = GOT_RGB;
        break;
      case 2:
        detectionMode = GOT_HOLES;
        break;
      case 3:
        detectionMode = GOT_DEPTH;
        break;
      case 4:
        detectionMode = GOT_HOLES_AND_DEPTH;
        break;
    }
    for (unsigned int i = 0 ; i < input->getRegions().size(); i++)
    {
      cv::Rect rect(input->getRegion(i).x - input->getRegion(i).width / 2,
        input->getRegion(i).y - input->getRegion(i).height / 2, input->getRegion(i).width,
        input->getRegion(i).height);
      holes_bounding_boxes.push_back(rect);

      if (input->getRegion(i).x - input->getRegion(i).width / 2 <= 0 &&
         input->getRegion(i).y - input->getRegion(i).height / 2 <= 0 &&
         input->getRegion(i).width <= 0 &&
         input->getRegion(i).height <= 0)
      {
        ROS_FATAL("[victim_node] Bounding box sent from hole out of boundaries");
      }

      EnhancedMat emat;
      emat.img = input->getRgbImage()(rect);
      emat.keypoint = cv::Point2f(input->getRegion(i).x, input->getRegion(i).y);
      imgs.rgbMasks.push_back(emat);

      if (GOT_HOLES_AND_DEPTH || GOT_DEPTH)
      {
        emat.img = input->getDepthImage()(rect);
        imgs.depthMasks.push_back(emat);
      }
    }

    std::vector<VictimPOIPtr> final_victims = victimFusion(imgs, detectionMode);

    /// Message alert creation
    for (int i = 0;  i < final_victims.size() ; i++)
    {
      // if (final_victims[i]->getClassLabel() == 1)
      // {
        // counter_++;
        /// Debug purposes
        if (paramsPtr_->debug_img || paramsPtr_->debug_img_publisher)
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
            case DEPTH_SVM:
              depth_svm_keypoints.push_back(kp);
              depth_svm_bounding_boxes.push_back(re);
              depth_svm_p.push_back(final_victims[i]->getProbability());
              break;
            case RGBD_SVM:
              rgbd_svm_keypoints.push_back(kp);
              rgbd_svm_bounding_boxes.push_back(re);
              rgbd_svm_p.push_back(final_victims[i]->getProbability());
          }
        }
      // }
    }

    /// Debug image
    if (paramsPtr_->debug_img || paramsPtr_->debug_img_publisher)
    {
      for (int jj = 0; jj < debugImage.size(); jj++)
      {
        for (unsigned int i = 0 ; i < holes_bounding_boxes.size() ; i++)
        {
          cv::rectangle(debugImage[jj], holes_bounding_boxes[i],
          CV_RGB(100, 100, 150));
        }

        cv::drawKeypoints(debugImage[jj], rgb_svm_keypoints, debugImage[jj],
          CV_RGB(255, 100, 0),
          cv::DrawMatchesFlags::DEFAULT);
        for (unsigned int i = 0 ; i < rgb_svm_bounding_boxes.size() ; i++)
        {
          cv::rectangle(debugImage[jj], rgb_svm_bounding_boxes[i],
            CV_RGB(255, 100, 0));
          {
            std::ostringstream convert;
            convert.str("");
            convert << rgb_svm_p[i];
            cv::putText(debugImage[jj], convert.str().c_str(),
              rgb_svm_keypoints[i].pt,
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
          }
        }

        cv::drawKeypoints(debugImage[jj], depth_svm_keypoints, debugImage[jj],
          CV_RGB(0, 255, 255),
          cv::DrawMatchesFlags::DEFAULT);
        for (unsigned int i = 0 ; i < depth_svm_bounding_boxes.size() ; i++)
        {
          cv::rectangle(debugImage[jj], depth_svm_bounding_boxes[i],
            CV_RGB(0, 255, 255));
          {
            std::ostringstream convert;
            convert.str("");
            convert << depth_svm_p[i];
            cv::putText(debugImage[jj], convert.str().c_str(),
              depth_svm_keypoints[i].pt,
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
          }
        }

        cv::drawKeypoints(debugImage[jj], rgbd_svm_keypoints, debugImage[jj],
          CV_RGB(0, 255, 0),
          cv::DrawMatchesFlags::DEFAULT);
        for (unsigned int i = 0 ; i < rgbd_svm_bounding_boxes.size() ; i++)
        {
          cv::rectangle(debugImage[jj], rgbd_svm_bounding_boxes[i],
            CV_RGB(0, 255, 0));
          {
            std::ostringstream convert;
            convert.str("");
            convert << rgbd_svm_p[i];
            cv::putText(debugImage[jj], convert.str().c_str(),
              rgbd_svm_keypoints[i].pt,
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
          }
        }

        {
          std::ostringstream convert;
          convert.str("");
          convert << "RGB_" << boost::to_upper_copy<std::string>(rgbValidatorPtr_->getClassifierType())
            << " : " << rgb_svm_keypoints.size();
          cv::putText(debugImage[jj], convert.str().c_str(),
            cvPoint(10, 60),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
        }

        {
          std::ostringstream convert;
          convert.str("");
          convert << "Depth_" << boost::to_upper_copy<std::string>(depthValidatorPtr_->getClassifierType())
            << " : " << depth_svm_keypoints.size();
          cv::putText(debugImage[jj], convert.str().c_str(),
            cvPoint(10, 80),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
        }

        {
          std::ostringstream convert;
          convert.str("");
          convert << "RGBD_"
                  << boost::to_upper_copy<std::string>(rgbdValidatorPtr_->getClassifierType())
                  << " : " << rgbd_svm_keypoints.size();
          cv::putText(debugImage[jj], convert.str().c_str(),
            cvPoint(10, 100),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
        }

        {
          std::ostringstream convert;
          convert.str("");
          convert << "Holes got : "<< input->getRegions().size();
          cv::putText(debugImage[jj], convert.str().c_str(),
            cvPoint(10, 120),
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(100, 100, 150), 1, CV_AA);
        }
      }
    }

    if (paramsPtr_->debug_img_publisher)
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = input->getHeader();
      msgPtr->encoding = sensor_msgs::image_encodings::BGR8;
      msgPtr->image = debugImage[0];
      // Publish the image message
      _debugVictimsPublisher.publish(*msgPtr->toImageMsg());
    }

    if (paramsPtr_->debug_img)
    {
      if (debugImage.size() > 1)
      {
        cv::Mat displayImage(debugImage[0].rows, debugImage[0].cols * 2, debugImage[0].type());
        debugImage[0].copyTo(displayImage(cv::Rect(0, 0, debugImage[0].cols, debugImage[0].rows)));
        debugImage[1].copyTo(displayImage(cv::Rect(debugImage[0].cols, 0, debugImage[0].cols, debugImage[0].rows)));
        cv::imshow("Victim Hole processor", displayImage);
        cv::waitKey(30);
      }
      else
      {
        cv::Mat displayImage;
        debugImage[0].copyTo(displayImage);
        cv::imshow("Victim Hole processor", displayImage);
        cv::waitKey(30);
      }
    }

    if (counter_ >= paramsPtr_->positivesCounter)
    {
      counter_ = 0;
    }
    /*
    else
    {
      final_victims.clear();
    }
    */
    return final_victims;
  }

  /**
  @brief Function that enables suitable subsystems, according
  to the current State
  @param [std::vector<cv::Mat>] : vector of images to be processed. Size of
  vector can be either 2 or 1, if we have both rgbd information or not
  @return void
  **/
  std::vector<VictimPOIPtr> VictimHoleProcessor::victimFusion(DetectionImages imgs,
    DetectionMode detectionMode)
  {
    std::vector<VictimPOIPtr> final_probabilities;

    std::vector<VictimPOIPtr> rgb_svm_probabilities;
    std::vector<VictimPOIPtr> depth_svm_probabilities;
    std::vector<VictimPOIPtr> rgbd_svm_probabilities;

    float probability, classLabel;

    if (detectionMode == GOT_HOLES || detectionMode == GOT_HOLES_AND_DEPTH)  // || detectionMode == GOT_RGB
    {
      for (int i = 0 ; i < imgs.rgbMasks.size(); i++)
      {
        VictimPOIPtr temp(new VictimPOI);
        rgbValidatorPtr_->calculatePredictionProbability(imgs.rgbMasks.at(i).img, &classLabel, &probability);
        if (classLabel == 1)
        {
          temp->setProbability(probability);
          temp->setClassLabel(classLabel);
          temp->setPoint(imgs.rgbMasks[i].keypoint);
          temp->setSource(RGB_SVM);
          temp->setWidth(imgs.rgbMasks[i].bounding_box.width);
          temp->setHeight(imgs.rgbMasks[i].bounding_box.height);
          rgb_svm_probabilities.push_back(temp);
          final_probabilities.push_back(temp);
        }
      }
    }

    if (detectionMode == GOT_HOLES_AND_DEPTH)
    {
      if (!paramsPtr_->rgbdEnabled)
      {
        for (int i = 0 ; i < imgs.depthMasks.size(); i++)
        {
          VictimPOIPtr temp(new VictimPOI);
          depthValidatorPtr_->calculatePredictionProbability(imgs.depthMasks.at(i).img, &classLabel, &probability);
          if (classLabel == 1)
          {
            temp->setProbability(probability);
            temp->setClassLabel(classLabel);
            temp->setPoint(imgs.depthMasks[i].keypoint);
            temp->setSource(DEPTH_SVM);
            temp->setWidth(imgs.depthMasks[i].bounding_box.width);
            temp->setHeight(imgs.depthMasks[i].bounding_box.height);
            depth_svm_probabilities.push_back(temp);
            final_probabilities.push_back(temp);
          }
        }
      }
      else
      {
        if (imgs.depthMasks.size() == imgs.rgbMasks.size())
        {
          for (int i = 0 ; i < imgs.depthMasks.size(); i++)
          {
            VictimPOIPtr temp(new VictimPOI);
            rgbdValidatorPtr_->calculatePredictionProbability(imgs.rgbMasks.at(i).img,
                                                            imgs.depthMasks.at(i).img,
                                                            &classLabel, &probability);
            if (classLabel == 1)
            {
              temp->setProbability(probability);
              temp->setClassLabel(classLabel);
              temp->setPoint(imgs.depthMasks[i].keypoint);
              temp->setSource(RGBD_SVM);
              temp->setWidth(imgs.depthMasks[i].bounding_box.width);
              temp->setHeight(imgs.depthMasks[i].bounding_box.height);
              rgbd_svm_probabilities.push_back(temp);
              final_probabilities.push_back(temp);
            }
          }
        }
      }
    }

    // SVM mask merging
    // Combine rgb & depth probabilities
    /*
    if (detectionMode == GOT_HOLES_AND_DEPTH)
    {
      for (unsigned int i = 0 ; i < depth_svm_probabilities.size() ; i++)
      {
        VictimPOIPtr temp(new VictimPOI);
        float mergedProb;
        mergedProb = (depth_svm_probabilities[i]->getClassLabel() * paramsPtr_->depth_svm_weight *
                     depth_svm_probabilities[i]->getProbability() +
                     rgb_svm_probabilities[i]->getClassLabel() * paramsPtr_->rgb_svm_weight *
                     rgb_svm_probabilities[i]->getProbability());

        if (mergedProb >= 0.0f)
        {
          counter_++;
          /// Weighted mean
          temp->setProbability(mergedProb);
          temp->setPoint(depth_svm_probabilities[i]->getPoint());
          temp->setSource(DEPTH_RGB_SVM);
          temp->setWidth(depth_svm_probabilities[i]->getWidth());
          temp->setHeight(depth_svm_probabilities[i]->getHeight());
          final_probabilities.push_back(temp);
        }
        else
          counter_--;

      }
    }
    */
    // Only rgb svm probabilities

    // if (detectionMode == GOT_HOLES )  // || detectionMode == GOT_RGB
    // {
      // for (unsigned int i = 0 ; i < rgb_svm_probabilities.size(); i++)
      // {
        // VictimPOIPtr temp(new VictimPOI);
        // if (rgb_svm_probabilities[i]->getClassLabel() == 1)
        // {
          // counter_++;
          // temp->setProbability(rgb_svm_probabilities[i]->getProbability() *
            // paramsPtr_->rgb_svm_weight);
          // temp->setPoint(rgb_svm_probabilities[i]->getPoint());
          // temp->setSource(RGB_SVM);
          // temp->setWidth(rgb_svm_probabilities[i]->getWidth());
          // temp->setHeight(rgb_svm_probabilities[i]->getHeight());
          // final_probabilities.push_back(temp);
        // }
        // else
          // counter_--;
     // }
    /* } */
    return final_probabilities;
  }

  /**
   * @brief
   **/
  bool VictimHoleProcessor::process(const EnhancedImageStampedConstPtr& input,
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

    /// Interpolated depth image publishing
    // Convert the image into a message
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->header = input->getHeader();
    msgPtr->encoding = sensor_msgs::image_encodings::MONO8;
    input->getDepthImage().copyTo(msgPtr->image);

    // Publish the image message
    interpolatedDepthPublisher_.publish(*msgPtr->toImageMsg());

    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
