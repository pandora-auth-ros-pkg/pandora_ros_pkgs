/******************************************************************************
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
*   Protopapas Marios <protopapas_marios@hotmail.com>
*   Manos Tsardoulias <etsardou@gmail.com>
*********************************************************************/

#include "pandora_vision_annotator/annotator_controller.h"


namespace pandora_vision
{
  /**
  @brief Thread that performs the ros::spin functionality
  @return void
  **/
  void spinThreadFunction(void)
  {
    ros::spin();
  }

  /**
  @brief Default contructor
  @param argc [int] Number of input arguments
  @param argv [char **] Input arguments
  @return void
  **/
  CController::CController(int argc, char **argv):
    connector_(argc, argv),
    argc_(argc),
    argv_(argv)
  {
    PredatorNowOn = false;
    offset = 0;
  }

  /**
  @brief Default destructor
  @return void
  **/
  CController::~CController(void)
  {

  }

  /**
  @brief Initializes the Qt event connections and ROS subscribers and publishers
  @return void
  **/
  void CController::initializeCommunications(void)
  {
    QObject::connect(
       &connector_, SIGNAL(offlineModeGiven()),
       this, SLOT(offlineModeGiven()));
  
    QObject::connect(
      &connector_, SIGNAL(onlineModeGiven()),
      this, SLOT(onlineModeGiven()));
  
    QObject::connect(
      this, SIGNAL(updateImage()),
      &connector_, SLOT(updateImage())); 
    
    annotationPublisher_ = n_.advertise<pandora_vision_msgs::Predator>("/vision/annotator_output", 1000);
    predatorSubscriber_ = n_.subscribe("/vision/predator_alert", 1, &CController::predatorCallback, this );

  }

  /**
  @brief Initializes the Qt event connections when offlineMode signal is sent
  @return void
  **/
  void CController::offlineModeGiven(void)
  {
     onlinemode = false;
     QObject::connect(
       &connector_, SIGNAL(rosTopicGiven()),
        this, SLOT(rosTopicGiven()));

     QObject::connect(
       &connector_, SIGNAL(predatorEnabled()),
        this, SLOT(predatorEnabled()));

     QObject::connect(
        &connector_, SIGNAL(appendToFile()),
        this, SLOT(appendToFile()));

    QObject::connect(
        &connector_, SIGNAL(removeFile()),
        this, SLOT(removeFile()));
  }      

  /**
  @brief Initializes the Qt event connections when onlineMode signal is sent
  @return void
  **/
  void CController::onlineModeGiven(void)
  {
    onlinemode = true;
    QObject::connect(
      &connector_, SIGNAL(rosTopicGiven()),
      this, SLOT(rosTopicGiven()));
  }

  /**
  @brief Delete annotation file  when removeFile signal is sent
  @return void
  **/
  void CController::removeFile(void)
  {
    std::string package_path = ros::package::getPath("pandora_vision_annotator");
    std::stringstream filename;
    filename << package_path << "/data/annotations.txt";
    ImgAnnotations::removeFile(filename.str());
  }

  /**
  @brief gets last frame index when appendToFile signal is sent
  @return void
  **/
  void CController::appendToFile(void)
  {
    std::string package_path = ros::package::getPath("pandora_vision_annotator");
    std::stringstream filename;
    filename << package_path << "/data/annotations.txt";
    ImgAnnotations::getLastFrameIndex(filename.str(), &offset);
    offset += 1;
    ROS_INFO_STREAM("Frame indexing starting at: " << offset);
  }

  /**
  @brief sets initial frame and calls function to publish
  first msg to be used in Predator when signal predatorEnabled is
  received
  @return void
  **/
  void CController::predatorEnabled(void)
  {
    ROS_INFO("PREDATOR NOW ON");
    cv::Mat temp;
    PredatorNowOn = true;
    baseFrame = connector_.getFrameNumber();
    sendInitialFrame(baseFrame);
    enableBackwardTracking = false;
  }
  
  /**
  @brief if online mode subscribes to topic else
  calls the loadBag() function when rosTopicGiven signal 
  is received
  @return void
  **/
  void CController::rosTopicGiven(void)
  {
    QString ros_topic = connector_.getRosTopic();
    if(onlinemode)
    {
      img_subscriber_ = n_.subscribe(
      ros_topic.toStdString(),
      1,
      &CController::receiveImage,
      this);
    }

    else
    { 
      QString bag_name = connector_.getBagName();
      std::string  package_path = ros::package::getPath("pandora_vision_annotator");
      std::stringstream bag_path;
      currentFrameNo_ = 0;
      bag_path << package_path << "/data/" <<bag_name.toStdString();
      loadBag(bag_path.str(), ros_topic.toStdString());
    }
  }

  /**
  @brief function that loads the bag
  @param filename [const std::string&] the name of the bag file
  @param topic [const std::string&] the topic to be loaded
  @return void
  **/
  void CController::loadBag(const std::string& filename, const std::string& topic)
  {     
    ROS_INFO("Loading bag...");
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    
    // Load all messages 
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      sensor_msgs::Image::ConstPtr img = m.instantiate<sensor_msgs::Image>();
      sensor_msgs::PointCloud2::ConstPtr pc =m.instantiate<sensor_msgs::PointCloud2>();
      
      if(img != NULL)
      {
        receiveImage(img);
      }

      if (pc != NULL)
      {
        receivePointCloud(pc);
      }
    }
    connector_.setFrames(frames, offset);
    connector_.setcurrentFrame(0);
    frames.clear();
  }
  
  /**
  @brief function that receives pointcloud msg and converts it
  to cv::Mat
  @param msg [const sensor_msgs::PointCloud2Ptr&] the pointcloud msg
  @return void
  **/
  void CController::receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*msg, *cloud);
    cv::Mat imageFrame;
    if (cloud->isOrganized())
    {       
      imageFrame = cv::Mat(cloud->height, cloud->width, CV_8UC3);
      for (int h = 0; h < imageFrame.rows; h++)
      {
        for (int w = 0; w < imageFrame.cols; w++)
        {
          pcl::PointXYZRGB point = cloud->at(w, h);
          Eigen::Vector3i rgb = point.getRGBVector3i();
          imageFrame.at<cv::Vec3b>(h, w)[2] = rgb[2];
          imageFrame.at<cv::Vec3b>(h, w)[1] = rgb[1];
          imageFrame.at<cv::Vec3b>(h, w)[0] = rgb[0];
        }
      }
    } 
    frames.push_back(imageFrame);
  }
  
  /**
  @brief Function called when new ROS message appears, from predator node
  @param msg [const pandora_vision_msgs::PredatorMsg&] The message
  @return void
  **/
  void CController::predatorCallback(const pandora_vision_msgs::Predator& msg)
  { 
 
    if(msg.header.stamp == 
        msgHeader_[currentFrameNo_ ].stamp && msg.regionOfInterest.center.x
        != 0 && msg.regionOfInterest.center.y != 0 
      && msg.regionOfInterest.width != 0 && msg.regionOfInterest.height != 0 )
    {
      if(currentFrameNo_ == baseFrame)
       connector_.setPredatorValues(msg.regionOfInterest.center.x,
           msg.regionOfInterest.center.y, msg.regionOfInterest.width,
           msg.regionOfInterest.height, true);
      else
        connector_.setPredatorValues(msg.regionOfInterest.center.x,
            msg.regionOfInterest.center.y, msg.regionOfInterest.width,
            msg.regionOfInterest.height, false);
       ROS_INFO_STREAM("predator alert for frame " << currentFrameNo_);
     
    }
    bool x;
    if(currentFrameNo_ < msgHeader_.size()-1 && !enableBackwardTracking )
    {
      x= false;
      ROS_INFO("SENDING NEXT FRAME ++");
      sendNextFrame(x);
    }
    
    if(currentFrameNo_ == msgHeader_.size()-1 )
    {
      if(baseFrame == 0)
      {
        sendFinalFrame();
      }
      else
      {
        enableBackwardTracking = true;
        sendInitialFrame(baseFrame);
        ROS_INFO_STREAM("ENABLE BACKWARD REINITIALIZING");
      }
    }
    
    if(currentFrameNo_ > 0 && enableBackwardTracking)
    {
      x= true;
      ROS_INFO("SEND NEXT FRAME --");
      sendNextFrame(x);
    }

    if(currentFrameNo_ == 0  && enableBackwardTracking)
    {
      ROS_INFO_STREAM("SEND FINAL FRAME");
      sendFinalFrame();
    }
  }
  
  /**
  @brief function that prepares and publish the fist msg to 
  be used in predator
  @param initialFrame [const int&] the initial frame number
  @return void
  **/
  void CController::sendInitialFrame(const int& initialFrame)
  {
      pandora_vision_msgs::Predator annotationMsg;
      cv::Mat temp;
      currentFrameNo_ = initialFrame; 
      connector_.getcurrentFrame(currentFrameNo_, &temp);
      if(enableBackwardTracking)
      {
      std::stringstream filename;
      std::string package_path = ros::package::getPath("pandora_vision_annotator");
      filename << package_path << "/data/annotations.txt";
      std::string img_name = "frame" + boost::to_string(currentFrameNo_) + ".png";
      //loader_.statusLabel->setText(QString(img_name.c_str()));
      ImgAnnotations::annotations.clear();
      ImgAnnotations::readFromFile(filename.str(), img_name);
      ROS_INFO_STREAM("READING FROM FILE" << img_name);
      }
      
      annotationMsg.header.frame_id = msgHeader_[currentFrameNo_].frame_id;
      annotationMsg.header.stamp = msgHeader_[currentFrameNo_].stamp;
      annotationMsg.regionOfInterest.center.x = ImgAnnotations::annotations[0].x1;
      annotationMsg.regionOfInterest.center.y = ImgAnnotations::annotations[0].y1;
      annotationMsg.regionOfInterest.width =  ImgAnnotations::annotations[0].x2 - ImgAnnotations::annotations[0].x1;
      annotationMsg.regionOfInterest.height = ImgAnnotations::annotations[0].y2 - ImgAnnotations::annotations[0].y1;
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = msgHeader_[currentFrameNo_].frame_id;
      out_msg.header.stamp = msgHeader_[currentFrameNo_].stamp;
      out_msg.encoding = "rgb8";     
      out_msg.image = temp;
      out_msg.toImageMsg(annotationMsg.image);
      annotationPublisher_.publish(annotationMsg);
      ROS_INFO_STREAM("send initial frame" << currentFrameNo_ 
                    << " " << annotationMsg.header.frame_id
                    << " " << annotationMsg.header.stamp 
                    << " " << annotationMsg.regionOfInterest.center.x
                    << " " << annotationMsg.regionOfInterest.center.y
                    << " " << annotationMsg.regionOfInterest.width
                    << " " << annotationMsg.regionOfInterest.height);

  }

  /**
  @brief function that prepares and publish the next msg to 
  be used in predator
  @param initialFrame [const int&] the initial frame number
  @return void
  **/
  void CController::sendNextFrame(bool enableBackward)
  {  
      pandora_vision_msgs::Predator annotationMsg;
      cv::Mat temp;
      if(enableBackward)
        currentFrameNo_--;
      else
        currentFrameNo_+= 1; 
      connector_.getcurrentFrame(currentFrameNo_, &temp);
      annotationMsg.header.frame_id = msgHeader_[currentFrameNo_].frame_id;
      annotationMsg.header.stamp = msgHeader_[currentFrameNo_].stamp;
      annotationMsg.regionOfInterest.center.x = -1;
      annotationMsg.regionOfInterest.center.y = -1;
      annotationMsg.regionOfInterest.width =  -1;
      annotationMsg.regionOfInterest.height = -1;
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = msgHeader_[currentFrameNo_].frame_id;
      out_msg.header.stamp = msgHeader_[currentFrameNo_].stamp;
      out_msg.encoding = "rgb8"; 
      out_msg.image = temp;
      out_msg.toImageMsg(annotationMsg.image);
      annotationPublisher_.publish(annotationMsg);
      connector_.setcurrentFrame(currentFrameNo_);
      ROS_INFO_STREAM("send next frame "<< currentFrameNo_ 
                       << " " << annotationMsg.header.frame_id 
                       << " " << annotationMsg.header.stamp 
                       << " " << annotationMsg.regionOfInterest.center.x 
                       << " " << annotationMsg.regionOfInterest.center.y 
                       << " " << annotationMsg.regionOfInterest.width 
                       << " " << annotationMsg.regionOfInterest.height);

  }

  /**
  @brief function that prepares and publish the final msg to 
  be used in predator
  @param initialFrame [const int&] the initial frame number
  @return void
  **/
  void CController::sendFinalFrame()
  {
    pandora_vision_msgs::Predator annotationMsg;
    cv::Mat temp;
    currentFrameNo_ = 0; 
    connector_.getcurrentFrame(currentFrameNo_, &temp);
    annotationMsg.header.frame_id = "close";
    annotationMsg.header.stamp = msgHeader_[currentFrameNo_].stamp;
    annotationMsg.regionOfInterest.center.x = 2;
    annotationMsg.regionOfInterest.center.y = 2;
    annotationMsg.regionOfInterest.width =  2;
    annotationMsg.regionOfInterest.height = 2;
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id = msgHeader_[currentFrameNo_].frame_id;
    out_msg.header.stamp = msgHeader_[currentFrameNo_].stamp;
    out_msg.encoding = "rgb8"; 
    out_msg.image = temp;
    out_msg.toImageMsg(annotationMsg.image);
    annotationPublisher_.publish(annotationMsg);
    connector_.setcurrentFrame(currentFrameNo_);
    ROS_INFO_STREAM("send 'closing' frame " << currentFrameNo_ 
                     << " " << annotationMsg.header.frame_id 
                     << " " << annotationMsg.header.stamp 
                     << " " << annotationMsg.regionOfInterest.center.x 
                     << " " << annotationMsg.regionOfInterest.center.y 
                     << " " << annotationMsg.regionOfInterest.width 
                     << " " << annotationMsg.regionOfInterest.height);

      predatorSubscriber_.shutdown();

  }
  
  /**
  @brief Function called when new ROS message appears, from any topic
  posting a sensor_msgs kind of msg
  @param msg [const sensor_msgs::ImageConstPtr& ] The message
  @return void
  **/
  void CController::receiveImage(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr in_msg;
    cv::Mat temp;
    try
    {
      in_msg = cv_bridge::toCvCopy(msg);
      if(msg->encoding == "8UC1"  || msg->encoding == "mono8" )
      {
        cv::cvtColor(in_msg->image, temp, CV_GRAY2RGB);
      }
      else if (msg->encoding == "16UC1" || msg->encoding == "32FC1")
      {
        double min, max;
        cv::minMaxLoc(in_msg->image, &min, &max);
        cv::Mat img_scaled_8u;
        cv::Mat(in_msg->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
        cv::cvtColor(img_scaled_8u, temp, CV_GRAY2RGB);
      }

      else if(msg->encoding == "rgb8" || msg->encoding == "bgr8" )
      {
        in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        temp = in_msg->image;
        //cv::cvtColor(in_msg->image, temp, CV_BGR2RGB); // cvtColor Makes a copt, that what i need
      }

    }

    catch(cv_bridge::Exception& e)
    {
      qWarning("while trying to convert image from '%s' to 'rgb8' an exception was thrown ((%s)", 
               msg->encoding.c_str(), e.what());
      return;
    }

    if(!onlinemode)
    {
      frames.push_back(temp);
      msgHeader_.push_back(msg->header);
    }

    else
    {
      QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
      dest.bits(); // enforce deep copy, see documentation
      connector_.setImage(dest);
      connector_.msgTimeStamp(msg->header);
      Q_EMIT updateImage();
    }
  }   
 
 
  /**
  @brief Initializes the ROS spin and Qt threads
  @return bool
  **/

  bool CController::init(void)
  {
    if ( !ros::master::check() )
    {
      return false;
    }
    connector_.show();

    initializeCommunications();
    boost::thread spinThread(&spinThreadFunction);
    return true;
  }
}// namespace pandora_vision


