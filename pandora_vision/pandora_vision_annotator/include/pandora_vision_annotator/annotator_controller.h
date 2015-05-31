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

#ifndef PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONTROLLER_H
#define PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONTROLLER_H

#include "pandora_vision_annotator/annotator_connector.h"
/**
@namespace pandora_vision
@brief The main namespace for pandora vision
**/
namespace pandora_vision
{

  /**
  @class CController
  @brief The main controller for the pandora annotator. Inherits QThread
  **/
  class CController :
    public QThread
  {
    Q_OBJECT

    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int  argc_;
      //!< Input arguments
      char** argv_;

      //!< ROS subscriber for the image topic
      ros::Subscriber img_subscriber_;
      
      //!< ROS subscriber for the predator alert
      ros::Subscriber predatorSubscriber_;

      //!< ROS publisher for the predator 
      ros::Publisher annotationPublisher_;

      //!< The ROS node handle
      ros::NodeHandle n_;
      
      //!< QImage created one time, containing the frame from the input topic
      QImage topic_img_;

      //!< Object of CConnector
      CConnector connector_;

      //!< Vector of frames
      std::vector<cv::Mat> frames;

      //!< Vector of frames
      std::vector<cv::Mat> tempFrames;

      //!< Vector of bag msgs Header
      std::vector<std_msgs::Header> msgHeader_;

      //!< the path of the ros package
      std::string package_path;

      //!< the number of current Frame
      int currentFrameNo_;

      //!< Initial frame to be used with Predator
      int baseFrame;

      //!< offset added to current frame Number
      int offset;

      //!< flag to indicate if annotator is on online onlinemode
      bool onlinemode;

      //!< flag to indicate if Predator is on
      bool PredatorNowOn;

      //!< flag to indicate if backward tracking is enabled
      bool enableBackwardTracking;

      bool depthVisible_;


    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CController(int argc, char **argv);

      /**
      @brief Default destructor
      @return void
      **/
      ~CController(void);

      /**
      @brief Initializes the Qt event connections and ROS subscribers and publishers
      @return void
      **/
      void initializeCommunications(void);

      /**
      @brief Initializes the ROS spin and Qt threads
      @return bool
      **/
      bool init();

      /**
      @brief function that receives pointcloud msg and converts it
      to cv::Mat
      @param msg [const sensor_msgs::PointCloud2Ptr&] the pointcloud msg
      @return void
      **/
      void receivePointCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

      /**
      @brief Function called when new ROS message appears, from any topic
      posting a sensor_msgs kind of msg
      @param msg [const sensor_msgs::ImageConstPtr& ] The message
      @return void
      **/
      void receiveImage(const sensor_msgs::ImageConstPtr& msg);

      /**
      @brief Function called when new ROS message appears, from any topic
      posting an EnhancedImage kind of msg
      @param msg [const pandora_vision_msgs::EnhancedImageConstPtr& ] The message
      @return void
      **/
      void receiveEnhancedImage(const pandora_vision_msgs::EnhancedImageConstPtr& msg);

      /**
      @brief function that loads the bag
      @param filename [const std::string&] the name of the bag file
      @param topic [const std::string&] the topic to be loaded
      @return void
      **/
      void loadBag(const std::string& filename, const std::string& topic);
      /**
      @brief Function called when new ROS message appears, from predator node
      @param msg [const pandora_vision_msgs::PredatorMsg&] The message
      @return void
      **/
      void predatorCallback(const pandora_vision_msgs::Predator& msg);
      /**
      @brief function that prepares and publish the fist msg to 
      be used in predator
      @param initialFrame [const int&] the initial frame number
      @return void
      **/
      void sendInitialFrame(const int &initialFrame);

      /**
      @brief function that prepares and publish the next msg to 
      be used in predator
      @param initialFrame [const int&] the initial frame number
      @return void
      **/
      void sendNextFrame(bool enableBackward);

      /**
      @brief function that prepares and publish the final msg to 
      be used in predator
      @param initialFrame [const int&] the initial frame number
      @return void
      **/
      void sendFinalFrame();


         //------------------------------------------------------------------------//
    public Q_SLOTS:

      /**
      @brief if online mode subscribes to topic else
      calls the loadBag() function when rosTopicGiven signal 
      is received
      @return void
      **/
      void rosTopicGiven(void);

      /**
      @brief sets initial frame and calls function to publish
      first msg to be used in Predator when signal predatorEnabled is
      received
      @return void
      **/
      void predatorEnabled(void);
      
      /**
      @brief Initializes the Qt event connections when onlineMode signal is sent
      @return void
      **/
      void onlineModeGiven(void);
      
      /**
      @brief Initializes the Qt event connections when offlineMode signal is sent
      @return void
      **/
      void offlineModeGiven(void);

      /**
      @brief gets last frame index when appendToFile signal is sent
      @return void
      **/
      void appendToFile(void);

      /**
      @brief Delete annotation file  when removeFile signal is sent
      @return void
      **/
      void removeFile();

      /**
      @brief save all annotated images
      @return void
      **/
      void saveImages(void);

    
    //------------------------------------------------------------------------//
    Q_SIGNALS:
      void updateImage();
  };

}// namespace pandora_vision

#endif  // PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONTROLLER_H
