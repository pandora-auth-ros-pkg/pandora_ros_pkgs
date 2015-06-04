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

#ifndef PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONNECTOR_H
#define PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONNECTOR_H

#include "pandora_vision_annotator/annotator_loader.h"

/**
@namespace pandora_vision
@brief The main namespace for pandora_vision
**/
namespace pandora_vision
{

  enum ImageStates
  {
    IDLE,
    VICTIM_CLICK,
    HOLE_CLICK,
    QR_CLICK,
    HAZMAT_CLICK,
    LANDOLTC_CLICK
  };

  enum States
  {
    OFFLINE,
    ONLINE,
    PREDATOR
  };

  /**
  @class CConnector
  @brief Serves the Qt events of the main GUI window. Inherits from QObject
  **/
  class CConnector: public QObject
  {
    Q_OBJECT

    //------------------------------------------------------------------------//
    private:

      //!< Number of input arguments
      int   argc_;
      //!< Input arguments
      char**  argv_;
      
      //!< vector that shows if bbox ready for display 
      //!< for each category
      std::vector<int> bbox_ready;

      //!< vector of frames
      std::vector<cv::Mat> frames;

      //!< current frame number
      int currFrame;
      
      //!< the offset added to frames
      int offset_;

      //!< the path of the ros package
      std::string package_path;

      //!< the msg Header
      std_msgs::Header msgHeader;
     
      //!< the msg loaded from bag
      sensor_msgs::Image msg_;

      //!< The loader of main GUI QWidget
      CLoader loader_;
      
      //!< the Qimage to be displayed
      QImage localImage_;

      //!< the image state
      ImageStates img_state_;

      //!< the annotator state
      States state_;
      
      /**
      @brief General event filter. Captures all events
      @param watched [QObject*] The object in which the event was triggered
      @param event [QEvent*] The type of event
      @return bool : True is event served
      **/
      bool eventFilter( QObject* watched, QEvent* event );
      
      /**
      @brief Transform QImage to cv::Mat
      @param src [QImage const&] The QImage
      @return cv::Mat
      **/
      cv::Mat QImage2Mat(QImage const& src);

    //------------------------------------------------------------------------//
    public:

      /**
      @brief Default contructor
      @param argc [int] Number of input arguments
      @param argv [char **] Input arguments
      @return void
      **/
      CConnector(int argc, char **argv);
     
      /**
      @brief function that calls the show() function 
      of loader in order to load the ui
      @return void
      **/
      void show(void);
      
      /**
      @brief function that gets the rostopic given 
      by the user
      @return void
      **/
      QString getRosTopic(void);

      /**
      @brief function that gets the bag name given 
      by the user
      @return void
      **/
      QString getBagName(void);
    
      /**
      @brief function that sets the msg Header from bag
      @param msg [const std_msgs::Header&] the msg Header
      @return void
      **/
      void msgTimeStamp(const std_msgs::Header& msg);
      
      /**
      @brief function that sets the msg from bag
      @param msg [const sensor_msgs::ImageConstPtr&] the msg Header
      @return void
      **/
      void setMsg(const sensor_msgs::ImageConstPtr& msg);

      void setPredatorValues(int x, int y, int width, int height, bool initial);
 
      /**
      @brief function that sets the QIMage
      @param img [Qimage] the QImage
      @return void
      **/
      void setImage(const QImage& img);
      
      /**
      @brief function that sets the frames loaded from bag
      @param x [const std::vector<cv::Mat>&] the vector of frames
      @param offset [int] the offset to be used for numbering
      @return void
      **/
      void setFrames(const std::vector<cv::Mat>& x, int offset);

      /**
      @brief function that sets the current frame for display
      transforming it into QImage
      @param x [int] the current frame
      @return void
      **/
      void setcurrentFrame(int x);

      /**
      @brief function that sets the current image state
      @param state [std::string] the current state
      @return void
      **/
      void setState(pandora_vision::States state);

      /**
      @brief function that returns the current Frame
      @param x [int] the current frame number
      @param frame [cv::Mat*] the actual frame matrix
      @return void
      **/
      void getcurrentFrame(int x, cv::Mat* frame);
      
      /**
      @brief function that when is called saves full
      frame annotations for all images
      @return void
      **/
      void setFullFrameAnnotations(void);
      
      /**
      @brief function that returns the current Frame
      @return [int] the frame number
      **/
      int getFrameNumber();
      
      /**
      @brief function that draws annotations to the 
      current QImage
      @return void
      **/
      void drawBox();


    //------------------------------------------------------------------------//
    public Q_SLOTS:

      void rosTopicPushButtonTriggered(void);
      
      /**
      @brief Qt slot when its called updates the Qpixmap for display
      @return void
      **/
      void updateImage();
     
      /**
      @brief Qt slot that is called when the onlineRadioButton is checked
      @return void
     **/
      void onlineRadioButtonChecked(void);

      /**
      @brief Qt slot that is called when the offlineRadioButton is checked
      @return void
      **/
      void offlineRadioButtonChecked(void);
      
      /**
      @brief Qt slot that is called when the appendCheckBOx is checked
      @return void
      **/
      void appendCheckBoxChecked(void);

      /**
      @brief Qt slot that is called when the victim pushButton 
      is triggered. Sets the image State.
      @return void
      **/
      void victimPushButtonTriggered(void);
  
      /**
      @brief Qt slot that is called when the holePushButton 
      is triggered. Sets the image State.
      @return void
      **/
      void holePushButtonTriggered(void);
      
      /**
      @brief Qt slot that is called when the qrPushButton 
      is triggered. Sets the image State.
      @return void
      **/
      void qrPushButtonTriggered(void);
       
      /**
      @brief Qt slot that is called when the landoltcPushButton 
      is triggered. Sets the image State.
      @return void
      **/
      void landoltcPushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the hazmatPushButton 
      is triggered. Sets the image State.
      @return void
      **/
      void hazmatPushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the submitPushButton 
      is triggered. Saves current frame and annotations to file.
      @return void
      **/
      void submitPushButtonTriggered(void);
      
      /**
      @brief Qt slot that is called when the clearPushButton 
      is triggered.delete annotations from current frame and file.
      @return void
      **/
      void clearPushButtonTriggered(void);
       
      /**
      @brief Qt slot that is called when the nextFramePushButton 
      is triggered.display next frame and load annotations from file.
      @return void
      **/
      void nextFramePushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the previousFramePushButton 
      is triggered.display previous frame and load annotations from file.
      @return void
      **/
      void previousFramePushButtonTriggered(void);
      /**
      @brief Qt slot that is called when the predatorPushButton is pressed
      @return void
      **/
      void predatorPushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the removeFilePushButton is pressed
      @return void
      **/
      void removeFilePushButtonTriggered(void);

      /**
      @brief Qt slot that is called when the saveImagesPushButton is pressed
      @return void
      **/
      void saveImagesPushButtonTriggered(void);

      /**
      @brief Qt slot that is called when a QListWidgetItem
      is triggered.display frame and load annotations from file.
      @return void
      **/
      void frameLabelTriggered(QListWidgetItem* item);
  


    //------------------------------------------------------------------------//
    Q_SIGNALS:

      void rosTopicGiven(void);
      void predatorEnabled(void);
      void onlineModeGiven(void);
      void offlineModeGiven(void);
      void appendToFile(void);
      void removeFile(void);
      void saveImages(void);
  };
}// namespace pandora_vision

#endif  // PANDORA_VISION_ANNOTATOR_ANNOTATOR_CONNECTOR_H

