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
* Author: George Aprilis
* 		  Despoina Paschalidou
*********************************************************************/

#include <vision_communications/FaceDirectionMsg.h>

#include "pandora_vision_face/face_detection.h"

namespace pandora_vision
{
/**
  @brief Constructor
**/
FaceDetection::FaceDetection() : _nh(), faceNowON(false)
{
  //!< Get general parameters for image processing
  getGeneralParams();

  //!< Get timer parameters
  getTimerParams();

  //!< Get general parameters for face detection
  getFaceParams();

  //!< Convert field of view from degrees to rads
  hfov = hfov * CV_PI / 180;
  vfov = vfov * CV_PI / 180;

  ratioX = hfov / frameWidth;
  ratioY = vfov / frameHeight;

  if (!boost::filesystem::exists(model_path))
  {
    ROS_WARN("Model file not found, downloading now...");
    std::string cmd = "wget " + model_url + " --no-check-certificate --output-document=" +
                      model_path;
    system(cmd.c_str());
  }

  //!< Initialize face detector
  _faceDetector =	new FaceDetector(cascade_path, model_path, bufferSize,
                                   skinEnabled, skinHist, wallHist, wall2Hist);

  //!< Memory will allocated in the imageCallback
  faceFrame = cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);


  //!< Declare publisher and advertise topic where algorithm
  //!< results are posted
  _victimDirectionPublisher =
    _nh.advertise<vision_communications::FaceDirectionMsg>("face_direction", 10);

  //!< Subscribe to input image's topic
  //!< image_transport::ImageTransport it(_nh);
  _frameSubscriber = image_transport::ImageTransport(_nh).subscribe(
                       imageTopic, 1, &FaceDetection::imageCallback, this);

  //!< Initialize states - robot starts in STATE_OFF
  curState = state_manager_communications::robotModeMsg::MODE_OFF;
  prevState = state_manager_communications::robotModeMsg::MODE_OFF;

  clientInitialize();

  //!< Create and start faceTimer
  faceTimer = _nh.createTimer(ros::Duration(faceTime),
                              &FaceDetection::faceCallback , this);
  faceTimer.start();

  ROS_INFO("[face_node] : Created Face Detection instance");
}

/**
  @brief Destructor
*/
FaceDetection::~FaceDetection()
{
  ROS_DEBUG("[face_node] : Destroying Face Detection instance");
  delete _faceDetector;
}

/**
  @brief Get parameters referring to the timer
  @return void
**/
void FaceDetection::getTimerParams()
{
  //!< Get the FaceDenseTime parameter if available
  if (_nh.hasParam("faceTime"))
  {
    _nh.getParam("faceTime", faceTime);
    ROS_DEBUG_STREAM("faceTime : " << faceTime);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
      Parameter faceDenseTime not found. Using Default");
    faceTime = 0.5;
  }

}

/**
 @brief Get parameters referring to the view and
 *frame characteristics
 @return void
**/
void FaceDetection::getGeneralParams()
{
  packagePath = ros::package::getPath("pandora_vision_face");

  //!< Get the faceDummy parameter if available;
  if (_nh.hasParam("faceDummy"))
  {
    _nh.getParam("faceDummy", faceDummy);
    ROS_DEBUG("faceDummy: %d", faceDummy);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
      Parameter faceDummy not found. Using Default");
    faceDummy = false;
  }

  //!< Get the camera to be used by hole node;
  if (_nh.hasParam("camera_name"))
  {
    _nh.getParam("camera_name", cameraName);
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter frameHeight not found. Using Default");
    cameraName = "camera";
  }

  //!< Get the Height parameter if available;
  if (_nh.hasParam("/" + cameraName + "/image_height"))
  {
    _nh.getParam("/" + cameraName + "/image_height", frameHeight);
    ROS_DEBUG_STREAM("height : " << frameHeight);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter frameHeight not found. Using Default");
    frameHeight = DEFAULT_HEIGHT;
  }

  //!< Get the Width parameter if available;
  if (_nh.hasParam("/" + cameraName + "/image_width"))
  {
    _nh.getParam("/" + cameraName + "/image_width", frameWidth);
    ROS_DEBUG_STREAM("width : " << frameWidth);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
    frameWidth = DEFAULT_WIDTH;
  }

  //!< Get the images's topic;
  if (_nh.hasParam("/" + cameraName + "/topic_name"))
  {
    _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter imageTopic not found. Using Default");
    imageTopic = "/camera_head/image_raw";
  }

  //!< Get the images's frame_id;
  if (_nh.hasParam("/" + cameraName + "/camera_frame_id"))
  {
    _nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
    ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter camera_frame_id not found. Using Default");
    cameraFrameId = "/camera";
  }

  //!< Get the HFOV parameter if available;
  if (_nh.hasParam("/" + cameraName + "/hfov"))
  {
    _nh.getParam("/" + cameraName + "/hfov", hfov);
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
    hfov = HFOV;
  }

  //!< Get the VFOV parameter if available;
  if (_nh.hasParam("/" + cameraName + "/vfov"))
  {
    _nh.getParam("/" + cameraName + "/vfov", vfov);
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  }
  else
  {
    ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
    vfov = VFOV;
  }

}

/**
  @brief Get parameters referring to the face detection algorithm
  @return void
**/
void FaceDetection::getFaceParams()
{

  //!< Get the path of haar_cascade xml file if available;
  if (_nh.hasParam("cascade_path"))
  {
    _nh.getParam("cascade_path", cascade_path);
    ROS_DEBUG_STREAM("cascade_path : " << cascade_path);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter cascadeName not found. Using Default");
    std::string temp = "/data/haarcascade_frontalface_alt_tree.xml";
    cascade_path.assign(packagePath);
    cascade_path.append(temp);
  }

  //!< Get the model.xml url;
  if (_nh.hasParam("model_url"))
  {
    _nh.getParam("model_url", model_url);
    ROS_DEBUG_STREAM("modelURL : " << model_url);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
      Parameter model_url not found. Using Default");
    model_url = "https://pandora.ee.auth.gr/vision/model.xml";
  }

  //!< Get the path of model_path xml file to be loaded
  if (_nh.hasParam("model_path"))
  {
    _nh.getParam("model_path",  model_path);
    ROS_DEBUG_STREAM(" model_path : " <<  model_path);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter model_path not found. Using Default");
    model_path = packagePath + "/data/model.xml";
  }


  if (_nh.hasParam("bufferSize"))
  {
    _nh.getParam("bufferSize", bufferSize);
    ROS_DEBUG_STREAM("bufferSize : " << bufferSize);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter bufferSize not found. Using Default");
    bufferSize = 5;
  }

  if (_nh.hasParam("skinEnabled"))
  {
    _nh.getParam("skinEnabled", skinEnabled);
    ROS_DEBUG_STREAM("skinEnabled : " << skinEnabled);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter skinEnabled not found. Using Default");
    skinEnabled = false;
  }

  if (_nh.hasParam("skinHist"))
  {
    _nh.getParam("skinHist", skinHist);
    ROS_DEBUG_STREAM("skinHist : " << skinHist);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter skinHist not found. Using Default");
    std::string temp = "/data/histogramms/histogramm_skin.jpg";
    skinHist.assign(packagePath);
    skinHist.append(temp);
  }

  if (_nh.hasParam("wallHist"))
  {
    _nh.getParam("wallHist", wallHist);
    ROS_DEBUG_STREAM("wallHist : " << wallHist);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter wallHist not found. Using Default");
    std::string temp = "/data/histogramms/histogramm_wall.jpg";
    wallHist.assign(packagePath);
    wallHist.append(temp);
  }

  if (_nh.hasParam("wall2Hist"))
  {
    _nh.getParam("wall2Hist", wall2Hist);
    ROS_DEBUG_STREAM("wall2Hist : " << wall2Hist);
  }
  else
  {
    ROS_DEBUG("[face_node] : \
        Parameter wall2Hist not found. Using Default");
    std::string temp = "/data/histogramms/histogramm_wall2.jpg";
    wall2Hist.assign(packagePath);
    wall2Hist.append(temp);
  }

}

/**
 * @brief Function called when new ROS message appears, for camera
 * @param msg [const sensor_msgs::ImageConstPtr&] The message
 * @return void
 */
void FaceDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  faceFrame = in_msg->image.clone();
  faceFrameTimestamp = msg->header.stamp;

  if (faceFrame.empty() )
  {
    ROS_ERROR("[face_node] : \
      No more Frames or something went wrong with bag file");
    return;
  }
}

/**
 * @brief This method uses a FaceDetector instance to detect all
 * present faces in a given frame
 * @return void
*/
void FaceDetection::faceCallback(const ros::TimerEvent&)
{
  if(!faceNowON)
  {
    return;
  }

  //!< Create message for FaceDetector
  vision_communications::FaceDirectionMsg faceMessage;

  //!< Hold the center coordinates of face
  float center_x = -1;
  float center_y = -1;

  if (faceDummy)
  {
    createDummyFaceMessage(center_x, center_y, faceMessage);
  }
  else
  {
    createFaceMessage(faceMessage);
  }
}
/**
 * @brief This method creates a message for debbuging reasons
 * @param center_x
 * @param center_y
 * @param faceMessage
 * @return void
*/
void FaceDetection::createDummyFaceMessage(float &center_x,
    float &center_y, vision_communications::FaceDirectionMsg &faceMessage )
{
  for( int i = 0; i < 3; i++)
  {
    center_x = ratioX * ( 300 - frameWidth / 2 );
    center_y = -1 * ratioY * ( 200 + frameHeight / 2 );

    faceMessage.yaw = center_x;
    faceMessage.pitch = center_y;
    faceMessage.header.frame_id = "Face";
    faceMessage.probability = 1;
    faceMessage.header.stamp = ros::Time::now();
    _victimDirectionPublisher.publish(faceMessage);

  }
}

/**
 * @brief This method creates the message to be published
 * @param center_x
 * @param center_y
 * @param faceMessage
 * @return void
*/
void FaceDetection::createFaceMessage(
  vision_communications::FaceDirectionMsg &faceMessage)
{
  //!< start the detection process
  int facesNum = _faceDetector->findFaces(faceFrame);

  //!< if there is a problem with loading, findFaces returns -2
  if (facesNum == -2)
    ROS_ERROR( "[face_node] : Problem with loading Skin images" );

  if(facesNum == 0)
  {
    ROS_DEBUG_NAMED("Face Callback", "Face not found");
  }

  //!< send message if faces are found
  if (facesNum)
  {
    int* facesTable = _faceDetector->getFacePositionTable();
    //!< Send a message for every face found in the frame
    for(int i = 0 ; i < facesNum ; i++)
    {
      faceMessage.yaw = ratioX * ( facesTable[i * 4] -
                                   (double)frameWidth / 2 );
      faceMessage.pitch = -ratioY * ( facesTable[i * 4 + 1] -
                                      (double)frameHeight / 2 );
      faceMessage.header.frame_id = cameraFrameId;
      faceMessage.probability = _faceDetector->getProbability();
      faceMessage.header.stamp = ros::Time::now();
      ROS_INFO("[face_node]:Face found");
      _victimDirectionPublisher.publish(faceMessage);
    }
    delete facesTable;
  }
}


/**
  * @brief Node's state manager
  * @param newState [int] The robot's new state
  * @return void
 */
void FaceDetection::startTransition(int newState)
{

  curState = newState;

  //!< check if face detection algorithm should be running now
  faceNowON =	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
              ( curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH ) ||
              ( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );

  //!< shutdown if the robot is switched off
  if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING)
  {
    ros::shutdown();
    return;
  }

  prevState = curState;

  //!< this needs to be called everytime a node finishes transition
  transitionComplete(curState);
}

/**
 * @brief After completion of state transition
 * @return void
 */
void FaceDetection::completeTransition(void)
{
  ROS_INFO("[Face_node] : Transition Complete");
}
}
