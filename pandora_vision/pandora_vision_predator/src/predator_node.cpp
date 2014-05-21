/*  Copyright (c) 2014, Victor Daropoulos
 *  All rights reserved.
 *  
 *  This file is part of Pandora_OpenTLD.

 *  Pandora_OpenTLD is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Pandora_OpenTLD is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Pandora_OpenTLD. If not, see http://www.gnu.org/licenses/.
 */

#include "pandora_vision_predator/predator_node.h"

namespace pandora_vision
{


/**
  @brief Default Constructor
  @return void
**/
Predator::Predator(): _nh()
{
  modelLoaded = false;
  
  getGeneralParams();
  
  ROS_INFO("[predator_node] : Created Predator instance");
  
  semaphore_locked = false;
  
  framecounter = 0;
  
  modelExportFile = exportPath.c_str();
  
  tld = new tld::TLD();
  
  tld->trackerEnabled = true;
  tld->alternating = false;
  tld->learningEnabled = learningEnabled;
    
  tld::DetectorCascade* detectorCascade = tld->detectorCascade;
  
  detectorCascade->varianceFilter->enabled = true;
  detectorCascade->ensembleClassifier->enabled = true;
  detectorCascade->nnClassifier->enabled = true;  
  detectorCascade->useShift = true;
  detectorCascade->shift = 0.1;
  detectorCascade->minScale = -10;
  detectorCascade->maxScale = 10;
  detectorCascade->minSize = 25;
  detectorCascade->numTrees = 15;
  detectorCascade->numFeatures = 15;
  detectorCascade->nnClassifier->thetaTP = 0.65;
  detectorCascade->nnClassifier->thetaFP = 0.5;
  
  _predatorPublisher = _nh.advertise<vision_communications::PredatorAlertMsg>(publisher_topic_name, 1000);
  
  _inputImageSubscriber = _nh.subscribe(imageTopic, 1, &Predator::imageCallback, this);
}

/**
  @brief Default Destructor
  @return void
**/
Predator::~Predator(void)
{
  ROS_INFO("[predator_node] : Destroying Predator instance");
}



static int drag = 0;
static cv::Point point;
static cv::Rect bbox;

/**
  @brief Function for capturing mouse events
  @param event [int] Mouse Even
  @param x [int] Bounding box x-coordinate
  @param y [int] Bounding box y-coordinate
  @param flags [int]
  @param param [void*] Pointer to parameter passed in mousecallback function
**/
static void mouseHandler(int event, int x, int y, int flags, void *param)
{
  
  cv::Mat *img = (cv::Mat *)param;

  /* user press left button */
  if (event == CV_EVENT_LBUTTONDOWN && !drag) 
  {
    point = cv::Point(x, y);
    drag = 1;
  }

  /* user drag the mouse */
  if (event == CV_EVENT_MOUSEMOVE && drag) 
  {
    cv::Mat imgCopy;
    //img->copyTo(imgCopy);
    imgCopy = img->clone();

    cv::rectangle(imgCopy, point, cv::Point(x, y), CV_RGB(255, 0, 0), 3, 8, 0);
    cv::imshow("tld", imgCopy);
    cv::waitKey(20);

  }

  /* user release left button */
  if (event == CV_EVENT_LBUTTONUP && drag) 
  {
    bbox = cv::Rect(point.x, point.y, x - point.x, y - point.y);
    drag = 0;
  }
}

/**
    @brief Callback for the RGB Image
    @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
    @return void
**/

void Predator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  double start = static_cast<double>(cv::getTickCount());
  framecounter++;
  double fps;
  
  char LearningString[10]="";
  char mystring[128];
  
  if(!semaphore_locked)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    PredatorFrame = in_msg -> image.clone();
    if ( PredatorFrame.empty() )
    {
      ROS_ERROR("[predator_node] : No more Frames");
      return;
    }
 
  
    //cv::Mat grey(PredatorFrame.rows, PredatorFrame.cols, CV_8UC1);
    cvtColor(PredatorFrame, grey, CV_BGR2GRAY);
    
    tld->detectorCascade->imgWidth = grey.cols;
    tld->detectorCascade->imgHeight = grey.rows;
    tld->detectorCascade->imgWidthStep = grey.step; 
  
    
    tld->processImage(PredatorFrame);
    
    //fps = cv::getTickFrequency() / (cv::getTickCount() - start);
    
    double end = (cvGetTickCount() - start) / cvGetTickFrequency();

    end = end / 1000000;

    fps = 1 / end;
    
    if(tld->learning)
    {
      snprintf(LearningString, sizeof(LearningString), "Learning");
    }
    
    if(tld->currBB != NULL)
    {
      cv::Scalar rectangleColor = tld->currConf > static_cast<double>(0.7) ? CV_RGB(0, 0, 255) : CV_RGB(255, 255, 0);
      cv::rectangle(PredatorFrame, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 8, 8, 0);
      sendMessage(*tld->currBB, tld->currConf, msg);
    }
    else
    {
      cv::Rect temp = cv::Rect(0, 0, 0, 0);
      sendMessage(temp, 0, msg);
    }
    
  }
  
  snprintf(mystring, sizeof(mystring), "#%d, Posterior %.2f; fps: %.2f, #numwindows:%d, %s",
  framecounter, tld->currConf, fps, tld->detectorCascade->numWindows, LearningString);
  cv::rectangle(PredatorFrame, cv::Point(0, 0), cv::Point(PredatorFrame.cols, 50), CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
  cv::putText(PredatorFrame, mystring, cv::Point(25, 25), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
  
  cv::imshow("tld", PredatorFrame);
  
  int keyCode = cv::waitKey(5)&255;
  
  switch (keyCode)
  {
    
    // draw bounding box
    case 'r':
    //~ if (semaphore_locked && PredatorFrame.empty() == false)
                               //~ break;

    ROS_INFO("Draw Bounding Box and Press Enter.");

    semaphore_locked = true;
    bbox = cv::Rect(-1, -1, -1, -1);
    cv::setMouseCallback("tld", mouseHandler, &PredatorFrame);
    break;
    // end drawing bounding box (Enter)
    
    case '\n':
    
    if (semaphore_locked == false) break;
    ROS_INFO("Initiating Hunt.");
    semaphore_locked = false;
    if (bbox.x == -1 || bbox.y == -1 || bbox.width == -1 || bbox.height == -1)
    {
      ROS_INFO("Invalid bounding box given.");
      break;
    }
    tld->selectObject(grey, &bbox);
    break;
    
    case 'l':
    
    tld->learningEnabled = !tld->learningEnabled;
    ROS_INFO("LearningEnabled: %d\n", tld->learningEnabled);
    break;
    
    case 'a':
    
    tld->alternating = !tld->alternating;
    ROS_INFO("alternating: %d\n", tld->alternating);
    break;
    
    case 'e':
    
    ROS_INFO("Saving model...");
    tld->writeToFile(modelExportFile);
    break;
    
    case 'c':
    
    ROS_INFO("Clearing Model");
    tld->release();
    break;
    
    case 'q':
    
    ROS_INFO("Shutdown Request by User. Quitting...");
    ros::shutdown();  
    break; 
    
    case 'i':
    const char* modelPath = patternPath.c_str();
    tld->release();
    ROS_INFO("Importing Model");
    tld->readFromFile(modelPath);
    break;

    
  }
  
  if(modelLoaded)
  {
    const char* modelPath = patternPath.c_str();
    tld->release();
    ROS_INFO("Importing Model");
    tld->readFromFile(modelPath);
    ROS_INFO("Initiating Hunt");
    modelLoaded = false;
  } 

}

/**
  @brief Checks for file existence
  @return [bool]
**/
  
bool Predator::is_file_exist(const std::string& fileName)
{
  struct stat buffer;
  return(stat (fileName.c_str(), &buffer) == 0);
}

/**
  @brief Get parameters referring to view and frame characteristics
  @return void
**/

void Predator::getGeneralParams()
{
  
  packagePath = ros::package::getPath("pandora_vision_predator");

  //!< Get the path to the pattern used for detection
  if (_nh.hasParam("pattern_path"))
  {
    _nh.getParam("pattern_path", patternPath);
    ROS_DEBUG_STREAM("pattern_path: " << patternPath);
    ROS_INFO("Pattern path loaded from launcher");
    if(is_file_exist(patternPath))
    {
      ROS_INFO("Model Loaded From Launcher");
      modelLoaded = true;
    }
    else
    {
      ROS_INFO("Model Does Not Exist In pattern_path. Awaiting User Input... \n");
    }
  }
  else
  {
    ROS_INFO("Pattern Path Not Found, Waiting User Input \n");
  }
  
  //!<Get Model Export Path
  if(_nh.hasParam("export_path"))
  {
    _nh.getParam("export_path", exportPath);
    ROS_DEBUG_STREAM("export_path: " << exportPath);
    ROS_INFO("Export Path Loaded From Launcher");
  }
  else
  {
    ROS_INFO("Export Path Not Defined. Using Default");
    std:: string temp = "/model";
    exportPath.assign(packagePath);
    exportPath.append(temp);
  }
  
  //!<Get Publisher Topic Name
  if(_nh.hasParam("publisher_topic"))
  {
    _nh.getParam("publisher_topic", publisher_topic_name);
    ROS_DEBUG_STREAM("publisher_topic: " << publisher_topic_name);
    ROS_INFO("Publisher Topic Name Loaded From Launcher");
  }
  else
  {
    ROS_INFO("Publisher Topic Name Not Loaded From Launcher. Using Default");
    publisher_topic_name = "PredatorAlert";
  }
    
  //!< Get value for enabling or disabling TLD learning mode
  
  if(_nh.hasParam("learning_enabled"))
  {
    _nh.getParam("learning_enabled", learningEnabled);
    ROS_INFO("Learning Enabled Value From Launcher");
  }
  else
  {
    ROS_INFO("Learning Enabled Value Not Loaded From Launcher");
  }
  
  //!< Get the camera to be used by predator node;
  if (_nh.hasParam("camera_name"))
  {
    _nh.getParam("camera_name", cameraName);
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  }
  else
  {
    ROS_DEBUG("[predator_node] : Parameter frameHeight not found. Using Default");
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
    ROS_DEBUG("[predator_node] : Parameter frameHeight not found. Using Default");
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
    ROS_DEBUG("[predator_node] : Parameter frameWidth not found. Using Default");
    frameWidth = DEFAULT_WIDTH;
  }

  //!< Get the subscriber's topic;
  if (_nh.hasParam("subscribe_topic"))
  {
    _nh.getParam("subscribe_topic", imageTopic);
    ROS_DEBUG_STREAM("subscribe_topic : " << imageTopic);
  }
  else
  {
    ROS_DEBUG("[predator_node] : Parameter subscribe_topic not found. Using Default");
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
    ROS_DEBUG("[predator_node] : Parameter camera_frame_id not found. Using Default");
    cameraFrameId = "/camera";
  }
}

/**
  @brief Sends message of tracked object
  @param rec [const cv::Rect&] The Bounding Box
  @param posterior [const float&] Confidence
  @return void
**/
  
void Predator::sendMessage(const cv::Rect& rec, const float& posterior, const sensor_msgs::ImageConstPtr& frame)
{
  vision_communications::PredatorAlertMsg predatorAlertMsg;
  
  predatorAlertMsg.x = rec.x;
  predatorAlertMsg.y = rec.y;
  predatorAlertMsg.width = rec.width;
  predatorAlertMsg.height = rec.height;
  predatorAlertMsg.posterior = posterior;
  predatorAlertMsg.img = *frame;
  
  _predatorPublisher.publish(predatorAlertMsg);  
  
}  


} // namespace pandora_vision

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "predator_node");
  pandora_vision::Predator predator;  
  ros::spin();
  return 0;
}

