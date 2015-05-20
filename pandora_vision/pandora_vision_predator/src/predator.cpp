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

#include "pandora_vision_predator/predator.h"

namespace pandora_vision
{
  bool Predator::show_debug_image = false;

  /**
  @brief Default Constructor
  @return void
  **/
  Predator::Predator(const std::string& ns): 
    _nh(ns), predatorNowON(false)
  {
    //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";
      
    modelLoaded = false;
      
    getGeneralParams();
    
    //!< The dynamic reconfigure parameter's callback
    server.setCallback(boost::bind(&Predator::parametersCallback, this, _1, _2));
    
      
    //!< Get the path to the pattern used for detection
    std::stringstream model_path_stream;
    model_path_stream << packagePath << "/model";
    
    patternPath = model_path_stream.str();
    if (is_file_exist(patternPath))
      modelLoaded = true;
    else
      ROS_INFO("Model not found!Waiting... <3 ");
      
    //!<Get Model Export Path
    exportPath = model_path_stream.str();
    
    //!< Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;
    
    ROS_INFO("[predator_node] : Created Predator instance");
    
    semaphore_locked = false;
    
    framecounter = 0;
    
    modelExportFile = exportPath.c_str();
    
    tld = new tld::TLD();
    
    tld->trackerEnabled = true;
    tld->alternating = false;
    tld->learningEnabled = learningEnabled;
      
    tld::DetectorCascade* detectorCascade = tld->detectorCascade;
    
    detectorCascade->varianceFilter->enabled = detectorCascadeParams.variance_filter;
    detectorCascade->ensembleClassifier->enabled = detectorCascadeParams.ensemble_classifier;
    detectorCascade->nnClassifier->enabled = detectorCascadeParams.nn_classifier;  
    detectorCascade->useShift = detectorCascadeParams.use_shift;
    detectorCascade->shift = detectorCascadeParams.shift;
    detectorCascade->minScale = detectorCascadeParams.min_scale;
    detectorCascade->maxScale = detectorCascadeParams.max_scale;
    detectorCascade->minSize = detectorCascadeParams.min_size;
    detectorCascade->numTrees = detectorCascadeParams.num_trees;
    detectorCascade->numFeatures = detectorCascadeParams.num_features;
    detectorCascade->nnClassifier->thetaTP = detectorCascadeParams.theta_TP;
    detectorCascade->nnClassifier->thetaFP = detectorCascadeParams.theta_FP;
    
    if (annotations)
    {
      _inputImageSubscriber = _nh.subscribe(annotator_topic_name, 1, &Predator::annotationCallback, this);
    }

    else
    {
      _inputImageSubscriber = _nh.subscribe(imageTopic, 1, &Predator::imageCallback, this);
    }
    //!< initialize states - robot starts in STATE_OFF
    curState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    prevState = state_manager_msgs::RobotModeMsg::MODE_OFF;

    clientInitialize();
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
  static cv::Rect initialbbox;

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
      
      if (Predator::show_debug_image)
      {
        cv::imshow("tld", imgCopy);
        cv::waitKey(20);
      }
    }
    /* user release left button */
    if (event == CV_EVENT_LBUTTONUP && drag) 
    {
      bbox = cv::Rect(point.x, point.y, x - point.x, y - point.y);
      drag = 0;
    }
  }

  /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool Predator::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";
    bool res = _nh.hasParam(model_param_name);

    std::string robot_description = "";

    if (!res || !_nh.getParam(model_param_name, robot_description))
    {
      ROS_ERROR("[Predator_node]:Robot description couldn't be retrieved from the parameter server.");
      return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model(
      urdf::parseURDF(robot_description));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> currentLink = model->getLink(_frame_id);
    if (currentLink)
    {
      boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();
      // Set the parent frame_id to the parent of the frame_id
      _parent_frame_id = parentLink->name;
      return true;
    }
    else
      _parent_frame_id = _frame_id;
      
    return false;
  }
  
  /**
  @brief Callback for the RGB Image
  @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
  @return void
  **/
  void Predator::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (!predatorNowON)
    {
      return;
    }

    double start = static_cast<double>(cv::getTickCount());
    framecounter++;
    double fps;
    
    char LearningString[10]="";
    char mystring[128];
    
    if (!semaphore_locked)
    {
      cv_bridge::CvImagePtr in_msg;
      in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      PredatorFrame = in_msg -> image.clone();
      PredatorFrameTimeStamp = msg->header.stamp;
      _frame_id = msg->header.frame_id;
      
      if (_frame_id.c_str()[0] == '/')
        _frame_id = _frame_id.substr(1);
      
      if (PredatorFrame.empty())
      {
        ROS_ERROR("[predator_node] : No more Frames");
        return;
      }
      
      std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
        
      if (_frame_ids_map.find(_frame_id) == _frame_ids_map.end()) 
      {
        bool _indicator = getParentFrameId();      
        _frame_ids_map.insert( it , std::pair<std::string, std::string>(
        _frame_id, _parent_frame_id));
      } 
      
      cvtColor(PredatorFrame, grey, CV_BGR2GRAY);
      
      tld->detectorCascade->imgWidth = grey.cols;
      tld->detectorCascade->imgHeight = grey.rows;
      tld->detectorCascade->imgWidthStep = grey.step; 
      
      tld->processImage(PredatorFrame);

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
    
    if (Predator::show_debug_image)
    {
      cv::imshow("tld", PredatorFrame);
    
      int keyCode = cv::waitKey(5)&255;
      
      switch (keyCode)
      {
        // draw bounding box
        case 'r':

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
    }
    
    if (modelLoaded)
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
  @brief Callback for the pandora_vision_annotator node
  @param msg [const pandora_vision_msgs::Predator& msg] The annotator msg
  @return void
  **/
  void Predator::annotationCallback(const pandora_vision_msgs::Predator& msg)
  {  
    if (!predatorNowON)
    {
      return;
    }

    if (msg.header.frame_id == "close") 
    {
      ROS_INFO_STREAM("Shutting down Predator node");
      ros::shutdown(); 
    }

    framecounter++;
    double fps;
    
    char LearningString[10] = "";
    char mystring[128];
    
    if (!semaphore_locked)
    {
      cv_bridge::CvImagePtr in_msg;
      in_msg = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::BGR8);
      PredatorFrame = in_msg -> image.clone();
      PredatorFrameTimeStamp = msg.header.stamp;
      _frame_id = msg.header.frame_id;
      
      if (_frame_id.c_str()[0] == '/')
        _frame_id = _frame_id.substr(1);
          
      if (PredatorFrame.empty())
      {
        ROS_ERROR("[predator_node] : No more Frames");
        return;
      }

      std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
        
      if (_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) 
      {
        bool _indicator = getParentFrameId();      
        _frame_ids_map.insert( it , std::pair<std::string, std::string>(
        _frame_id, _parent_frame_id));
      } 
      
      cvtColor(PredatorFrame, grey, CV_BGR2GRAY);
      
      tld->detectorCascade->imgWidth = grey.cols;
      tld->detectorCascade->imgHeight = grey.rows;
      tld->detectorCascade->imgWidthStep = grey.step; 

      if (msg.regionOfInterest.center.x != -1 && msg.regionOfInterest.center.y != -1 
        && msg.regionOfInterest.width != -1 && msg.regionOfInterest.height !=-1) 
      {
        ROS_INFO_STREAM("Starting at: " << msg.regionOfInterest.center.x << " " 
          << msg.regionOfInterest.center.y << " " << msg.regionOfInterest.width << " " 
          << msg.regionOfInterest.height);
        bbox = cv::Rect(msg.regionOfInterest.center.x, msg.regionOfInterest.center.y,
          msg.regionOfInterest.width, msg.regionOfInterest.height);
        tld->selectObject(grey, &bbox); 
        tld->learningEnabled = true;
        sendAnnotation(*tld->currBB, tld->currConf);
        cv::imshow("tld", PredatorFrame);
        cv::waitKey(20);
      }
      else
      { 
        double start = static_cast<double>(cv::getTickCount());
        tld->processImage(PredatorFrame);
        double end = (cvGetTickCount() - start) / cvGetTickFrequency();
        end = end / 1000000;
        fps = 1 / end;
      
        if (tld->learning)
        {
          snprintf(LearningString, sizeof(LearningString), "Learning");
        }
      
        if (tld->currBB != NULL)
        {
          cv::Scalar rectangleColor = tld->currConf > static_cast<double>(0.7) ? CV_RGB(0, 0, 255) : CV_RGB(255, 255, 0);
          cv::rectangle(PredatorFrame, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 8, 8, 0);
          sendAnnotation(*tld->currBB, tld->currConf);
        }
        else
        { 
          cv::Rect temp = cv::Rect(0, 0, 0, 0);
          sendAnnotation(temp, 0);
        }
    
        snprintf(mystring, sizeof(mystring), "#%d, Posterior %.2f; fps: %.2f, #numwindows:%d, %s",
        framecounter, tld->currConf, fps, tld->detectorCascade->numWindows, LearningString);
        cv::rectangle(PredatorFrame, cv::Point(0, 0), cv::Point(PredatorFrame.cols, 50), CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
        cv::putText(PredatorFrame, mystring, cv::Point(25, 25), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
        cv::imshow("tld", PredatorFrame);
        cv::waitKey(20);
      }
    }
  }

  /**
  @brief Checks for file existence
  @return [bool]
  **/
  bool Predator::is_file_exist(const std::string& fileName)
  {
    struct stat buffer;
    return (stat (fileName.c_str(), &buffer) == 0);
  }

  /**
  @brief Get parameters referring to view and frame characteristics
  @return void
  **/
  void Predator::getGeneralParams()
  {
    packagePath = ros::package::getPath("pandora_vision_predator");
     
    //!< Get value of annotations state
    if (_nh.getParam("annotations", annotations))
      ROS_INFO("annotations state is loaded");
    else
    {
      annotations= false;
      ROS_INFO("Unable to load annotations state from launcher");
    }

    //! Publishers
      
    //! Declare publisher and advertise topic
    //! where algorithm results are posted if it works alone or with annotator
    if (_nh.getParam("published_topic_names/predator_alert", param))
    {
      if (annotations)
      {
        _predatorPublisher = 
          _nh.advertise<pandora_vision_msgs::Predator>(param, 1000);
      }
      else
      {
        _predatorPublisher = 
          _nh.advertise<pandora_common_msgs::GeneralAlert>(param, 1000);
      }
    }
    else
    {
      ROS_FATAL("Predator alert topic name not found");
      ROS_BREAK();
    }
    
    //! Declare publisher and advertise topic
    //! where algorithm results are posted if it works in compination with landoltc3d
    if (_nh.getParam("published_topic_names/predator_employment_output", param))
    {
      _landoltc3dPredatorPublisher =
        _nh.advertise<pandora_vision_msgs::Predator>(param, 1000);
    }
    else
    {
      ROS_FATAL("Predator to landoltc alert topic name not found");
      ROS_BREAK();
    }
    
    //! Declare subscriber
    //! if it works with annotator
    if (_nh.getParam("subscribed_topic_names/annotator_topic_name", annotator_topic_name))
      ROS_INFO("[predator_node] : Loaded topic name to use with annotator");
    else
    {
      ROS_FATAL("[predator_node] : annotator topic name not found");
      ROS_BREAK();
    }
    
    //!< Get value for enabling or disabling TLD learning mode
    if( _nh.getParam("learning_enabled", learningEnabled))
      ROS_INFO("Learning Enabled Value From Launcher");
    else
    {
      learningEnabled = false;
      ROS_INFO("Learning Enabled Value Not Loaded From Launcher");
    }
    
    //!< Get value of current operation state
    if( _nh.getParam("operation_state", operation_state))
      ROS_INFO("Operation state is loaded");
    else
    {
      operation_state = true;
      ROS_INFO("Unable to load operation state from launcher");
    }
    
    //!< Get the camera to be used by predator node;
    if (_nh.getParam("camera_name", cameraName))
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    else
    {
      ROS_FATAL("[predator_node] : Camera not found");
      ROS_BREAK();
    }

    //! Get the Height parameter if available;
    if (_nh.getParam("image_height", frameHeight))
      ROS_DEBUG_STREAM("height : " << frameHeight);
    else
    {
      ROS_FATAL("[predtator_node] : Parameter frameHeight not found. ");
      ROS_BREAK();
    }
      
    //! Get the Width parameter if available;
    if (_nh.getParam("image_width", frameWidth))
      ROS_DEBUG_STREAM("width : " << frameWidth);
    else
    {
      ROS_FATAL("[predator_node] : Parameter frameWidth not found");
      ROS_BREAK();
    }
      
    //!< Get the HFOV parameter if available;
    if ( _nh.getParam("hfov", hfov))
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    else
    {
      ROS_FATAL("[predator_node] : Horizontal field of view not found");
      ROS_BREAK();
    }

    //!< Get the VFOV parameter if available;
    if (_nh.getParam("vfov", vfov))
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    else
    {
      ROS_FATAL("[predator_node] : Vertical field of view not found");
      ROS_BREAK();
    }
    
    //!< Get the listener's topic;
    if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    else
    {
      ROS_FATAL("[predator_node] : Imagetopic name not found");
      ROS_BREAK();
    }
    
    //----------------detectorCascadeParams---------------------//
    
    bool param_bool_temp;
    double param_double_temp;
    int param_int_temp;
    
    if (_nh.getParam("variance_filter", param_bool_temp))
    {
      ROS_DEBUG_STREAM("variance_filter : " << param_bool_temp);
      detectorCascadeParams.variance_filter = param_bool_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : variance_filter param not found");
      detectorCascadeParams.variance_filter = true;
    }
    
    if (_nh.getParam("ensemble_classifier", param_bool_temp))
    {
      ROS_DEBUG_STREAM("ensemble_classifier : " << param_bool_temp);
      detectorCascadeParams.ensemble_classifier = param_bool_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : ensemble_classifier param not found");
      detectorCascadeParams.ensemble_classifier = true;
    }
    
    if (_nh.getParam("nn_classifier", param_bool_temp))
    {
      ROS_DEBUG_STREAM("nn_classifier : " << param_bool_temp);
      detectorCascadeParams.nn_classifier = param_bool_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : nn_classifier param not found");
      detectorCascadeParams.nn_classifier = true;
    }
    
    if (_nh.getParam("use_shift", param_bool_temp))
    {
      ROS_DEBUG_STREAM("use_shift : " << param_bool_temp);
      detectorCascadeParams.use_shift = param_bool_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : use_shift param not found");
      detectorCascadeParams.use_shift = true;
    }
    
    if (_nh.getParam("shift", param_double_temp))
    {
      ROS_DEBUG_STREAM("shift : " << param_double_temp);
      detectorCascadeParams.shift = param_double_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : shift param not found");
      detectorCascadeParams.shift = 0.1;
    }
    
    if (_nh.getParam("min_scale", param_int_temp))
    {
      ROS_DEBUG_STREAM("min_scale : " << param_int_temp);
      detectorCascadeParams.min_scale = param_int_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : min_scale param not found");
      detectorCascadeParams.min_scale = -10;
    }
    
    if (_nh.getParam("max_scale", param_int_temp))
    {
      ROS_DEBUG_STREAM("max_scale : " << param_int_temp);
      detectorCascadeParams.max_scale = param_int_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : max_scale param not found");
      detectorCascadeParams.max_scale = 10;
    }
    
    if (_nh.getParam("min_size", param_int_temp))
    {
      ROS_DEBUG_STREAM("min_size : " << param_int_temp);
      detectorCascadeParams.min_size = param_int_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : min_size param not found");
      detectorCascadeParams.min_size = 25;
    }
    
    if (_nh.getParam("num_trees", param_int_temp))
    {
      ROS_DEBUG_STREAM("num_trees : " << param_int_temp);
      detectorCascadeParams.num_trees = param_int_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : num_trees param not found");
      detectorCascadeParams.num_trees = 15;
    }
    
    if (_nh.getParam("num_features", param_int_temp))
    {
      ROS_DEBUG_STREAM("num_features : " << param_int_temp);
      detectorCascadeParams.num_features = param_int_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : num_features param not found");
      detectorCascadeParams.num_features = 15;
    }
    
    if (_nh.getParam("theta_TP", param_double_temp))
    {
      ROS_DEBUG_STREAM("theta_TP : " << param_double_temp);
      detectorCascadeParams.theta_TP = param_double_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : theta_TP param not found");
      detectorCascadeParams.theta_TP = 0.65;
    }
    
    if (_nh.getParam("theta_FP", param_double_temp))
    {
      ROS_DEBUG_STREAM("theta_FP : " << param_double_temp);
      detectorCascadeParams.theta_FP = param_double_temp;
    }
    else
    {
      ROS_WARN("[predator_node] : theta_FP param not found");
      detectorCascadeParams.theta_FP = 0.5;
    }
  }

  /**
  @brief Sends message of tracked object
  @param rec [const cv::Rect&] The Bounding Box
  @param posterior [const float&] Confidence
  @return void
  **/
  void Predator::sendAnnotation(const cv::Rect& rec, const float& posterior) 
  {
    if (operation_state == true)
    {
      pandora_vision_msgs::Predator predatorMsg;
      predatorMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
      predatorMsg.header.stamp = PredatorFrameTimeStamp;
      predatorMsg.regionOfInterest.center.x = rec.x;
      predatorMsg.regionOfInterest.center.y = rec.y;
      predatorMsg.regionOfInterest.width = rec.width;
      predatorMsg.regionOfInterest.height = rec.height;
      predatorMsg.posterior = posterior;

     ROS_INFO_STREAM("send predator alert " << predatorMsg.header.frame_id << " " 
                    << predatorMsg.header.stamp << " "  << predatorMsg.regionOfInterest.center.x << " " 
                    << predatorMsg.regionOfInterest.center.y << " "<< predatorMsg.regionOfInterest.width << " "
                    << predatorMsg.regionOfInterest.height <<  " " << predatorMsg.posterior);
      _predatorPublisher.publish(predatorMsg);
    }
  }

  /**
  @brief Sends message of tracked object
  @param rec [const cv::Rect&] The Bounding Box
  @param posterior [const float&] Confidence
  @return void
  **/
  void Predator::sendMessage(const cv::Rect& rec, const float& posterior, 
      const sensor_msgs::ImageConstPtr& frame)
  {
    if ( operation_state == true)
    {
      pandora_vision_msgs::Predator predatorLandoltcMsg;
      predatorLandoltcMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
      predatorLandoltcMsg.header.stamp = PredatorFrameTimeStamp;
      predatorLandoltcMsg.regionOfInterest.center.x = rec.x;
      predatorLandoltcMsg.regionOfInterest.center.y = rec.y;
      predatorLandoltcMsg.regionOfInterest.width = rec.width;
      predatorLandoltcMsg.regionOfInterest.height = rec.height;
      predatorLandoltcMsg.posterior = posterior;
      predatorLandoltcMsg.image = *frame;
      
      _landoltc3dPredatorPublisher.publish(predatorLandoltcMsg);  
    }
    else
    {
      pandora_common_msgs::GeneralAlert predatorAlertMsg;

      if (posterior != 0)
      {
        predatorAlertMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
        predatorAlertMsg.header.stamp = PredatorFrameTimeStamp;
        predatorAlertMsg.info.probability = posterior;
        int center_x = rec.x + rec.width / 2;
        int center_y = rec.y + rec.height / 2;

        // Predator's center's coordinates relative to the center of the frame
        float x = center_x
          - static_cast<float>(frameWidth) / 2;
        float y = static_cast<float>(frameHeight) / 2
          - center_y;

        // Predator center's yaw and pitch
        predatorAlertMsg.info.yaw = atan(2 * x / frameWidth * tan(hfov / 2));
        predatorAlertMsg.info.pitch = atan(2 * y / frameHeight * tan(vfov / 2));

        _predatorPublisher.publish(predatorAlertMsg);
      }
    }
  }

  /**
  @brief Node's state manager
  @param newState [int] The robot's new state
  @return void
  */
  void Predator::startTransition(int newState)
  {
    curState = newState;

    //!< check if predator algorithm should be running now
    predatorNowON =
      (curState ==
       state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE)
      || (curState ==
          state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION)
      || (curState ==
          state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD)
      || (curState ==
          state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST);

    //!< shutdown if the robot is switched off
    if (curState ==
        state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState = curState;
    
    //!< this needs to be called everytime a node finishes transition
    transitionComplete(curState);
  }

  /**
  @brief After completion of state transition
  @return void
  */
  void Predator::completeTransition()
  {
    ROS_INFO("[Predator_node] : Transition Complete");
  }

  /**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_predator::predator_cfgConfig&]
  @param[in] level [const uint32_t] The level
  @return void
  **/
  void Predator::parametersCallback(
    const pandora_vision_predator::predator_cfgConfig& config,
    const uint32_t& level)
  {
    Predator::show_debug_image = config.show_predator_image;
  }

}  // namespace pandora_vision
