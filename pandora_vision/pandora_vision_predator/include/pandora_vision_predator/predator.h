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


#ifndef PANDORA_VISION_PREDATOR_PREDATOR_NODE_H
#define PANDORA_VISION_PREDATOR_PREDATOR_NODE_H

#include "ros/ros.h"
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdlib.h>
#include <sys/stat.h>

#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "vision_communications/LandoltcPredatorMsg.h"

#include "../tld/TLD.h"
#include <urdf_parser/urdf_parser.h>
#include <map>
#include "state_manager/state_client.h"

#include <dynamic_reconfigure/server.h>
#include <pandora_vision_predator/predator_cfgConfig.h>

#define SHOW_DEBUG_IMAGE 

namespace pandora_vision
{
  
struct DetectorCascadeParams
{
  bool variance_filter;
  bool ensemble_classifier;
  bool nn_classifier;
  bool use_shift;
  double shift;
  int min_scale;
  int max_scale;
  int min_size;
  int num_trees;
  int num_features;
  double theta_TP;
  double theta_FP;
};

class Predator : public StateClient
{
  private:
    
    //!<Node Handler
    ros::NodeHandle _nh;
    
    //!<Subscriber of RGB Image
    ros::Subscriber _inputImageSubscriber;
    
    //!<Predator Publisher
    ros::Publisher _predatorPublisher;
    
    //!<Predator Publisher for use in combination with landoltc3d
    ros::Publisher _landoltc3dPredatorPublisher;
  
    //!< Current frame to be processed
    cv::Mat PredatorFrame;
    
    //!< Grey frame of type 8UC1
    cv::Mat grey;
    
    //!< The topic subscribed to for the front camera
    std::string imageTopic;
    
    //!< Frame height
    int frameHeight;
  
    //!< Frame width
    int frameWidth;
    
    //!<Camera Name

    std::string cameraName;
    
    //!<Frame ID
    std::string _parent_frame_id; 
    std::string _frame_id;
    
    //!<Pointer to TLD Instance
    
    tld::TLD *tld; 
    
    //!<Semaphore used for synchronization
    bool semaphore_locked;
    
    //!<Path of exported model
    const char* modelExportFile;
    
    //!<Path of imported model
    std::string patternPath;
    
    //!<Path of package
    std::string packagePath;
    
    //!<Export Path of model
    std::string exportPath;
    
    //!<Publisher Topic Name
    std::string publisher_topic_name;
    
    //!<Path of imported model
    const char* modelPath;
    
    //!<Boolean value representing if model is loaded
    bool modelLoaded;
    
    //!<Value representing total number of frames
    int framecounter;
    
    //!<Value for enabling or disabling TLD learning mode
    bool learningEnabled;
    
    ///Flag to identify if it works alone or in combination with
    ///landoltc3d_node
    bool operation_state;
    
    /**
    @brief Callback for the RGB Image
    @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
    @return void
    **/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    double hfov;
    double vfov;
    
    /**
    @brief Function that retrieves the parent to the frame_id
    @return bool Returns true is frame_id found or false if not 
    */ 
    bool getParentFrameId();
      
    std::map<std::string, std::string> _frame_ids_map;
    
    bool predatorNowON;  
    
    DetectorCascadeParams detectorCascadeParams;
    
    //!< The dynamic reconfigure parameters' server
    dynamic_reconfigure::Server<pandora_vision_predator::predator_cfgConfig>
    server;

    dynamic_reconfigure::Server<pandora_vision_predator::predator_cfgConfig>::CallbackType f;
    
    /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_predator::predator_cfgConfig&]
    @param[in] level [const uint32_t] The level 
    @return void
    **/
    void parametersCallback(
    const pandora_vision_predator::predator_cfgConfig& config,
    const uint32_t& level);
    
   
    
  public:
  
    static bool show_debug_image;
    /**
    @brief Default Constructor
    @return void
    **/
    explicit Predator(const std::string& n);

    /**
    @brief Get parameters referring to view and frame characteristics
    @return void
    **/
    void getGeneralParams();

    /**
    @brief Checks for file existence
    @return [bool]
    **/
    bool is_file_exist(const std::string& fileName);

    /**
    @brief Sends message of tracked object
    @param rec [const cv::Rect&] The Bounding Box
    @param posterior [const float&] Confidence
    @return void
    **/
    void sendMessage(const cv::Rect& rec, const float& posterior, const sensor_msgs::ImageConstPtr& frame);


    /**
    @brief Default Destructor
    @return void
    **/
    ~Predator();

    std::string param;

    /**
    @brief Node's state manager
    @param newState [int] The robot's new state
    @return void
    */
    void startTransition(int newState);

    /**
    @brief After completion of state transition
    @return void
    */
    void completeTransition(void);

    int curState;
    int prevState;
  
};
} // namespace pandora_vision
#endif  // PANDORA_VISION_PREDATOR_PREDATOR_NODE_H
  
  
