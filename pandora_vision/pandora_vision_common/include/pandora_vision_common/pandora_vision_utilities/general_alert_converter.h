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
 *********************************************************************/

#ifndef PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_GENERAL_ALERT_CONVERTER_H
#define PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_GENERAL_ALERT_CONVERTER_H

#include <map>
#include <string>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <urdf_parser/urdf_parser.h>

#include <ros/ros.h>

#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_common_msgs/GeneralAlert.h"
#include "pandora_common_msgs/GeneralAlertInfo.h"

#include "pandora_vision_common/poi.h"
#include "pandora_vision_common/poi_stamped.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_common/pandora_vision_interface/vision_exceptions.h"

namespace pandora_vision
{
  class GeneralAlertConverter
  {
   public:
    /**
     * @brief Constructor
     **/
    GeneralAlertConverter() {}

    /**
     * @brief Destructor
     **/
    virtual ~GeneralAlertConverter() {}

    pandora_common_msgs::GeneralAlert
    getGeneralAlert(const ros::NodeHandle& nh, const POIStamped& result,
                    double frameWidth, double frameHeight);

    /**
     * @brief Function that calculates parameters yaw and pitch for every POI, given its
     * coordinates, and puts them in a structure with POI's probability and timestamp
     * @param nh [ros::NodeHandle const&] The NodeHandle of the node using this function
     * @param result [const POIsStampedConstPtr&] A constant reference to a constant shared
     * pointer of a POI with timestamp
     * @return [pandora_common_msgs::GeneralAlertInfoVector] ROS message type that contains
     * yaw, pitch and probability of every POI in the processed frame and the frame's header
     **/
    pandora_common_msgs::GeneralAlertVector
    getGeneralAlertVector(const ros::NodeHandle& nh, const POIsStamped& result);

    POI
    getPOI(const ros::NodeHandle& nh, const pandora_common_msgs::GeneralAlert& result,
           double frameWidth, double frameHeight);

    /**
     * @brief Function that finds in a dictionary the parent frame id with the frame id
     * as key. If the parameter is not found there, the robot model is searched and when
     * the connection to the frame id is found, it is inserted to the dictionary
     * @param nh [ros::NodeHandle const&] The NodeHandle of the node using this function
     * @param key [std::string&] The key used to search in the dictionary
     * @param model_param_name [std::string&] The model parameter name
     * @return [std::string] The parent frame id
     **/
    std::string
    findParentFrameId(const ros::NodeHandle& nh,
                      const std::string& key,
                      const std::string& model_param_name);

    double findHfov(const ros::NodeHandle& nh, const std::string& frame_id);
    double findVfov(const ros::NodeHandle& nh, const std::string& frame_id);

   private:
    /**
     * @brief Function that finds in a dictionary a parameter of type T with the frame id
     * as key. If the parameter is not found there, external files (yaml files, launchers)
     * are searched and, when found, the parameter is inserted to the dictionary
     * @param nh [ros::NodeHandle const&] The NodeHandle of the node using this function
     * @param dict [std::map<std::string, T>*] Pointer to the dictionary to be searched and
     * possibly altered
     * @param key [const std::string&] The key used to search in the dictionary
     * @return [T] The parameter that is found
     **/
    template <class T>
    T
    findParam(const ros::NodeHandle& nh, std::map<std::string, T>* dict, const std::string& key);

   protected:
    /// A dictionary that includes every parent frame id that the node uses
    /// with frame id as key
    std::map<std::string, std::string> parentFrameDict_;

    /// A dictionary that includes Horizontal Fields Of View for every camera
    /// with frame id as key
    std::map<std::string, double> hfovDict_;

    /// A dictionary that includes Vertical Fields Of View for every camera
    /// with frame id as key
    std::map<std::string, double> vfovDict_;
  };

  pandora_common_msgs::GeneralAlert
  GeneralAlertConverter::
  getGeneralAlert(const ros::NodeHandle& nh, const POIStamped& result,
                  double frameWidth, double frameHeight)
  {
    pandora_common_msgs::GeneralAlert alert;
    double hfov = findHfov(nh, result.header.frame_id);
    double vfov = findVfov(nh, result.header.frame_id);
    alert.header = result.header;
    alert.header.frame_id = findParentFrameId(nh, result.header.frame_id,
                                              "/robot_description");
    double x = frameWidth / 2 - result.point.x;
    double y = result.point.y - frameHeight / 2;

    alert.info.yaw = atan(2 * x / frameWidth * tan(hfov * CV_PI / 360.0f));
    alert.info.pitch = atan(2 * y / frameHeight * tan(vfov * CV_PI / 360.0f));
    alert.info.probability = result.probability;

    return alert;
  }

  pandora_common_msgs::GeneralAlertVector
  GeneralAlertConverter::
  getGeneralAlertVector(const ros::NodeHandle& nh, const POIsStamped& result)
  {
    pandora_common_msgs::GeneralAlertVector generalAlertInfos;

    std::string parentFrameId = findParentFrameId(nh, result.header.frame_id, "/robot_description");

    generalAlertInfos.header = result.header;
    generalAlertInfos.header.frame_id = parentFrameId;

    for (int i = 0; i < result.pois.size(); ++i) {
      POIStamped poiStamped;
      poiStamped.point = result.pois[i]->point;
      poiStamped.probability = result.pois[i]->probability;
      poiStamped.header = result.header;
      pandora_common_msgs::GeneralAlert alert;
      alert = getGeneralAlert(nh, poiStamped, result.frameWidth, result.frameHeight);
      generalAlertInfos.alerts.push_back(alert.info);
    }

    return generalAlertInfos;
  }

  POI
  GeneralAlertConverter::
  getPOI(const ros::NodeHandle& nh, const pandora_common_msgs::GeneralAlert& result,
         double frameWidth, double frameHeight)
  {
    POI poi;
    std::string child_frame_id;
    std::map<std::string, std::string>::const_iterator iter;
    for (iter = parentFrameDict_.begin(); iter == parentFrameDict_.end(); ++iter) {
      if (iter->second == result.header.frame_id)
      {
        child_frame_id = iter->first;
      }
    }
    double hfov = findHfov(nh, child_frame_id);
    double vfov = findVfov(nh, child_frame_id);

    double x = tan(result.info.yaw) * (frameWidth / 2) / tan(hfov * CV_PI / 360.0f);
    double y = tan(result.info.pitch) * (frameHeight / 2) / tan(vfov * CV_PI / 360.0f);

    poi.point.x = frameWidth / 2 - x;
    poi.point.y = frameHeight / 2 + y;

    if (poi.point.x < 0)
      poi.point.x = 0;
    if (poi.point.x > static_cast<int>(frameWidth) - 1)
      poi.point.x = static_cast<int>(frameWidth) - 1;
    if (poi.point.y < 0)
      poi.point.y = 0;
    if (poi.point.y > static_cast<int>(frameHeight) - 1)
      poi.point.y = static_cast<int>(frameHeight) - 1;

    poi.probability = result.info.probability;

    return poi;
  }

  double
  GeneralAlertConverter::
  findHfov(const ros::NodeHandle& nh, const std::string& frame_id)
  {
    return findParam<double>(nh, &hfovDict_, frame_id + "/hfov");
  }

  double
  GeneralAlertConverter::
  findVfov(const ros::NodeHandle& nh, const std::string& frame_id)
  {
    return findParam<double>(nh, &vfovDict_, frame_id + "/vfov");
  }

  template <class T>
  T
  GeneralAlertConverter::
  findParam(const ros::NodeHandle& nh, std::map<std::string, T>* dict, const std::string& key)
  {
    typename std::map<std::string, T>::iterator iter;
    if ((iter = dict->find(key)) != dict->end())
    {
      return iter->second;
    }
    else
    {
      std::string true_key;
      if (key[0] == '/')
      {
        true_key = key;
      }
      else
      {
        true_key = '/' + key;
      }

      T param;

      if (!nh.getParam(true_key, param))
      {
        throw vision_config_error(key + " : not found");
      }
      dict->insert(std::make_pair(key, param));
      return param;
    }
  }

  std::string
  GeneralAlertConverter::
  findParentFrameId(const ros::NodeHandle& nh,
      const std::string& key,
      const std::string& model_param_name)
  {
    std::map<std::string, std::string>::iterator iter;
    if ((iter = parentFrameDict_.find(key)) != parentFrameDict_.end())
    {
      return iter->second;
    }
    else
    {
      std::string true_key;
      if (key[0] == '/')
      {
        true_key = key;
        true_key.erase(0, 1);
      }
      else
      {
        true_key = key;
      }

      std::string robot_description;

      if (!nh.getParam(model_param_name, robot_description))
      {
        throw vision_config_error(model_param_name + " : not found");
      }

      std::string parent_frame_id;

      boost::shared_ptr<urdf::ModelInterface> model(
        urdf::parseURDF(robot_description));
      // Get current link and its parent
      boost::shared_ptr<const urdf::Link> currentLink(model->getLink(true_key));
      boost::shared_ptr<const urdf::Link> parentLink(currentLink->getParent());
      // Set the parent frame_id to the parent of the frame_id
      parent_frame_id = parentLink->name;
      parentFrameDict_.insert(std::make_pair(key, parent_frame_id));
      return parent_frame_id;
    }
  }
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_GENERAL_ALERT_CONVERTER_H
