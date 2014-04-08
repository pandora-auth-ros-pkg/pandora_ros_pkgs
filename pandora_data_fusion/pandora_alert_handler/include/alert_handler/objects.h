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
* Authors: 
*   Christos Zalidis <zalidis@gmail.com>
*   Triantafyllos Afouras <afourast@gmail.com>
*********************************************************************/

#ifndef ALERT_HANDLER_OBJECTS_H
#define ALERT_HANDLER_OBJECTS_H

#include <set>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "data_fusion_communications/DatafusionGeotiffSrv.h"
#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

/**
@class Object
@brief Abstract class representing an Object in 3d space
**/ 
class Object
{
 public:

  typedef boost::shared_ptr<Object> Ptr;
  typedef boost::shared_ptr<Object const> ConstPtr;

  /**
  @brief Constructor
  **/
  Object();

  /**
  @brief Returns if this Object is the same with the given one
  @param object [const Ptr&] The object to compare with
  @param distance [float] The minimum distance between two objects so that they 
    be regarded the same
  @return bool
  **/
  virtual bool isSameObject(const ConstPtr& object, float distance) const;

  /**
  @brief Returns the object's pose
  @return geometry_msgs::PoseStamped The object's pose
  **/
  virtual geometry_msgs::PoseStamped getPoseStamped() const;

  /**
  @brief Fill in the geotiff info with the object's details 
  @param res 
    [data_fusion_communications::DatafusionGeotiffSrv::Response*] 
      The output service response param
  @return void
  **/
  virtual void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const {}

  /**
  @brief Get the object's visualization  
  @param markers [visualization_msgs::MarkerArray*] A marker array to be filled
    with markers containing the object's visualization
  @return void
  **/
  virtual void getVisualization(
    visualization_msgs::MarkerArray* markers) const {};
  
  /**
  @brief Getter for member id_
  @return int id
  **/
  int getId() const
  {
    return id_;
  }
  
  /**
  @brief Getter for member counter_
  @return int counter
  **/
  int getCounter() const
  {
    return counter_;
  }
  
  /**
  @brief Getter for member legit_
  @return bool legit
  **/
  bool getLegit() const
  {
    return legit_;
  }
  
  /**
  @brief Getter for member type_
  @return std::string type
  **/
  std::string getType() const
  {
    return type_;
  }
  
  /**
  @brief Getter for member legit_
  @return bool legit
  **/
  float getProbability() const
  {
    return probability_;
  }
  
  /**
  @brief Getter for member pose_
  @return geometry_msgs::Pose& The object's pose
  **/
  const geometry_msgs::Pose& getPose() const
  {
    return pose_;
  }
  
  /**
  @brief Getter for member frame_id
  @return geometry_msgs::Pose& The object's frame_id
  **/
  std::string getFrameId() const
  {
    return frame_id_;
  }
  
  /**
  @brief Setter for member id_
  @param id [int] The new id value
  @return void
  **/
  void setId(int id)
  {
    id_ = id;
  }

  /**
  @brief Setter for member counter_
  @param counter [int] The new counter value
  @return void
  **/
  void setCounter(int counter)
  {
    counter_ = counter;
  }

  /**
  @brief Setter for member legit_
  @param legit [int] The new legit value
  @return void
  **/
  void setLegit(bool legit)
  {
    legit_ = legit;
  }
  
  /**
  @brief Setter for member type_
  @param type [std::string] The new type value
  @return void
  **/
  void setType(std::string type)
  {
    type_ = type;
  }
  
  /**
  @brief Setter for member probability_
  @param probability [float] The new probability value
  @return void
  **/
  void setProbability(float probability)
  {
    probability_ = probability;
  }
  
  /**
  @brief Setter for member pose_
  @param pose [const geometry_msgs::Pose&] The new pose value
  @return void
  **/
  void setPose(const geometry_msgs::Pose& pose)
  {
    pose_ = pose;
  }
  
  /**
  @brief Increments counter by 1
  @return void
  **/
  void incrementCounter()
  {
    counter_++;
  }
  
 protected:
 
  //!< The object's id
  int id_;
  //!< Counter for counting how many times this object is found
  int counter_;
  //!< True if we have confidence that this object is eligible for use
  bool legit_;
  //!< A string indicating the type of object
  std::string type_;
  //!< The Objects's probability
  float probability_;

  //!< The object's pose in 3d space
  geometry_msgs::Pose pose_;
  //!< The reference frame for the pose. Should normally be "/world"
  std::string frame_id_;

};

typedef Object::Ptr ObjectPtr;
typedef Object::ConstPtr ObjectConstPtr;

typedef std::vector<ObjectPtr> ObjectPtrVector;
typedef boost::shared_ptr<ObjectPtrVector> ObjectPtrVectorPtr;
typedef std::vector<ObjectPtrVector> ObjectPtrVectorVector;

typedef std::vector<ObjectConstPtr> ObjectConstPtrVector;
typedef boost::shared_ptr<ObjectConstPtrVector> ObjectConstPtrVectorPtr;
typedef std::vector<ObjectConstPtrVector> ObjectConstPtrVectorVector;

/**
  @class Qr
  @brief Concrete class representing a Qr Object. Inherits from Object
**/ 
class Qr : public Object
{
 public:
  typedef boost::shared_ptr<Qr> Ptr;
  typedef boost::shared_ptr<Qr const> ConstPtr;

  /**
  @brief Constructor
  **/
  Qr();

  virtual bool isSameObject(const ObjectConstPtr& object, float distance) const;

  virtual geometry_msgs::PoseStamped getPoseStamped() const;

  virtual void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;
  
  virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
 
  /**
  @brief Getter for member content_
  @return std::string The QR's content
  **/
  std::string getContent() const
  {
    return content_;
  }
 
  /**
  @brief Setter for member content_
  @return void
  **/
  void setContent(std::string content)
  {
    content_ = content;
  }
 
 protected:
  //!< The qr's content
  std::string content_;
  //!< The time when this qr was first found
  ros::Time timeFound_;
};

typedef Qr::Ptr QrPtr;
typedef Qr::ConstPtr QrConstPtr;
typedef std::vector< QrPtr > QrPtrVector;
typedef boost::shared_ptr< QrPtrVector > QrPtrVectorPtr;

/**
  @class Hazmat
  @brief Concrete class representing a Hazmat Object. Inherits from Object
**/ 
class Hazmat : public Object
{
 public:
  typedef boost::shared_ptr<Hazmat> Ptr;
  typedef boost::shared_ptr<Hazmat const> ConstPtr;

  /**
  @brief Constructor
  **/
  Hazmat();

  virtual bool isSameObject(const ObjectConstPtr& object, float distance) const;

  virtual geometry_msgs::PoseStamped getPoseStamped() const;

  virtual void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;
  
  virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
  
  /**
  @brief Getter for member pattern_
  @return int The Hazmat's pattern
  **/
  int getPattern() const
  {
    return pattern_;
  }
  
  /**
  @brief Setter for member pattern_
  @return void
  **/
  void setPattern(int pattern)
  {
    pattern_ = pattern;
  }
  
 protected:
  //!< The hazmat's pattern
  int pattern_;
};

typedef Hazmat::Ptr HazmatPtr;
typedef Hazmat::ConstPtr HazmatConstPtr;
typedef std::vector< HazmatPtr > HazmatPtrVector;
typedef boost::shared_ptr< HazmatPtrVector > HazmatPtrVectorPtr;

/**
  @class Hole
  @brief Concrete class representing a Hole Object. Inherits from Object
**/ 
class Hole : public Object
{
 public:
  typedef boost::shared_ptr<Hole> Ptr;
  typedef boost::shared_ptr<Hole const> ConstPtr;

  /**
  @brief Constructor
  **/
  Hole();

  virtual bool isSameObject(const ObjectConstPtr& object, float distance) const;

  virtual geometry_msgs::PoseStamped getPoseStamped() const;

  virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

  /**
  @brief Getter for member holeId_
  @return int The holeId 
  **/
  unsigned int getHoleId() const 
  {
    return holeId_;
  }
  
  /**
  @brief Setter for member holeId_
  @return void
  **/
  void setHoleId(int holeId) 
  {
    holeId_ = holeId;
  }

 protected:
  //!< The hole's holeId. Caution: Not to be confused with the Object id!
  unsigned int holeId_;
};

typedef Hole::Ptr HolePtr;
typedef Hole::ConstPtr HoleConstPtr;
typedef std::vector< HolePtr > HolePtrVector;
typedef boost::shared_ptr< HolePtrVector > HolePtrVectorPtr;

/**
  @class Tpa
  @brief Concrete class representing a Tpa Object. Inherits from Object
**/ 
class Tpa : public Object
{
 public:
  typedef boost::shared_ptr<Tpa> Ptr;
  typedef boost::shared_ptr<Tpa const> ConstPtr;

  /**
  @brief Constructor
  **/
  Tpa();

  virtual bool isSameObject(const ObjectConstPtr& object, float distance) const;

  virtual geometry_msgs::PoseStamped getPoseStamped() const;

  virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
};

typedef Tpa::Ptr TpaPtr;
typedef Tpa::ConstPtr TpaConstPtr;
typedef std::vector< TpaPtr > TpaPtrVector;
typedef boost::shared_ptr< TpaPtrVector > TpaPtrVectorPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECTS_H
