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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_BASE_OBJECT_H
#define PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_BASE_OBJECT_H

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "pandora_data_fusion_msgs/GetGeotiff.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class BaseObject
    * @brief Base class for object type objects
    */
  class BaseObject
  {
   public:
    //!< Type definitions
    typedef boost::shared_ptr<BaseObject> Ptr;
    typedef boost::shared_ptr<BaseObject const> ConstPtr;

   public:
    virtual void update(const ConstPtr& measurement) = 0;
    virtual bool isSameObject(const ConstPtr& object) const = 0;
    virtual geometry_msgs::PoseStamped getPoseStamped() const = 0;
    virtual geometry_msgs::PoseStamped getTfInfo() const = 0;
    virtual void fillGeotiff(
        const pandora_data_fusion_msgs::GetGeotiffResponsePtr& res) const = 0;
    virtual void getVisualization(visualization_msgs::
        MarkerArray* markers) const = 0;
    virtual std::string getType() const = 0;
    virtual int getId() const = 0;
    virtual bool getLegit() const = 0;
    virtual float getProbability() const = 0;
    virtual geometry_msgs::Pose getPose() const = 0;
    virtual std::string getFrameId() const = 0;
    virtual void setId(int id) = 0;
    virtual void setLegit(bool legit) = 0;
    virtual void setProbability(float probability) = 0;
    virtual void setPose(const geometry_msgs::Pose& pose) = 0;
  };

  typedef BaseObject::Ptr ObjectPtr;
  typedef BaseObject::ConstPtr ObjectConstPtr;

  typedef std::vector<ObjectPtr> ObjectPtrVector;
  typedef boost::shared_ptr<ObjectPtrVector> ObjectPtrVectorPtr;
  typedef std::vector<ObjectPtrVector> ObjectPtrVectorVector;

  typedef std::vector<ObjectConstPtr> ObjectConstPtrVector;
  typedef boost::shared_ptr<ObjectConstPtrVector> ObjectConstPtrVectorPtr;
  typedef std::vector<ObjectConstPtrVector> ObjectConstPtrVectorVector;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECTS_OBJECT_INTERFACE_BASE_OBJECT_H
