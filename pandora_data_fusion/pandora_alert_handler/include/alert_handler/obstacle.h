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

#ifndef PANDORA_ALERT_HANDLER_OBSTACLE_H
#define PANDORA_ALERT_HANDLER_OBSTACLE_H

#include <vector>
#include <string>

#include "pandora_vision_msgs/ObstacleAlert.h"
#include "pandora_vision_msgs/ObstacleAlertVector.h"

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class Obstacle
    * @brief: Concrete class representing an obstacle in the world. Inherits
    * from KalmanObject
    */
  class Obstacle : public KalmanObject<Obstacle>
  {
   public:
    //!< Type Definitions
    typedef pandora_vision_msgs::ObstacleAlert Alert;
    typedef pandora_vision_msgs::ObstacleAlertVector AlertVector;

   public:
    Obstacle ();
    virtual ~Obstacle ();

    /* public methods */
    virtual bool isSameObject(const ObjectConstPtr& object) const;
    virtual void update(const ObjectConstPtr& measurement);

    uint8_t getObstacleType() const;
    void setObstacleType(uint8_t obstacleType);

    double getLength() const;
    void setLength(double length);

    double getWidth() const;
    void setWidth(double width);

   protected:
    /* data */
    uint8_t obstacleType_;
    double length_;
    double width_;
  };

  typedef Obstacle::Ptr ObstaclePtr;
  typedef Obstacle::ConstPtr ObstacleConstPtr;
  typedef Obstacle::PtrVector ObstaclePtrVector;
  typedef Obstacle::PtrVectorPtr ObstaclePtrVectorPtr;
  typedef Obstacle::List ObstacleList;
  typedef Obstacle::ListPtr ObstacleListPtr;
  typedef Obstacle::ListConstPtr ObstacleListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBSTACLE_H
