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

#include "pandora_alert_handler/objects/obstacle.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  Obstacle::Obstacle() {}
  Obstacle::~Obstacle() {}

  bool Obstacle::isSameObject(const ObjectConstPtr& object) const
  {
    bool isSame = Object<Obstacle>::isSameObject(object);
    if (!isSame) return false;
    return (obstacleType_ == boost::dynamic_pointer_cast<Obstacle const>(
        object)->getObstacleType());
  }

  void Obstacle::update(const ObjectConstPtr& measurement)
  {
    KalmanObject<Obstacle>::update(measurement);

    ObstacleConstPtr obstacleMeas = boost::dynamic_pointer_cast<Obstacle const>(measurement);
    geometry_msgs::Pose newPose = this->getPose();
    newPose.orientation = obstacleMeas->getPose().orientation;
    this->setPose(newPose);
    double newLength = obstacleMeas->getLength();
    if (newLength > length_)
      length_ = newLength;
    double newWidth = obstacleMeas->getWidth();
    if (newWidth > width_)
      width_ = newWidth;
  }

  void Obstacle::fillGeotiff(const pandora_data_fusion_msgs::
      GetGeotiffResponsePtr& res) const
  {
    res->obstacles.push_back(getObstacleInfo());
  }

  pandora_data_fusion_msgs::ObstacleInfo Obstacle::getObstacleInfo() const
  {
    Info obstacleInfo;

    obstacleInfo.id = getId();
    obstacleInfo.obstacleFrameId = getFrameId();
    obstacleInfo.timeFound = getTimeFound();
    obstacleInfo.obstaclePose = getPoseStamped();
    obstacleInfo.probability = getProbability();

    obstacleInfo.length = getLength();
    obstacleInfo.width = getWidth();
    obstacleInfo.type = getObstacleType();

    return obstacleInfo;
  }

  uint8_t Obstacle::getObstacleType() const
  {
    return obstacleType_;
  }

  void Obstacle::setObstacleType(uint8_t obstacleType)
  {
    obstacleType_ = obstacleType;
  }

  double Obstacle::getLength() const
  {
    return length_;
  }

  void Obstacle::setLength(double length)
  {
    length_ = length;
  }

  double Obstacle::getWidth() const
  {
    return width_;
  }

  void Obstacle::setWidth(double width)
  {
    width_ = width;
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion
