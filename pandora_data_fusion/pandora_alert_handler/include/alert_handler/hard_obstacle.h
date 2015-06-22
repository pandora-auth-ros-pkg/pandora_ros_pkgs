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
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_ALERT_HANDLER_HARD_OBSTACLE_H
#define PANDORA_ALERT_HANDLER_HARD_OBSTACLE_H

#include <vector>
#include <string>

#include "pandora_vision_msgs/ObstacleAlert.h"
#include "pandora_vision_msgs/ObstacleAlertVector.h"

#include "alert_handler/obstacle.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class HardObstacle
    * @brief: Concrete class representing a hard obstacle (like pikes on the
    * floor or a cliff) in the world. Inherits from Obstacle
    */
  class HardObstacle : public Obstacle
  {
   public:
    //!< Type Definitions
    typedef boost::shared_ptr<HardObstacle> Ptr;
    typedef boost::shared_ptr<HardObstacle const> ConstPtr;
    typedef std::vector<Ptr> PtrVector;
    typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
    typedef ObjectList<HardObstacle> List;
    typedef boost::shared_ptr< List > ListPtr;
    typedef boost::shared_ptr< List const > ListConstPtr;

   public:
    HardObstacle ();
    virtual ~HardObstacle ();

    virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
    virtual std::string setFrameId(int id);
  };

  typedef HardObstacle::Ptr HardObstaclePtr;
  typedef HardObstacle::ConstPtr HardObstacleConstPtr;
  typedef HardObstacle::PtrVector HardObstaclePtrVector;
  typedef HardObstacle::PtrVectorPtr HardObstaclePtrVectorPtr;
  typedef HardObstacle::List HardObstacleList;
  typedef HardObstacle::ListPtr HardObstacleListPtr;
  typedef HardObstacle::ListConstPtr HardObstacleListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_HARD_OBSTACLE_H
