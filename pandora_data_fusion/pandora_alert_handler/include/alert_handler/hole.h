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

#ifndef ALERT_HANDLER_HOLE_H
#define ALERT_HANDLER_HOLE_H

#include <vector>

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Hole
     * @brief Concrete class representing a Hole Object. Inherits from Object
     */
    class Hole : public KalmanObject<Hole>
    {
      public:
        //!< Type Definitions
        typedef boost::shared_ptr<Hole> Ptr;
        typedef boost::shared_ptr<Hole const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef ObjectList<Hole> List;
        typedef boost::shared_ptr<List> ListPtr;
        typedef boost::shared_ptr< const ObjectList<Hole> > ListConstPtr;

      public:
        /**
         * @brief Constructor
         */
        Hole();

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;

        /**
         * @brief Getter for member holeId_
         * @return int The holeId
         */
        unsigned int getHoleId() const
        {
          return holeId_;
        }

        /**
         * @brief Setter for member holeId_
         * @return void
         */
        void setHoleId(int holeId)
        {
          holeId_ = holeId;
        }

      private:
        //!< The hole's holeId. Caution: Not to be confused with the Object id!
        unsigned int holeId_;

    };

    typedef Hole::Ptr HolePtr;
    typedef Hole::ConstPtr HoleConstPtr;
    typedef Hole::PtrVector HolePtrVector;
    typedef Hole::PtrVectorPtr HolePtrVectorPtr;
    typedef Hole::List HoleList;
    typedef Hole::ListPtr HoleListPtr;
    typedef Hole::ListConstPtr HoleListConstPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_HOLE_H
