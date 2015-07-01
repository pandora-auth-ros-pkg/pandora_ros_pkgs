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

#ifndef PANDORA_ALERT_HANDLER_OBJECTS_QR_H
#define PANDORA_ALERT_HANDLER_OBJECTS_QR_H

#include <vector>
#include <string>

#include "pandora_vision_msgs/QRAlertVector.h"
#include "pandora_vision_msgs/QRAlert.h"
#include "pandora_data_fusion_msgs/QrInfo.h"

#include "pandora_alert_handler/objects/object_interface/kalman_object.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class Qr
    * @brief Concrete class representing a Qr Object. Inherits from Object
    */
  class Qr : public KalmanObject<Qr>
  {
   public:
    //!< Type Definitions
    typedef pandora_vision_msgs::QRAlertVector AlertVector;
    typedef pandora_vision_msgs::QRAlert Alert;
    typedef pandora_data_fusion_msgs::QrInfo Info;

   public:
    static void setUpObject(const Ptr& ptr, const Alert& msg)
    {
      ptr->setContent(msg.QRcontent);
    }

    /**
      * @brief Constructor
      */
    Qr();

    virtual bool isSameObject(const ObjectConstPtr& object) const;
    virtual void getVisualization(visualization_msgs::
        MarkerArray* markers) const;
    virtual void fillGeotiff(const pandora_data_fusion_msgs::
        GetGeotiffResponsePtr& res) const;

    pandora_data_fusion_msgs::QrInfo getQrInfo() const;

    /**
      * @brief Getter for member content_
      * @return std::string The QR's content
      */
    std::string getContent() const
    {
      return content_;
    }

    /**
      * @brief Setter for member content_
      * @return void
      */
    void setContent(const std::string& content)
    {
      content_ = content;
    }

   private:
    //!< The qr's content
    std::string content_;
  };

  typedef Qr::Ptr QrPtr;
  typedef Qr::ConstPtr QrConstPtr;
  typedef Qr::PtrVector QrPtrVector;
  typedef Qr::PtrVectorPtr QrPtrVectorPtr;
  typedef Qr::List QrList;
  typedef Qr::ListPtr QrListPtr;
  typedef Qr::ListConstPtr QrListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECTS_QR_H
