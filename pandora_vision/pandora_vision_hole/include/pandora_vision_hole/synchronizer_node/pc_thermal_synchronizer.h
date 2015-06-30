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
 *   Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_PC_THERMAL_SYNCHRONIZER_H
#define PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_PC_THERMAL_SYNCHRONIZER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "distrib_msgs/flirLeptonMsg.h"
#include "pandora_vision_msgs/SynchronizedMsg.h"

namespace pandora_vision
{
namespace pandora_vision_hole
{
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PcSubscriber;
  typedef boost::shared_ptr<PcSubscriber> PcSubscriberPtr;
  typedef message_filters::Subscriber<distrib_msgs::flirLeptonMsg> ThermalSubscriber;
  typedef boost::shared_ptr<ThermalSubscriber> ThermalSubscriberPtr;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, distrib_msgs::flirLeptonMsg> ApprTimePcThermalPolicy;
  typedef boost::shared_ptr<ApprTimePcThermalPolicy> ApprTimePcThermalPolicyPtr;
  typedef message_filters::Synchronizer<ApprTimePcThermalPolicy> ApprTimePcThermalSynchronizer;
  typedef boost::shared_ptr<ApprTimePcThermalSynchronizer> ApprTimePcThermalSynchronizerPtr;

  /**
   * @class PcThermalSynchronizer TODO
   */
  class PcThermalSynchronizer : public nodelet::Nodelet
  {
   public:
    PcThermalSynchronizer();
    virtual ~PcThermalSynchronizer();

    virtual void onInit();

    /**
     * @brief The synchronized callback called when the input messages from kinect
     * and thermal camera are acquired. A custom synchronized message is filled and
     * sent to synchronizer node for further usage.
     * @param[in] pcMsg [const sensor_msgs::PointCloud2::ConstPtr&]
     * The input pointcloud from kinect
     * @param[in] thermalMsg[const distrib_msgs::flirLeptonMsg::ConstPtr&]
     * The input thermal message
     */
    void synchronizedCallback(
        const sensor_msgs::PointCloud2ConstPtr& pcMsg,
        const distrib_msgs::flirLeptonMsgConstPtr& thermalMsg);

   private:
    /**
     * @brief Acquires topics' names needed to be subscribed to and advertise
     * to by the rgb_depth_thermal_synchronizer node
     */
    void getTopicNames();

   private:
    //!< Node's distinct name
    std::string nodeName_;
    //!< The ROS node handle in general namespace
    ros::NodeHandle nh_;
    //!< The ROS node handle in private namespace
    ros::NodeHandle private_nh_;


    //!< The message_filters subscriber to kinect that aquires the PointCloud2
    //!< message
    PcSubscriberPtr inputPointCloudSubscriberPtr_;
    //!< The name of the topic from where the PointCloud2 message is acquired
    std::string inputPointCloudTopic_;

    //!< The message_filters subscriber to flir-lepton camera that aquires
    //!< the flirLeptonMsg message
    ThermalSubscriberPtr inputThermalCameraSubscriberPtr_;
    //!< The name of the topic from where the flirLeptonMsg message is acquired
    std::string inputThermalCameraTopic_;

    ApprTimePcThermalSynchronizerPtr synchronizerPtr_;

    ros::Publisher synchronizedMsgPublisher_;
    std::string synchronizedMsgTopic_;

    //!< The queue size of the approximate time synch method
    int queue_;
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_PC_THERMAL_SYNCHRONIZER_H
