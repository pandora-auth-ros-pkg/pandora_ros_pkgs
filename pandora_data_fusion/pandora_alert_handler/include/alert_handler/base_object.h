// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_BASE_OBJECT_H
#define ALERT_HANDLER_BASE_OBJECT_H

#include <set>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "pandora_data_fusion_msgs/DatafusionGeotiffSrv.h"
#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/utils.h"

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
        virtual PoseStamped getPoseStamped() const = 0;
        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const = 0;
        virtual std::string getType() const = 0;
        virtual int getId() const = 0;
        virtual bool getLegit() const = 0;
        virtual float getProbability() const = 0;
        virtual const Pose& getPose() const = 0;
        virtual std::string getFrameId() const = 0;
        virtual void setId(int id) = 0;
        virtual void setLegit(bool legit) = 0;
        virtual void setProbability(float probability) = 0;
        virtual void setPose(const Pose& pose) = 0;

      protected:

        //!< The reference frame for the pose. Should normally be "/world"
        static std::string frame_id_;

      private:

        friend class ObjectListTest;
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

#endif  // ALERT_HANDLER_BASE_OBJECT_H
