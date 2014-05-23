// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_MOTION_H
#define ALERT_HANDLER_MOTION_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Motion
     * @brief Concrete class representing a Motion Object. Inherits from Object
     */ 
    class Motion : public KalmanObject<Motion>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Motion> Ptr;
        typedef boost::shared_ptr<Motion const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<Motion> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Motion> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Motion();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
    };

    typedef Motion::Ptr MotionPtr;
    typedef Motion::ConstPtr MotionConstPtr;
    typedef Motion::PtrVector MotionPtrVector;
    typedef Motion::PtrVectorPtr MotionPtrVectorPtr;
    typedef Motion::ListPtr MotionListPtr;
    typedef Motion::ListConstPtr MotionListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_MOTION_H
