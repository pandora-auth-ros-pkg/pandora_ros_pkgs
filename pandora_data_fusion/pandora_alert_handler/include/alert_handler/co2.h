// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_CO2_H
#define ALERT_HANDLER_CO2_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Co2
     * @brief Concrete class representing a Co2 Object. Inherits from Object
     */ 
    class Co2 : public KalmanObject<Co2>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Co2> Ptr;
        typedef boost::shared_ptr<Co2 const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<Co2> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Co2> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Co2();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
    };

    typedef Co2::Ptr Co2Ptr;
    typedef Co2::ConstPtr Co2ConstPtr;
    typedef Co2::PtrVector Co2PtrVector;
    typedef Co2::PtrVectorPtr Co2PtrVectorPtr;
    typedef Co2::ListPtr Co2ListPtr;
    typedef Co2::ListConstPtr Co2ListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_CO2_H
