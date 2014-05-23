// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_THERMAL_H
#define ALERT_HANDLER_THERMAL_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Thermal
     * @brief Concrete class representing a Thermal Object. Inherits from Object
     */ 
    class Thermal : public KalmanObject<Thermal>
    {
      public:

        typedef boost::shared_ptr<Thermal> Ptr;
        typedef boost::shared_ptr<Thermal const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<Thermal> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Thermal> > ListConstPtr;

        /**
         * @brief Constructor
         */
        Thermal();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

    };

    typedef Thermal::Ptr ThermalPtr;
    typedef Thermal::ConstPtr ThermalConstPtr;
    typedef Thermal::PtrVector ThermalPtrVector;
    typedef Thermal::PtrVectorPtr ThermalPtrVectorPtr;
    typedef Thermal::ListPtr ThermalListPtr;
    typedef Thermal::ListConstPtr ThermalListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_THERMAL_H
