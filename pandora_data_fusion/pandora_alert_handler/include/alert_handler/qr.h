// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_QR_H
#define ALERT_HANDLER_QR_H

#include "alert_handler/kalman_object.h"

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
        typedef boost::shared_ptr<Qr> Ptr;
        typedef boost::shared_ptr<Qr const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<Qr> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Qr> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Qr();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual PoseStamped getPoseStamped() const;

        virtual void fillGeotiff(pandora_data_fusion_msgs::
            DatafusionGeotiffSrv::Response* res) const;

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;

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
        void setContent(std::string content)
        {
          content_ = content;
        }

        /**
         * @brief Getter for member timeFound_
         * @return ros::Time The QR's timeFound
         */
        ros::Time getTimeFound() const
        {
          return timeFound_;
        }

        /**
         * @brief Setter for member timeFound_
         * @return void
         */
        void setTimeFound(ros::Time timeFound)
        {
          timeFound_ = timeFound;
        }

      private:

        //!< The qr's content
        std::string content_;
        //!< The time when this qr was first found
        ros::Time timeFound_;
    };

    typedef Qr::Ptr QrPtr;
    typedef Qr::ConstPtr QrConstPtr;
    typedef Qr::PtrVector QrPtrVector;
    typedef Qr::PtrVectorPtr QrPtrVectorPtr;
    typedef Qr::ListPtr QrListPtr;
    typedef Qr::ListConstPtr QrListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_QR_H
