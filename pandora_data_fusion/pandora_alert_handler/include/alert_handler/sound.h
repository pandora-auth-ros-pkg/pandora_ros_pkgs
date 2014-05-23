// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_SOUND_H
#define ALERT_HANDLER_SOUND_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Sound
     * @brief Concrete class representing a Sound Object. Inherits from Object
     */ 
    class Sound : public KalmanObject<Sound>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Sound> Ptr;
        typedef boost::shared_ptr<Sound const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<Sound> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Sound> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Sound();

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;
    };

    typedef Sound::Ptr SoundPtr;
    typedef Sound::ConstPtr SoundConstPtr;
    typedef Sound::PtrVector SoundPtrVector;
    typedef Sound::PtrVectorPtr SoundPtrVectorPtr;
    typedef Sound::ListPtr SoundListPtr;
    typedef Sound::ListConstPtr SoundListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_SOUND_H
