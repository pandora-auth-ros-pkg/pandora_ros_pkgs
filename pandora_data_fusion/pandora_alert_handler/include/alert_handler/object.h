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

#ifndef ALERT_HANDLER_OBJECT_H
#define ALERT_HANDLER_OBJECT_H

#include <vector>
#include <string>

#include "alert_handler/base_object.h"
#include "alert_handler/defines.h"
#include "alert_handler/object_list.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Object
     * @brief Abstract class representing an Object in 3D space
     */
    template <class DerivedObject> class Object : public BaseObject
    {
      public:
        //!< Type Definitions
        typedef boost::shared_ptr<DerivedObject> Ptr;
        typedef boost::shared_ptr<DerivedObject const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef boost::shared_ptr< ObjectList<DerivedObject> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<DerivedObject> > ListConstPtr;

        /**
         * @brief Setter for member type_
         * @param type [std::string] The new type value
         * @return void
         */
        static void setObjectType(std::string type)
        {
          type_ = type;
        }

        /**
         * @brief Getter for static object type's score.
         * @return int score
         */
        static int getObjectScore()
        {
          return objectScore_;
        }

        /**
         * @brief Getter for static object type's probability threshold.
         * @return float probability
         */
        static float getProbabilityThres()
        {
          return probabilityThres_;
        }

        /**
         * @brief Getter for static distance threshold.
         * @return float distanceThres
         */
        static float getDistanceThres()
        {
          return distanceThres_;
        }

        /**
         * @brief Getter for static orientation difference threshold
         * @return float orientDiff
         */
        static float getOrientDiff()
        {
          return orientDiff_;
        }

        /**
         * @brief Getter for static merge threshold.
         * @return float mergeDistance
         */
        static float getMergeDistance()
        {
          return mergeDistance_;
        }

        /**
         * @brief Method for accessing DerivedObject's associated list.
         * @return ListPtr the list
         */
        static ListPtr getList()
        {
          return listPtr_;
        }

        /**
         * @brief Getter for member type_
         * @return std::string type
         */
        static std::string getObjectType()
        {
          return type_;
        }

        /**
         * @brief Setter for static distance threshold.
         * @param distanceThres [float] distance
         * @return void
         */
        static void setDistanceThres(float distanceThres)
        {
          distanceThres_ = distanceThres;
        }

        /**
         * @brief Setter for static orientation difference threshold
         * @param orientDiff [float] difference threshold in degrees
         * @return float orientDiff
         */
        static void setOrientDiff(float orientDiff)
        {
          orientDiff_ = orientDiff * DEGREE;
        }

        /**
         * @brief Setter for static merge threshold.
         * @param mergeDistance [float] distance
         * @return void
         */
        static void setMergeDistance(float mergeDistance)
        {
          mergeDistance_ = mergeDistance;
        }

        /**
         * @brief Setter for static object type's score.
         * @param objectScore [int] score
         * @return void
         */
        static void setObjectScore(int objectScore)
        {
          objectScore_ = objectScore;
        }

        /**
         * @brief Setter for static object type's probability threshold.
         * @param probabilityThres [float] probability
         * @return void
         */
        static void setProbabilityThres(float probabilityThres)
        {
          probabilityThres_ = probabilityThres;
        }

        /**
         * @brief Setter for static pointer to the list that will be filled
         * by DerivedObjects.
         * @param listPtr [ListPtr] pointer to list
         * @return void
         */
        static void setList(ListPtr listPtr)
        {
          listPtr_ = listPtr;
        }

      public:
        static bool is3D;
        static bool isVictimAlert;

      protected:
        //!< Variable with objects' min distance.
        static float distanceThres_;
        //!< Variable with object's maximum orientation difference (in yaw)
        static float orientDiff_;
        //!< Variable with objects' merge distance.
        static float mergeDistance_;
        //!< Variable containing object type's score.
        static int objectScore_;
        //!< Variable containing object type's probability threshold for an
        //!< object to become legitimate.
        static float probabilityThres_;
        //!< A string indicating the type of object
        static std::string type_;
        //!< Pointer to list that contains Objects with type DerivedObject;
        static ListPtr listPtr_;

      public:
        /**
         * @brief Constructor
         */
        Object();

        /**
         * @brief Returns if this Object is the same with the given one
         * @param object [ConstPtr const&] The object to compare with
         * @param distance [float] The minimum distance between two objects so
         * that they be regarded the same
         * @return bool
         */
        virtual bool isSameObject(const ObjectConstPtr& object) const;

        /**
         * @brief Returns the object's pose
         * @return geometry_msgs::PoseStamped The object's pose
         */
        virtual geometry_msgs::PoseStamped getPoseStamped() const;

        /**
         * @brief Updates this object with the measurement.
         * @return void
         */
        virtual void update(const ObjectConstPtr& object);

        /**
         * @brief Check if this object with its current probability is legit.
         * @return void
         */
        void checkLegit();

        /**
         * @brief Getter for member id_
         * @return int id
         */
        int getId() const
        {
          return id_;
        }

        /**
         * @brief Getter for member legit_
         * @return bool legit
         */
        bool getLegit() const
        {
          return legit_;
        }

        std::string getType() const
        {
          return type_;
        }

        /**
         * @brief Getter for member legit_
         * @return bool legit
         */
        float getProbability() const
        {
          return probability_;
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
         * @brief Getter for member pose_
         * @return geometry_msgs::Pose& The object's pose
         */
        geometry_msgs::Pose getPose() const
        {
          return pose_;
        }

        /**
         * @brief Getter for member frame_id
         * @return geometry_msgs::Pose& The object's frame_id
         */
        std::string getFrameId() const
        {
          return frame_id_;
        }

        /**
         * @brief Setter for member id_
         * @param id [int] The new id value
         * @return void
         */
        void setId(int id)
        {
          id_ = id;
          frame_id_ = type_ + "_" + boost::to_string(id_);
        }

        /**
         * @brief Setter for member legit_
         * @param legit [int] The new legit value
         * @return void
         */
        void setLegit(bool legit)
        {
          legit_ = legit;
        }

        /**
         * @brief Setter for member probability_
         * @param probability [float] The new probability value
         * @return void
         */
        void setProbability(float probability)
        {
          probability_ = probability;
        }

        /**
         * @brief Setter for member timeFound_
         * @return void
         */
        void setTimeFound(const ros::Time& timeFound)
        {
          timeFound_ = timeFound;
        }

        /**
         * @brief Setter for member pose_
         * @param pose [const geometry_msgs::Pose&] The new pose value
         * @return void
         */
        void setPose(const geometry_msgs::Pose& pose)
        {
          pose_ = pose;
        }

      protected:
        //!< The object's id
        int id_;
        //!< The reference frame for object.
        std::string frame_id_;
        //!< True if we have confidence that this object is eligible for use
        bool legit_;
        //!< The Objects's probability
        float probability_;
        //!< The time when this object was first found
        ros::Time timeFound_;

        //!< The object's pose in 3d space
        geometry_msgs::Pose pose_;

      private:
        friend class ObjectListTest;
    };

    template <class DerivedObject>
      Object<DerivedObject>::Object()
      {
        legit_ = false;
      }

    template <class DerivedObject>
      bool Object<DerivedObject>::is3D = true;
    template <class DerivedObject>
      bool Object<DerivedObject>::isVictimAlert = true;
    template <class DerivedObject>
      float Object<DerivedObject>::distanceThres_ = 0.5;
    template <class DerivedObject>
      float Object<DerivedObject>::orientDiff_ = 45.0;
    template <class DerivedObject>
      float Object<DerivedObject>::mergeDistance_ = 0.03;
    template <class DerivedObject>
      float Object<DerivedObject>::probabilityThres_ = 0;
    template <class DerivedObject>
      int Object<DerivedObject>::objectScore_ = 0;
    template <class DerivedObject>
      std::string Object<DerivedObject>::type_ = "object";
    template <class DerivedObject>
      typename Object<DerivedObject>::ListPtr Object<DerivedObject>::listPtr_;

    template <class DerivedObject>
      geometry_msgs::PoseStamped Object<DerivedObject>::getPoseStamped() const
      {
        geometry_msgs::PoseStamped objPoseStamped;
        objPoseStamped.pose = pose_;
        objPoseStamped.header.frame_id = frame_id_;
        objPoseStamped.header.stamp = ros::Time::now();
        return objPoseStamped;
      }

    template <class DerivedObject>
      bool Object<DerivedObject>::isSameObject(const ObjectConstPtr& object) const
      {
        if (type_ != object->getType())
          return false;
        bool isSame = Utils::arePointsInRange(
            pose_.position, object->getPose().position,
            DerivedObject::is3D, distanceThres_);
        if (!isSame) return false;
        isSame = Utils::isOrientationClose(pose_.orientation,
            object->getPose().orientation, orientDiff_);
        return isSame;
      }

    template <class DerivedObject>
      void Object<DerivedObject>::update(const ObjectConstPtr& measurement)
      {
        (*this) = *boost::dynamic_pointer_cast<const DerivedObject>(measurement);
      }

    template <class DerivedObject>
      void Object<DerivedObject>::checkLegit()
      {
        if (probability_ >= probabilityThres_)
        {
          legit_ = true;
        }
      }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_H
