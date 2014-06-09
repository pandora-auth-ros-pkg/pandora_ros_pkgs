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

#ifndef ALERT_HANDLER_OBJECT_LIST_H
#define ALERT_HANDLER_OBJECT_LIST_H

#include <list>
#include <vector>
#include <boost/iterator/iterator_adaptor.hpp>

#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/base_object.h"
// #include "alert_handler/const_iterator_const_ref.h"
#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    template <class ObjectType>
      class ObjectList
      {
        public:

          //!< Type Definitions
          typedef boost::shared_ptr< ObjectType > Ptr;
          typedef boost::shared_ptr< ObjectType const > ConstPtr;
          typedef std::list< Ptr > List;
          typedef typename List::iterator iterator;  
          typedef typename List::const_iterator const_iterator_vol_ref;
          typedef typename List::const_iterator const_iterator;
          // typedef const_iterator_const_ref<const_iterator_vers_ref, Ptr, 
          // ConstPtr> const_iterator;
          typedef std::list<iterator> IteratorList;

        public:

          ObjectList(); 

          int add(const Ptr& object);

          /**
           * @brief Fills a vector with all legit ObjectTypes from list.
           * @param vector [ObjectConstPtrVectorPtr] vector to be filled
           * @return void
           */
          void getAllLegitObjects(ObjectConstPtrVectorPtr vector) const;

          const_iterator begin() const;
          const_iterator end() const;

          int size() const;
          void pop_back();
          void clear();

          bool isObjectPoseInList(const ObjectConstPtr& object, float radius,
              bool is3D = true) const;
          void removeInRangeOfObject(const ObjectConstPtr& object, float range);

          void getObjectsPosesStamped(PoseStampedVector* poses) const;
          void fillGeotiff(
              pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response* res) const;
          void getVisualization(visualization_msgs::MarkerArray* markers) const;

        protected:

          bool isAnExistingObject(
              const ConstPtr& object, IteratorList* iteratorListPtr);

          virtual void updateObjects(const ConstPtr& object,
              const IteratorList& iteratorList);

          void removeElementAt(iterator it);

        protected:

          List objects_;

        private:

          int id_;

        private:

          friend class ObjectListTest;
      };

    typedef boost::shared_ptr< ObjectList<BaseObject> > ObjectListPtr;

    typedef boost::shared_ptr< const ObjectList<BaseObject> > ObjectListConstPtr;

    template <class ObjectType>
      ObjectList<ObjectType>::ObjectList() 
      {
        id_ = 0;
      }

    template <class ObjectType>
      typename ObjectList<ObjectType>::const_iterator
      ObjectList<ObjectType>::begin() const
      {
        return objects_.begin();
      }

    template <class ObjectType>
      typename ObjectList<ObjectType>::const_iterator
      ObjectList<ObjectType>::end() const
      {
        return objects_.end();
      }

    template <class ObjectType>
      int ObjectList<ObjectType>::add(const Ptr& object)
      {
        IteratorList iteratorList;

        if (isAnExistingObject(object, &iteratorList))
        {
          updateObjects(object, iteratorList);
          return 0;
        }

        object->setId(id_++);
        objects_.push_back(object);
        return ObjectType::getObjectScore();
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::getAllLegitObjects(
          ObjectConstPtrVectorPtr vector) const
      {
        const_iterator objectIt;

        for(objectIt = objects_.begin(); objectIt != objects_.end(); ++objectIt)
        {
          if((*objectIt)->getLegit())
          {
            vector->push_back(*objectIt);
          }
        }
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::removeElementAt(
          ObjectList<ObjectType>::iterator it)
      {
        objects_.erase(it);
      }

    template <class ObjectType>
      int ObjectList<ObjectType>::size() const
      {
        return objects_.size();
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::pop_back()
      {
        objects_.pop_back();
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::clear()
      {
        objects_.clear();
        id_ = 0;
      }

    template <class ObjectType>
      bool ObjectList<ObjectType>::isObjectPoseInList(
          const ObjectConstPtr& object, float radius, bool is3D) const
      {
        for(const_iterator it = this->begin(); it != this->end(); ++it)
        {
          float distance = 0;
          if(is3D)
          {
            distance = Utils::distanceBetweenPoints3D(object->getPose().position, 
                (*it)->getPose().position);
          }
          else
          {
            distance = Utils::distanceBetweenPoints2D(object->getPose().position, 
                (*it)->getPose().position);
          }
          if(distance < radius)
          {
            return true;
          }
        }

        return false;
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::removeInRangeOfObject(
          const ObjectConstPtr& object, float range)
      {
        iterator iter = objects_.begin();

        while (iter != objects_.end())
        {
          bool inRange = Utils::distanceBetweenPoints3D(
              object->getPose().position, (*iter)->getPose().position) < range;

          if ( inRange ) 
          {
            ROS_DEBUG_NAMED("object_handler",
                "[OBJECT_HANDLER %d] Deleting hole...", __LINE__);
            objects_.erase(iter++);
          }
          else 
          {
            ++iter;
          }
        }
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::getObjectsPosesStamped(
          PoseStampedVector* poses) const
      {
        for (const_iterator it = this->begin(); it != this->end(); ++it)
        {
          poses->push_back((*it)->getPoseStamped());
        }
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::fillGeotiff(
          pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response* res) const
      {
        for (const_iterator it = this->begin(); it != this->end(); ++it)
        {
          (*it)->fillGeotiff(res);
        }
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::getVisualization(
          visualization_msgs::MarkerArray* markers) const
      {
        markers->markers.clear();
        for (const_iterator it = this->begin(); it != this->end(); ++it)
        {
          (*it)->getVisualization(markers);
        }
      }

    template <class ObjectType>
      bool ObjectList<ObjectType>::isAnExistingObject(
          const ConstPtr& object, IteratorList* iteratorListPtr)
      {
        for (iterator it = objects_.begin(); it != objects_.end(); ++it)
        {
          if ((*it)->isSameObject(object))
          {
            iteratorListPtr->push_back(it);
          }
        }
        if (!iteratorListPtr->empty()) 
        {
          return true;
        }
        return false;
      }

    template <class ObjectType>
      void ObjectList<ObjectType>::updateObjects(const ConstPtr& object,
          const IteratorList& iteratorList)
      {
        for ( typename IteratorList::const_iterator it = iteratorList.begin(); 
            it != iteratorList.end(); ++it)
        {
          (*(*it))->update(object);
        }
      }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_LIST_H
