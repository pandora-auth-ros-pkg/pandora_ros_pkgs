// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_OBJECT_LIST_H
#define ALERT_HANDLER_OBJECT_LIST_H

#include <list>
#include <vector>
#include <boost/iterator/iterator_adaptor.hpp>

#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/objects.h"
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
 
  typedef boost::shared_ptr< ObjectType > Ptr;
  typedef boost::shared_ptr< ObjectType const > ConstPtr;
  typedef std::list< Ptr > List;
  typedef typename List::iterator iterator;  
  typedef typename List::const_iterator const_iterator_vers_ref;
  typedef typename List::const_iterator const_iterator;
  // typedef const_iterator_const_ref<const_iterator_vers_ref, Ptr, 
            // ConstPtr> const_iterator;
  typedef std::list<iterator> IteratorList;

 public:

  ObjectList(int counterThreshold = 1, float distanceThreshold = 0.5);

  const_iterator begin() const;
  const_iterator end() const;
  int size() const;
  bool isObjectPoseInList(const ObjectConstPtr& object, float closestAlert) const;

  bool add(const Ptr& object);
  void pop_back();
  void clear();

  void removeInRangeOfObject(const ObjectConstPtr& object, float range);

  void getObjectsPosesStamped(
    std::vector<geometry_msgs::PoseStamped>* poses) const;

  void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;

  void getVisualization(visualization_msgs::MarkerArray* markers) const;

  void setParams(int counterThreshold, float distanceThreshold);

 protected:

  bool isAnExistingObject(
    const ConstPtr& object, IteratorList* iteratorListPtr);

  void updateObject(
    const Ptr& object,
      const IteratorList& iteratorList);

  void removeElementAt(iterator it);

 protected:

  List objects_;
  float DIST_THRESHOLD;
  int COUNTER_THRES;

 private:

  friend class ObjectListTest;

 private:

  int id_;

};

typedef boost::shared_ptr< ObjectList<Object> > ObjectListPtr;
typedef boost::shared_ptr< ObjectList<Hole> >  HoleListPtr;
typedef boost::shared_ptr< ObjectList<Qr> >  QrListPtr;
typedef boost::shared_ptr< ObjectList<Hazmat> > HazmatListPtr;
typedef boost::shared_ptr< ObjectList<Tpa> >  TpaListPtr;

typedef boost::shared_ptr< const ObjectList<Object> > ObjectListConstPtr;
typedef boost::shared_ptr< const ObjectList<Hole> >  HoleListConstPtr;
typedef boost::shared_ptr< const ObjectList<Qr> >  QrListConstPtr;
typedef boost::shared_ptr< const ObjectList<Hazmat> > HazmatListConstPtr;
typedef boost::shared_ptr< const ObjectList<Tpa> >  TpaListConstPtr;

template <class ObjectType>
ObjectList<ObjectType>::ObjectList(int counterThreshold,
    float distanceThreshold)
{
  id_ = 0;
  COUNTER_THRES = counterThreshold;
  DIST_THRESHOLD = distanceThreshold;
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
bool ObjectList<ObjectType>::add(const Ptr& object)
{
  IteratorList iteratorList;

  if (isAnExistingObject(object, &iteratorList))
  {
    updateObject(object, iteratorList);
    return false;
  }

  object->setId(id_++);
  object->incrementCounter();
  objects_.push_back(object);
  return true;
}

template <class ObjectType>
void ObjectList<ObjectType>::removeElementAt(
  ObjectList<ObjectType>::iterator it)
{
    objects_.erase(it);
}

template <class ObjectType>
void ObjectList<ObjectType>::setParams(int counterThreshold,
    float distanceThreshold)
{
  COUNTER_THRES = counterThreshold;
  DIST_THRESHOLD = distanceThreshold;
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
}

template <class ObjectType>
bool ObjectList<ObjectType>::isObjectPoseInList(
    const ObjectConstPtr& object, float range) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    float distance =
      Utils::distanceBetweenPoints3D(object->getPose().position,
                                      (*it)->getPose().position);
         
    if (distance < range)
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
    std::vector<geometry_msgs::PoseStamped>* poses) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    poses->push_back((*it)->getPoseStamped());
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const
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
    if ((*it)->isSameObject(object, DIST_THRESHOLD))
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
void ObjectList<ObjectType>::updateObject(
    const Ptr& object,
      const IteratorList& iteratorList)
{
  int totalCounter = 0;
  int maxCounter = (*iteratorList.front())->getCounter();
  int maxId = (*iteratorList.front())->getId();

  for ( typename IteratorList::const_iterator it = iteratorList.begin();
         it != iteratorList.end() ; ++it)
  {
    // find total counter value, set new object's id as the id of the
    // object with the highest counter as of now.
    totalCounter += (*(*it))->getCounter();
    if ((*(*it))->getCounter() > maxCounter)
    {
      maxCounter = (*(*it))->getCounter();
      maxId = (*(*it))->getId();
    }
    removeElementAt(*it);
  }

  object->setId(maxId);
  object->setCounter(++totalCounter);

  if (object->getCounter() > COUNTER_THRES)
  {
    object->setLegit(true);
  }

  objects_.push_back(object);
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_LIST_H
