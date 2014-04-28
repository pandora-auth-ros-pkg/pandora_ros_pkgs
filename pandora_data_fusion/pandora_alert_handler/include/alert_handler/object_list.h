// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_OBJECT_LIST_H
#define ALERT_HANDLER_OBJECT_LIST_H

#include <list>
#include <vector>
#include <boost/iterator/iterator_adaptor.hpp>

#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/objects.h"
#include "alert_handler/filter_model.h"
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

  //!< Type definitions
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

  ObjectList(); 

  const_iterator begin() const;
  const_iterator end() const;
  int size() const;
  bool isObjectPoseInList(const ObjectConstPtr& object,
      float closestAlert) const;

  int add(const Ptr& object);
  void pop_back();
  void clear();

  void removeInRangeOfObject(const ObjectConstPtr& object, float range);

  void getObjectsPosesStamped(PoseStampedVector* poses) const;

  void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;

  void getVisualization(visualization_msgs::MarkerArray* markers) const;

  void setParams(int objectScore, float distanceThreshold, float x_var_thres = 0.0009, 
      float y_var_thres = 0.0009, float z_var_thres = 0.0009,
      float prior_x_sd = 0.05, float prior_y_sd = 0.05, float prior_z_sd = 0.05,
      float system_noise_sd = 0.003, float measurement_noise_sd = 0.05);

  FilterModelPtr getFilterModel() const;

 protected:

  bool isAnExistingObject(
    const ConstPtr& object, IteratorList* iteratorListPtr);

  virtual void updateObjects(const ConstPtr& object,
    const IteratorList& iteratorList);

  void checkLegit(const Ptr& object);

  void removeElementAt(iterator it);

 protected:

  List objects_;

  FilterModelPtr filterModelPtr_;

  //!< params
  float DIST_THRESHOLD;

 private:

  friend class ObjectListTest;

 private:

  int id_;

  //!< params
  int OBJECT_SCORE;

  float X_VAR_THRES;
  float Y_VAR_THRES;
  float Z_VAR_THRES;

  float PRIOR_X_SD;
  float PRIOR_Y_SD;
  float PRIOR_Z_SD;

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
ObjectList<ObjectType>::ObjectList() 
{
  filterModelPtr_.reset( new FilterModel );
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

  object->initializeObjectFilter(PRIOR_X_SD, PRIOR_Y_SD, PRIOR_Z_SD);
  
  object->setId(id_++);
  objects_.push_back(object);
  return OBJECT_SCORE;
}

template <class ObjectType>
void ObjectList<ObjectType>::removeElementAt(
  ObjectList<ObjectType>::iterator it)
{
    objects_.erase(it);
}

template <class ObjectType>
void ObjectList<ObjectType>::setParams(int objectScore, 
    float distanceThreshold, float x_var_thres, 
    float y_var_thres, float z_var_thres,
    float prior_x_sd, float prior_y_sd, float prior_z_sd,
    float system_noise_sd, float measurement_noise_sd)
{
  OBJECT_SCORE = objectScore;
  DIST_THRESHOLD = distanceThreshold;
  X_VAR_THRES = x_var_thres;
  Y_VAR_THRES = y_var_thres;
  Z_VAR_THRES = z_var_thres;
  PRIOR_X_SD = prior_x_sd;
  PRIOR_Y_SD = prior_y_sd;
  PRIOR_Z_SD = prior_z_sd;
  filterModelPtr_->setParams(system_noise_sd, measurement_noise_sd);
  filterModelPtr_->initializeFilterModel();
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
FilterModelPtr ObjectList<ObjectType>::getFilterModel() const
{
  return filterModelPtr_;
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
    PoseStampedVector* poses) const
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
void ObjectList<ObjectType>::updateObjects(const ConstPtr& object,
    const IteratorList& iteratorList)
{
  for ( typename IteratorList::const_iterator it = iteratorList.begin(); 
      it != iteratorList.end(); ++it)
  {
    (*(*it))->update(object, filterModelPtr_);
    checkLegit((*(*it)));
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::checkLegit(const Ptr& object)
{
  bool objectIsLegit = object->getVarianceX() < X_VAR_THRES && 
                       object->getVarianceY() < Y_VAR_THRES &&
                       object->getVarianceZ() < Z_VAR_THRES;
  if (objectIsLegit)
  {
    object->setLegit(true);
  }
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_LIST_H
