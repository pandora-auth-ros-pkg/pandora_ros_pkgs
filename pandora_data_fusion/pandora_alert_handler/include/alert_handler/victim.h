// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_VICTIM_H
#define ALERT_HANDLER_VICTIM_H

#include <vector>
#include <set>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

#include "data_fusion_communications/DatafusionGeotiffSrv.h"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class Victim : public Object
{
 public:
 
  typedef boost::shared_ptr<Victim> Ptr;
  typedef boost::shared_ptr<Victim const> ConstPtr;

 public:

  Victim();

  virtual bool isSameObject(const ObjectConstPtr& object, float distance) const;

  virtual geometry_msgs::PoseStamped getPoseStamped() const;
  
  virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

  virtual void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;

  geometry_msgs::PoseStamped getApproachPoint() const;
  
  void setObjects(const ObjectConstPtrVector& objects, float approachDistance);
  
  void eraseObjectAt(int index, float approachDistance);
  
  void sanityCheck(const ObjectConstPtrVectorPtr& allObjects,
            float distThreshold, float approachDistance);
   
  void addSensor(int sensorId);

  tf::Transform getRotatedTransform() const;
  
  /**
  @brief Getter for member valid_
  @return bool valid_
  **/
  bool getValid() const
  {
    return valid_;
  }
   
  /**
  @brief Getter for member visited_
  @return bool visited_
  **/
  bool getVisited() const
  {
    return visited_;
  }
   
  /**
  @brief Getter for member selectedObjectIndex_
  @return bool selectedObjectIndex_
  **/
  int getSelectedObjectIndex() const
  {
    return selectedObjectIndex_;
  }
  
  /**
  @brief Getter for member approachPose_
  @return const geometry_msgs::Pose& approachPose_
  **/
  const geometry_msgs::Pose& getApproachPose() const
  {
    return approachPose_;
  }
  
  /**
  @brief Getter for member sensorIds_
  @return std::set<int>& sensorIds_
  **/
  const std::set<int>& getSensorIds() const
  {
    return sensorIds_;
  }
  
  /**
  @brief Getter for member objects_
  @return std::set<int>& objects_
  **/
  const ObjectConstPtrVector& getObjects() const
  {
    return objects_;
  }
   
  /**
  @brief Setter for member valid_
  @param valid [bool] The new valid_ value
  @return void
  **/
  void setValid(bool valid)
  {
    valid_ = valid;
  }
  
  /**
  @brief Setter for member visited_
  @param visited [bool] The new visited_ value
  @return void
  **/
  void setVisited(bool visited)
  {
    visited_ = visited;
  }
    
 private:
   
  void updatePose(const geometry_msgs::Pose& newPose, float approachDistance);

  /**
  @brief Calculates the approach pose of the victim for it's current pose
  @param approachDistance [float] The disired distance from the wall
  @return geometry_msgs::Pose The approach pose
  **/
  geometry_msgs::Pose calculateApproachPose(float approachDistance) const;
  
  /**
  @brief Updates the representative object and consequently the pose 
  @param approachDistance [float] The disired distance from the wall
  @details Should be always called after any change on the objects_
  @return void
  **/
  void updateRepresentativeObject(float approachDistance);
  
  int findRepresentativeObject() const;
  
  tf::Transform getTransform() const;
  
 protected:
  
  //!< The validity of the Victim
  bool valid_;      
  //!< True if the victim was visited false otherwise     
  bool visited_;            

  int selectedObjectIndex_;

  geometry_msgs::Pose approachPose_;    
  
  //!< Holds the type of sensor that created the alarm
  std::set<int> sensorIds_;   
  
  ObjectConstPtrVector objects_;
 
 private:

  friend class VictimTest;
  friend class VictimClustererTest;

 private:
  
  static int lastVictimId_;  //!< The last in line victim ID
  
};

typedef Victim::Ptr VictimPtr;
typedef Victim::ConstPtr VictimConstPtr;
typedef std::vector<VictimPtr> VictimPtrVector;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_H
