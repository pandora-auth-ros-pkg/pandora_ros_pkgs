// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object()
{
  legit_ = false;
  frame_id_ = "/world";
}

PoseStamped Object::getPoseStamped() const
{
  PoseStamped objPoseStamped;
  objPoseStamped.pose = pose_;
  return objPoseStamped;
}

bool Object::isSameObject(const ConstPtr& object, float distance) const
{
  return Utils::distanceBetweenPoints3D(
      pose_.position, object->getPose().position)
      < distance;
}

void Object::initializeObjectFilter(float prior_x_sd, float prior_y_sd,
    float prior_z_sd)
{
  //!< Priors  
  //!< Filter's prior mean
  MatrixWrapper::ColumnVector priorMu_(1);
  //!< Filter's prior covariance
  MatrixWrapper::SymmetricMatrix priorVar_(1, 1);

  priorMu_(1) = pose_.position.x;
  priorVar_(1, 1) = pow(prior_x_sd, 2);
  priorX_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterX_.reset( new Filter(priorX_.get()) );
  
  priorMu_(1) = pose_.position.y;
  priorVar_(1, 1) = pow(prior_y_sd, 2);
  priorY_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterY_.reset( new Filter(priorY_.get()) );
  
  priorMu_(1) = pose_.position.z;
  priorVar_(1, 1) = pow(prior_z_sd, 2);
  priorZ_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterZ_.reset( new Filter(priorZ_.get()) );
}

/**
 * @details Updates this object's position according to new measurement's
 * position. This update is the result of the change in object's conviction
 * pdf on its position which is calculated from the given filter model
 * and the current measurement. The filter is an implementation of
 * Kalman Filter.
 */
void Object::update(const ConstPtr& measurement, 
    const FilterModelConstPtr& model)
{
  Point measurementPosition = measurement->getPose().position;
  MatrixWrapper::ColumnVector newPosition(1);
  //!< Filter's input vector
  MatrixWrapper::ColumnVector input(1);  
  //!< Input is 0.0 as our actions doesn't change the world model.
  input(1) = 0.0;
 
  //!< Updating existing object's filter pdfs.
  SystemModelPtrVector systemModels = model->getSystemModels();
  MeasurementModelPtrVector measurementModels = model->getMeasurementModels();
  
  newPosition(1) = measurementPosition.x;
  filterX_->Update(systemModels[0].get(), 
      input, measurementModels[0].get(), newPosition);
    
  newPosition(1) = measurementPosition.y;
  filterY_->Update(systemModels[1].get(), 
      input, measurementModels[1].get(), newPosition);
    
  newPosition(1) = measurementPosition.z;
  filterZ_->Update(systemModels[2].get(), 
      input, measurementModels[2].get(), newPosition);

  //!< Updating existing object's expected pose.
  Pose newObjectPose;
  newObjectPose.position.x = filterX_->PostGet()
    ->ExpectedValueGet()(1);
  newObjectPose.position.y = filterY_->PostGet()
    ->ExpectedValueGet()(1);
  newObjectPose.position.z = filterZ_->PostGet()
    ->ExpectedValueGet()(1);

  //!< Setting existing object's orientation.
  newObjectPose.orientation = pose_.orientation;

  pose_ = newObjectPose;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

