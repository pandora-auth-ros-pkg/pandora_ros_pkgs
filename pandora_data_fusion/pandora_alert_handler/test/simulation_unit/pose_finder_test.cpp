"Copyright [year] <Copyright Owner>"

#include <iostream>
#include "test/Image.h"
#include "map/map_attributes.h"
#include "test/test_utilities.h"
#include "alert_handler/pose_finder.h"
// #include <gtest/gtest.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "vision_communications/HolesDirectionsVectorMsg.h"

#define MAP_SIZE 4096

tf::StampedTransform lookupTransformFromWorld(std::string parentFrame, std::string childFrame)
{
  tf::StampedTransform tfTransform;
  tf::TransformListener listener;

  try 
  {
    listener.waitForTransform(parentFrame, childFrame, ros::Time::now(), ros::Duration(1));
    listener.lookupTransform(parentFrame, childFrame, ros::Time::now(), tfTransform);
  }
  catch (tf::TransformException ex) 
  {
    ROS_ERROR("%s", ex.what());
  }
  
  return tfTransform;
}

void getYawPitchFromVector(geometry_msgs::Point point, float* yawPtr, float* pitchPtr)
{
  float x = -point.y;
  float y = point.z;
  float z = point.x;
  // float absz = z>=0 ? z : -z;
  // if ( absz < 0.1 ) {
  // ROS_DEBUG(" Head too close to victim , z = %f " , absz );
  // break ;
  //  }
  *yawPtr = atan2(x, z);
  *pitchPtr = atan2(y, sqrt( x*x + z*z ));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_finder_test");
  
  MapAttributes mapAtt(MAP_SIZE, MAP_SIZE);
  
  std::string _mapImagePath;
  
  ros::NodeHandle _nh;
  if (_nh.hasParam("mapImagePath")) {
    _nh.getParam("mapImagePath", _mapImagePath);
    ROS_DEBUG("Set mapImagePath to %s", _mapImagePath.c_str());
  }
  else {
    ROS_DEBUG("Parameter mapImagePath not found. Using Default");
    _mapImagePath = "map1.png";
  }

  initMapFromImage(_mapImagePath, &mapAtt);
  
  tf::StampedTransform worldHeadCameraTransform = lookupTransformFromWorld(
      "world", "headCamera");
  tf::StampedTransform headCameraVictimTransform = lookupTransformFromWorld(
      "headCamera", "victim");
  
  tfScalar cameraYaw, cameraPitch, cameraRoll;
  worldHeadCameraTransform.getBasis().getRPY(cameraRoll, cameraPitch, cameraYaw);
  
  Utils utilsObj;
  
  geometry_msgs::Point headCamera2VictimPoint = 
    utilsObj.vector3ToPoint( headCameraVictimTransform.getOrigin() );

  float alertYaw;
  float alertPitch;

  getYawPitchFromVector(headCamera2VictimPoint, &alertYaw, &alertPitch);

  printf(" yaw = %f , pitch = %f\n", alertYaw, alertPitch);

  PoseFinder poseFinder(mapAtt.map, 5, -5);

  geometry_msgs::Pose victimPose2;
  
  try {
    victimPose2 = poseFinder.findAlertPose( alertYaw, alertPitch, worldHeadCameraTransform);
  }
  catch (AlertException ex) {
    ROS_ERROR("[POSE_FINDER_TEST %d] %s", __LINE__, ex.what());
    // return EXIT_FAILURE;
  }
  
  tf::TransformBroadcaster victimFrameBroadcaster;

  while(ros::ok()) {
    
    tf::Transform transVision = worldHeadCameraTransform;
    tf::Quaternion rotation;
    rotation.setRPY(0, cameraPitch-alertPitch, cameraYaw-alertYaw);
    transVision.setRotation(rotation);

    tf::Quaternion tfQuaternion(victimPose2.orientation.x, victimPose2.orientation.y, 
        victimPose2.orientation.z, victimPose2.orientation.w);
    tf::Vector3 vec(victimPose2.position.x, victimPose2.position.y, victimPose2.position.z);
    tf::Transform transVictim2(tfQuaternion, vec);

    victimFrameBroadcaster.sendTransform(
        tf::StampedTransform( transVision , ros::Time::now(), "world", "vision")  );
    victimFrameBroadcaster.sendTransform( 
        tf::StampedTransform( transVictim2 , ros::Time::now(), "world", "victim2")  );
    
    ros::Duration(1).sleep();
  }

}

