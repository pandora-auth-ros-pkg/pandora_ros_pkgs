#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include "pandora_data_fusion_msgs/GetObjectsSrv.h"
#include "pandora_data_fusion_msgs/GetMarkersSrv.h"

typedef std::vector<geometry_msgs::PoseStamped> poseStampedVector;

class ObjectVisualization
{
    //!< Timers for tf
    ros::Timer _broadcastTimer;
    ros::Timer _broadcastTimer2;

    ros::Publisher _hazmat_marker_pub ; 
    ros::Publisher _landoltc_marker_pub ; 
    ros::Publisher _dataMatrix_marker_pub ; 
    ros::Publisher _hole_marker_pub ;
    ros::Publisher _qr_marker_pub ;
    ros::Publisher _thermal_marker_pub ;
    ros::Publisher _sound_marker_pub ;
    ros::Publisher _co2_marker_pub ;
    ros::Publisher _face_marker_pub ;
    ros::Publisher _motion_marker_pub ;
    ros::Publisher _victims_visited_marker_pub ;
    ros::Publisher _victims_to_go_marker_pub ;

    //!< Broadcasters fot tf
    tf::TransformBroadcaster _frameBroadcaster;

    ros::ServiceClient _getObjectsClient;
    ros::ServiceClient _getMarkersClient;

    ros::NodeHandle _nh;

    pandora_data_fusion_msgs::GetObjectsSrv _objectsSrv;
    pandora_data_fusion_msgs::GetMarkersSrv _markersSrv;

    void broadcastTimerCb(const ros::TimerEvent& event);

    void broadcastTimerCb2(const ros::TimerEvent& event);

    void broadcastPoseVector(poseStampedVector& vect);

  public:
    
    ObjectVisualization();
};

ObjectVisualization::ObjectVisualization()
{
  _broadcastTimer = _nh.createTimer(ros::Duration(0.1), &ObjectVisualization::broadcastTimerCb, this);

  _broadcastTimer2 = _nh.createTimer(ros::Duration(0.1), &ObjectVisualization::broadcastTimerCb2, this);


  while(!ros::service::waitForService("/data_fusion/get_objects", ros::Duration(30)) && ros::ok())
  {
    ROS_ERROR("[ ObjectVisualization ] Couldn't find service /data_fusion/get_objects");
  }

  while(!ros::service::waitForService("/data_fusion/get_markers", ros::Duration(.1)) && ros::ok()) 
  {
    ROS_ERROR("[ ObjectVisualization ] Couldn't find service /data_fusion/get_markers");
  }

  //~ ros::Duration(15).sleep();

  _getObjectsClient = _nh.serviceClient<pandora_data_fusion_msgs::GetObjectsSrv>
    ("/data_fusion/get_objects");

  _getMarkersClient = _nh.serviceClient<pandora_data_fusion_msgs::GetMarkersSrv>
    ("/data_fusion/get_markers");

  _hazmat_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("hazmats_markers", 1);
  _landoltc_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("landoltc_markers", 1);
  _dataMatrix_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("dataMatrix_markers", 1);
  _hole_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("holes_markers", 1);
  _qr_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("qrs_markers", 1);
  _thermal_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("thermals_markers", 1);
  _sound_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("sounds_markers", 1);
  _co2_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("co2s_markers", 1);
  _face_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("faces_markers", 1);
  _motion_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("motions_markers", 1);
  _victims_visited_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("victims_to_go_markers", 1);
  _victims_to_go_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("victims_visited_markers", 1);

  _broadcastTimer.start();
}

void ObjectVisualization::broadcastTimerCb(const ros::TimerEvent& event)
{
  while (!_getObjectsClient.call(_objectsSrv) && ros::ok())
  {
    ROS_ERROR("[ ObjectVisualization ] Failed to get objects for visualization. Retrying...");
    ros::Duration(0.5).sleep();
  }

  broadcastPoseVector(_objectsSrv.response.holes);
  broadcastPoseVector(_objectsSrv.response.qrs);
  broadcastPoseVector(_objectsSrv.response.landoltcs);
  broadcastPoseVector(_objectsSrv.response.dataMatrices);
  broadcastPoseVector(_objectsSrv.response.hazmats);
  broadcastPoseVector(_objectsSrv.response.thermals);
  broadcastPoseVector(_objectsSrv.response.sounds);
  broadcastPoseVector(_objectsSrv.response.co2s);
  broadcastPoseVector(_objectsSrv.response.faces);
  broadcastPoseVector(_objectsSrv.response.motions);
  broadcastPoseVector(_objectsSrv.response.victimsToGo);
}

void ObjectVisualization::broadcastTimerCb2(const ros::TimerEvent& event)
{
  while (!_getMarkersClient.call(_markersSrv) && ros::ok())
  {
    ROS_ERROR("[ ObjectVisualization ] Failed to get objects for visualization. Retrying...");
    ros::Duration(0.5).sleep();
  }

  _hole_marker_pub.publish(_markersSrv.response.holes);
  _hazmat_marker_pub.publish(_markersSrv.response.qrs);
  _qr_marker_pub.publish(_markersSrv.response.hazmats);
  _thermal_marker_pub.publish(_markersSrv.response.thermals);
  _sound_marker_pub.publish(_markersSrv.response.sounds);
  _co2_marker_pub.publish(_markersSrv.response.co2s);
  _face_marker_pub.publish(_markersSrv.response.faces);
  _motion_marker_pub.publish(_markersSrv.response.motions);
  _landoltc_marker_pub.publish(_markersSrv.response.landoltcs);
  _dataMatrix_marker_pub.publish(_markersSrv.response.dataMatrices);

  _victims_visited_marker_pub.publish(_markersSrv.response.victimsToGo);
  _victims_to_go_marker_pub.publish(_markersSrv.response.victimsVisited);
}

void ObjectVisualization::broadcastPoseVector(poseStampedVector& vect)
{
  for (poseStampedVector::iterator it=vect.begin(); it!=vect.end(); ++it)
  {
    tf::Quaternion tfQuaternion(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w);
    tf::Vector3 vec(it->pose.position.x,it->pose.position.y,it->pose.position.z);
    tf::Transform transVic(tfQuaternion, vec);

    ROS_DEBUG_NAMED("broadcastPoseVector","Publishing tf : %f , %f , %f , world to %s ",vec[0] ,vec[1] , vec[2] ,
        it->header.frame_id.c_str() );

    _frameBroadcaster.sendTransform( tf::StampedTransform( transVic, ros::Time::now(),
          "world", it->header.frame_id ) );
  }
}

int main (int argc, char **argv)
{
  ros::init(argc,argv,"object_visualization",ros::init_options::NoSigintHandler);

  ObjectVisualization yo;

  ros::spin();

  return 0;
}
