#include "ros/ros.h"

#include "pandora_data_fusion_msgs/GetMarkersSrv.h"

class ObjectVisualization
{
    //!< Timers for visualization
    ros::Timer _broadcastTimer;

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

    ros::ServiceClient _getMarkersClient;

    ros::NodeHandle _nh;

    pandora_data_fusion_msgs::GetMarkersSrv _markersSrv;

    void broadcastTimerCb(const ros::TimerEvent& event);

  public:
    
    ObjectVisualization();
};

ObjectVisualization::ObjectVisualization()
{
  _broadcastTimer = _nh.createTimer(ros::Duration(0.1), &ObjectVisualization::broadcastTimerCb, this);

  while(!ros::service::waitForService("/data_fusion/get_markers", ros::Duration(.1)) && ros::ok()) 
  {
    ROS_ERROR("[ ObjectVisualization ] Couldn't find service /data_fusion/get_markers");
  }

  //~ ros::Duration(15).sleep();

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

int main (int argc, char **argv)
{
  ros::init(argc,argv,"object_visualization",ros::init_options::NoSigintHandler);

  ObjectVisualization yo;

  ros::spin();

  return 0;
}
