#include "ros/ros.h"

#include <tf/transform_broadcaster.h>
#include "pandora_data_fusion_msgs/GetObjectsSrv.h"
#include "pandora_data_fusion_msgs/GetMarkersSrv.h"

typedef std::vector<geometry_msgs::PoseStamped> poseStampedVector;

class ObjectVisualization {


	// timers for tf
	ros::Timer _broadcastTimer;
	ros::Timer _broadcastTimer2;

	ros::Publisher _hazmat_marker_pub ; 
	ros::Publisher _hole_marker_pub ;
	ros::Publisher _qr_marker_pub ;
	ros::Publisher _thermal_marker_pub ;
	ros::Publisher _victims_visiter_marker_pub ;
	ros::Publisher _victims_to_go_marker_pub ;

	// broadcasters fot tf
	tf::TransformBroadcaster _holeFrameBroadcaster;
	tf::TransformBroadcaster _qrFrameBroadcaster;
	tf::TransformBroadcaster _hazmatFrameBroadcaster;
	
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



ObjectVisualization::ObjectVisualization(){
	
	_broadcastTimer = _nh.createTimer(ros::Duration(0.1), &ObjectVisualization::broadcastTimerCb, this);
	
	_broadcastTimer2 = _nh.createTimer(ros::Duration(0.1), &ObjectVisualization::broadcastTimerCb2, this);


	while (!ros::service::waitForService("/data_fusion/get_objects", ros::Duration(30)) && ros::ok()) {
		
		ROS_ERROR("[ ObjectVisualization ] Couldn't find service /data_fusion/get_objects");
	}
	
	while (!ros::service::waitForService("/data_fusion/get_markers", ros::Duration(.1)) && ros::ok()) {
		
		ROS_ERROR("[ ObjectVisualization ] Couldn't find service /data_fusion/get_markers");
	}
	
	//~ ros::Duration(15).sleep();
	
	
	_getObjectsClient = _nh.serviceClient<pandora_data_fusion_msgs::GetObjectsSrv>
		("/data_fusion/get_objects");
		
	_getMarkersClient = _nh.serviceClient<pandora_data_fusion_msgs::GetMarkersSrv>
		("/data_fusion/get_markers");

	_hazmat_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("hazmats_markers", 1);
	_hole_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("holes_markers", 1);
	_qr_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("qrs_markers", 1);
	_thermal_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("thermals_markers", 1);
	_victims_visiter_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("victims_to_go_markers", 1);
	_victims_to_go_marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("victims_visited_markers", 1);

	_broadcastTimer.start();
}

void ObjectVisualization::broadcastTimerCb(const ros::TimerEvent& event){
	
	while (!_getObjectsClient.call(_objectsSrv) && ros::ok()){
		ROS_ERROR("[ ObjectVisualization ] Failed to get objects for visualization. Retrying...");
		ros::Duration(0.5).sleep();
	}
	
	broadcastPoseVector(_objectsSrv.response.holes);
	broadcastPoseVector(_objectsSrv.response.qrs);
	broadcastPoseVector(_objectsSrv.response.hazmats);
	broadcastPoseVector(_objectsSrv.response.thermals);
	broadcastPoseVector(_objectsSrv.response.victimsToGo);
}

void ObjectVisualization::broadcastTimerCb2(const ros::TimerEvent& event){
	
	
	while (!_getMarkersClient.call(_markersSrv) && ros::ok()){
		ROS_ERROR("[ ObjectVisualization ] Failed to get objects for visualization. Retrying...");
		ros::Duration(0.5).sleep();
	}
	
	if (_markersSrv.response.holes.markers.size()>0 || _markersSrv.response.qrs.markers.size()>0 || _markersSrv.response.hazmats.markers.size() >0){
		
		std::cout << "holes size: " << _markersSrv.response.holes.markers.size() << "\n";
		std::cout << "qrs size: " << _markersSrv.response.qrs.markers.size() << "\n";
		std::cout << "hazmats size: " << _markersSrv.response.hazmats.markers.size() << "\n";
	}
	
	_hole_marker_pub.publish(_markersSrv.response.holes);
	_hazmat_marker_pub.publish(_markersSrv.response.qrs);
	_qr_marker_pub.publish(_markersSrv.response.hazmats);
	_thermal_marker_pub.publish(_markersSrv.response.thermals);
	_victims_visiter_marker_pub.publish(_markersSrv.response.victimsToGo);
	_victims_to_go_marker_pub.publish(_markersSrv.response.victimsVisited);

}

void ObjectVisualization::broadcastPoseVector(poseStampedVector& vect)
{
		
	for (poseStampedVector::iterator it=vect.begin(); it!=vect.end(); ++it) {
		
		tf::Quaternion tfQuaternion(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w);
		tf::Vector3 vec(it->pose.position.x,it->pose.position.y,it->pose.position.z);
		tf::Transform transVic(tfQuaternion, vec);
		
		ROS_DEBUG_NAMED("broadcastPoseVector","Publishing tf : %f , %f , %f , world to %s ",vec[0] ,vec[1] , vec[2] ,
			it->header.frame_id.c_str() );
		
		_holeFrameBroadcaster.sendTransform( tf::StampedTransform( transVic, ros::Time::now(),
			"world", it->header.frame_id ) );
		
	}
	
	
}

int main (int argc, char **argv){
	
	ros::init(argc,argv,"object_visualization",ros::init_options::NoSigintHandler);
	
	ObjectVisualization yo;
	
	ros::spin();
	return 0;
}
