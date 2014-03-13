
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <pthread.h>
#include <set>
#include <sys/time.h>

#include "ros/ros.h"

#include "pandora_navigation_communications/navigationMapSrv.h"

#include <visualization_msgs/Marker.h>

using namespace std;

/*! \struct Point
    \brief Holds information about a Point
*/
struct Point{
	int x,y;
	Point(int x,int y){
		this->x=x;
		this->y=y;
	}
};

/**
@brief Finds the straight line between two points
@param x1 The first point - x coord
@param x2 The second point - x coord
@param y1 The first point - y coord
@param y2 The second point - y coord
@return A vector with the line points
 **/
std::vector<Point> getLine(int x1,int x2,int y1,int y2){
	vector<Point> ret;
	ret.clear(); 
	int temp;
	if(x1==x2){
		if(y1>y2){
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			ret.push_back(Point(x1,i));
		return ret;
	}
	if(x1>x2){
		temp=x1;
		x1=x2;
		x2=temp;
		temp=y1;
		y1=y2;
		y2=temp;
	}
	float l=((float)(y2-(float)y1)/(float)(x2-(float)x1));
	if((x2-x1)>abs(y2-y1)){	//Must find y's
		for(int i=x1;i<x2;i++)
			ret.push_back(Point(i,(int(l*i-l*x1+(float)y1))));
		return ret;
	}
	else{					//Must find x's
		if(y1>y2){
			temp=x1;
			x1=x2;
			x2=temp;
			temp=y1;
			y1=y2;
			y2=temp;
		}
		for(int i=y1;i<y2;i++)
			ret.push_back(Point((i-(float)y1+l*x1)/l,i));
			
			
		return ret;
	}
}

/**
@brief Streams the navigation map to GUI
@return True on success
 **/
int main (int argc,char **argv){
	ros::init(argc, argv, "rvizVisualizer");
	ros::NodeHandle n;

	ros::ServiceClient navigSrvClient,targetSrvClient;
	pandora_navigation_communications::navigationMapSrv navigMapSrv,targetSelMapSrv;
	navigSrvClient=n.serviceClient<pandora_navigation_communications::navigationMapSrv>("NavigationMapSrv/navigationController");
	targetSrvClient=n.serviceClient<pandora_navigation_communications::navigationMapSrv>("NavigationMapSrv/targetSelectorController");
	
	while (!ros::service::waitForService("/NavigationMapSrv/navigationController", ros::Duration(30)) && ros::ok()) {
		ROS_ERROR("[ Navigation visualizer ] Couldn't find service /NavigationMapSrv/navigationController");
	}
	while (!ros::service::waitForService("/NavigationMapSrv/targetSelectorController", ros::Duration(30)) && ros::ok()) {
		ROS_ERROR("[ Navigation visualizer ] Couldn't find service /NavigationMapSrv/targetSelectorController");
	}
	
	std::vector<unsigned char> map,coverage,voronoi;
	std::vector<int> voronodesx,voronodesy,goalsx,goalsy,coverageLimitsx,coverageLimitsy;
	int width,height,xx,yy;
	int xRobot,yRobot;
	float angleRobot;
	
	ros::Publisher voronodes_pub = n.advertise<visualization_msgs::Marker>("voronodes", 1);
	ros::Publisher voronodesCons_pub = n.advertise<visualization_msgs::Marker>("voronodesCons", 1);
	ros::Publisher voronoi_pub = n.advertise<visualization_msgs::Marker>("voronoi", 1);
	ros::Publisher decomp_pub = n.advertise<visualization_msgs::Marker>("decomposition", 1);
	ros::Publisher trajectory_pub = n.advertise<visualization_msgs::Marker>("trajectory", 1);

	while(ros::ok()){
		try {
			while(!navigSrvClient.call(navigMapSrv) && ros::ok())
				ros::Duration(1).sleep();
			while(!targetSrvClient.call(targetSelMapSrv) && ros::ok())
				ros::Duration(1).sleep();

			voronodesx=targetSelMapSrv.response.voronodesx;
			voronodesy=targetSelMapSrv.response.voronodesy;
				
			width=navigMapSrv.response.xsize;
			height=navigMapSrv.response.ysize;
			xRobot=navigMapSrv.response.xRobot;
			yRobot=navigMapSrv.response.yRobot;
			angleRobot=navigMapSrv.response.angleRobot;
			
			coverage=targetSelMapSrv.response.coverage;
			voronoi=navigMapSrv.response.voronoi;
			
			visualization_msgs::Marker voronodes;
			voronodes.header.frame_id = "/map";
			voronodes.header.stamp =ros::Time::now();
			voronodes.action = visualization_msgs::Marker::ADD;
			voronodes.id = 0;
			voronodes.type = visualization_msgs::Marker::SPHERE_LIST;
			voronodes.scale.x = 0.13;
			voronodes.scale.y = 0.13;
			voronodes.color.g = 0.5f;
			voronodes.color.a = 1.0;
			
			int voronodesSize=targetSelMapSrv.response.voronodesx.size();
			for(int i=0;i<voronodesSize;i++)
			{
				geometry_msgs::Point p;
				p.x = -(2048-targetSelMapSrv.response.prevxmin-targetSelMapSrv.response.voronodesx[i])*0.02;
				p.y = -(2048-targetSelMapSrv.response.prevymin-targetSelMapSrv.response.voronodesy[i])*0.02;
				p.z = 0.1;
				voronodes.points.push_back(p);
			}
			voronodes_pub.publish(voronodes);
			
			//--------------------------------------------------------------------------------//
			visualization_msgs::Marker voronodes_connections;
			voronodes_connections.header.frame_id = "/map";
			voronodes_connections.header.stamp =ros::Time::now();
			voronodes_connections.action = visualization_msgs::Marker::ADD;
			voronodes_connections.id = 1;
			voronodes_connections.type = visualization_msgs::Marker::POINTS;
			voronodes_connections.scale.x = 0.03;
			voronodes_connections.scale.y = 0.03;
			voronodes_connections.color.g = 0.2f;
			voronodes_connections.color.a = 0.5;
			for(unsigned int i=0;i<targetSelMapSrv.response.neighborsFirst.size();i++){
				vector<Point> tempv=getLine(voronodesx[targetSelMapSrv.response.neighborsFirst[i]],voronodesx[targetSelMapSrv.response.neighborsLast[i]],voronodesy[targetSelMapSrv.response.neighborsFirst[i]],voronodesy[targetSelMapSrv.response.neighborsLast[i]]);
				
				for(int j=0;j<tempv.size();j++)
				{
					geometry_msgs::Point p;
					p.x = -(2048-targetSelMapSrv.response.prevxmin-tempv[j].x)*0.02;
					p.y = -(2048-targetSelMapSrv.response.prevymin-tempv[j].y)*0.02;
					p.z = 0.1;
					voronodes_connections.points.push_back(p);
				}
			}
			voronodesCons_pub.publish(voronodes_connections);
			
			//------------------------------------------------------------------------------------//
			voronoi=targetSelMapSrv.response.voronoi;
			visualization_msgs::Marker voronoi_vis;
			voronoi_vis.header.frame_id = "/map";
			voronoi_vis.header.stamp =ros::Time::now();
			voronoi_vis.action = visualization_msgs::Marker::ADD;
			voronoi_vis.id = 1;
			voronoi_vis.type = visualization_msgs::Marker::POINTS;
			voronoi_vis.scale.x = 0.016;
			voronoi_vis.scale.y = 0.016;
			voronoi_vis.color.r = 0.9f;
			voronoi_vis.color.a = 0.8;

			for(int i=0;i<width;i++)
				for(int j=0;j<height;j++)
					if(voronoi[i*height+j]){
						geometry_msgs::Point p;
						p.x = -(2048-targetSelMapSrv.response.prevxmin-i)*0.02;
						p.y = -(2048-targetSelMapSrv.response.prevymin-j)*0.02;
						p.z = 0.1;
						voronoi_vis.points.push_back(p);
					}
			voronoi_pub.publish(voronoi_vis);
			//---------------------------------------------------------------------------------//
			//	DECOMP
			visualization_msgs::Marker decomp_vis;
			decomp_vis.header.frame_id = "/map";
			decomp_vis.header.stamp =ros::Time::now();
			decomp_vis.action = visualization_msgs::Marker::ADD;
			decomp_vis.id = 1;
			decomp_vis.type = visualization_msgs::Marker::POINTS;
			decomp_vis.scale.x = 0.015;
			decomp_vis.scale.y = 0.015;
			decomp_vis.color.r = decomp_vis.color.g = decomp_vis.color.b = 0.3f;
			decomp_vis.color.a = 0.2;

			for(unsigned int i=0;i<navigMapSrv.response.decompNeighborsFirst.size();i++){
				vector<Point> tempv=getLine(navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsFirst[i]],navigMapSrv.response.decompx[navigMapSrv.response.decompNeighborsLast[i]],navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsFirst[i]],navigMapSrv.response.decompy[navigMapSrv.response.decompNeighborsLast[i]]);
				for(unsigned int k=0;k<tempv.size();k++){
					geometry_msgs::Point p;
					p.x = -(2048-targetSelMapSrv.response.prevxmin-tempv[k].x)*0.02;
					p.y = -(2048-targetSelMapSrv.response.prevymin-tempv[k].y)*0.02;
					p.z = 0.1;
					decomp_vis.points.push_back(p);
				}
			}
			decomp_pub.publish(decomp_vis);
			
			ros::Duration(1).sleep();
		}
		catch (...) {
			ROS_ERROR("Visualization exception");
		}
		
	}
	return 0;
}
