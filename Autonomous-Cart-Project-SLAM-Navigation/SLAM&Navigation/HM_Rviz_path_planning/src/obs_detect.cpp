#include <ros/ros.h>

//Header files
//#include <HM_Rviz_path_planning/HM_visualization.h>
#include <HM_WayPoint/HM_waypoint_server.h>

//Message files
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
//Action
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <stdlib.h>

#include <tf/tf.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>
using namespace std;
// std_msgs/Header header
// float32 angle_min
// float32 angle_max
// float32 angle_increment
// float32 time_increment
// float32 scan_time
// float32 range_min
// float32 range_max
// float32[] ranges
// float32[] intensities

float posX=0;
float posY=0;
float yaw=0;
ros::Publisher marker_arr_pub;
float quatToDegree(float x, float y, float z, float w){
    float yaw = atan2(2.0*(y*x+z*w),(-z*z-y*y+x*x+w*w));
    yaw=yaw*180/3.141592;
    return yaw;
}
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &data){
	posX=data->pose.pose.position.x;
	posY=data->pose.pose.position.y;
	yaw=quatToDegree(data->pose.pose.orientation.x,data->pose.pose.orientation.y,data->pose.pose.orientation.z,data->pose.pose.orientation.w);
	cout<<"position "<<posX<<" "<<posY<<" Yaw "<<yaw<<endl;
}
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& data){
	float boxLength = 2;
	float distance = 3;

	visualization_msgs::MarkerArray mArr;

	for(int i=0;i<data->ranges.size();i++){
		if(data->ranges[i]>5) cout<<"INF"<<endl;
		else{
			cout<<i<<" ranges"<<data->ranges[i]<<endl;
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "my_Points";
			marker.id = i;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = posX+(data->ranges[i])*cos((yaw+i)*3.141592/180);
			marker.pose.position.y = posY+(data->ranges[i])*sin((yaw+i)*3.141592/180);
			marker.pose.position.z = 1;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.color.a = 1.0; // Don't forget to set the alpha!

			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			if(	abs(data->ranges[i]*sin(i/180*3.141592)) < boxLength/2 	&&\
				distance < data->ranges[i]*cos(i/180*3.141592)			&&\
				data->ranges[i]*cos(i/180*3.141592) < distance+boxLength){
				marker.color.r = 1.0;
				marker.color.g = 0.0;

			}
			
			marker.lifetime = ros::Duration();
			mArr.markers.push_back(marker);
		}
		
	}
	bool trigger=false;
	for(int i=0;i<30;i++){
		if(	abs(data->ranges[i]*sin(i/180*3.141592)) < boxLength/2 	&&\
			distance < data->ranges[i]*cos(i/180*3.141592)			&&\
			data->ranges[i]*cos(i/180*3.141592) < distance+boxLength){
			trigger=true;
		}
	}
	for(int i=330;i<360;i++){
		if(	abs(data->ranges[i]*sin(i/180*3.141592)) < boxLength/2 	&&\
			distance < data->ranges[i]*cos(i/180*3.141592)			&&\
			data->ranges[i]*cos(i/180*3.141592) < distance+boxLength){
			trigger=true;
		}
	}
	marker_arr_pub.publish(mArr);

	if(trigger){
		cout<<"Object Detected!!"<<endl;
	}else{
		cout<<"Clear"<<endl;
	}


}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "obs_detect");

	ROS_INFO("HI");
	ROS_INFO("HI : %d", TEST);

	ros::NodeHandle nh;
	ros::Subscriber goal_fromRviz = nh.subscribe("/scan", 3, lidar_callback);
	ros::Subscriber pose = nh.subscribe("amcl_pose",10,pose_callback);
	marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("/my2DScan", 10);

	ros::Rate loop(10);
	while(ros::ok){
		loop.sleep();
		ros::spinOnce();
	}
	return 0;
}
