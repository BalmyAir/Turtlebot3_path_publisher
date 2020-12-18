#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <HM_WayPoint/HM_waypoint_server.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <stdlib.h>

//Message files
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PointStamped.h>

double poseAMCLx, poseAMCLy, poseAMCLa;
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = msgAMCL->pose.pose.orientation.w;   
    //ROS_INFO(msgAMCL);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "AMCL_Marker");
    ros::NodeHandle nh;
    //ros::Subscriber sub_amcl = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", poseAMCLCallback);
	WPS::WP_nav W(3);
	


	ros::Publisher marker_amcl_pub = nh.advertise<visualization_msgs::MarkerArray>("/hihihi", 10);
	ros::Rate loop_rate(1);
	while(ros::ok())
		{
			W.getAMCLpose();
			W.marker_amcl();
			//ROS_INFO("AMCL_pose: (%f,%f)",W.receivedAmclPose.pose.position.x, W.receivedAmclPose.pose.position.y);
			ROS_INFO("PUBLISH!!!  ");
			marker_amcl_pub.publish(W.mAMCL[0]);
			loop_rate.sleep();
			ros::spinOnce();
		}


  /*      ROS_INFO("Received Navigation Plan!!");
				ROS_INFO("Plan Size : %d", (int)receivedPlan.response.plan.poses.size());
				//this->testing.call(receivedPlan.request, receivedPlan.response);
				//std::cout << testing.getService() <<std::endl;
				ROS_INFO("------------------------------------------------");
				ROS_INFO("Plan Size : %d", receivedPlan.response.plan.poses[1].pose.position);
				for(int ii=0;ii<receivedPlan.response.plan.poses.size();ii++){
					visualization_msgs::Marker marker;
					marker.header.frame_id = "map";
					marker.header.stamp = ros::Time();
					marker.ns = "tempPath"+std::to_string(tempIdx);
					marker.id = ii;
					marker.type = visualization_msgs::Marker::SPHERE;
					marker.action = visualization_msgs::Marker::ADD;
					marker.pose.position.x = poseAMCLx;
					marker.pose.position.y = poseAMCLy;
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
					marker.lifetime = ros::Duration();
					mArr[tempIdx].markers.push_back(marker);
    int count = 0;
    while (ros::ok())
    {
        geometry_msgs::Pose error;
        error.position.x = poseAMCLx;
        error.position.y = poseAMCLy;
        error.orientation.w = poseAMCLa;
        pub.publish(error);
        ros::spinOnce();
        loop_rate.sleep();
        count=count+1;
    }
*/
    return 0;
}