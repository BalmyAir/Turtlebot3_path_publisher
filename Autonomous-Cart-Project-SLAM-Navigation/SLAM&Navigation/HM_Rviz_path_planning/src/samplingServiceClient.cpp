/*
HPC_LAB Sampling Service
	Sample the only 4 way Point(distance = maybe 1.0m)
*/

//ros basic file
#include <ros/ros.h>
#include <iostream>

#include <HM_WayPoint/HM_waypoint_server.h>


int main(int argc, char **argv)
{

	ros::init(argc, argv, "samplingServiceClient");

	ROS_INFO("Client");

	ros::NodeHandle nh;

	bool arrived = false;

	//ros::ServiceServer service = nh.advertiseService("samplingWaypoint", samplingWaypoint);
	//ROS_INFO("Ready to sampling WayPoint");

	ros::ServiceClient c = nh.serviceClient<nav_msgs::GetPlan>("samplingWaypoint");

	WPS::WP_nav W(0);

	nav_msgs::GetPlan temp_path;

	WPS::MoveBaseClient ac("move_base", true);

	//W.SetAmclPose

	W.getAMCL_Goalpose();

	temp_path.request.start.pose = W.receivedAmclPose.pose;
	temp_path.request.start.header = W.receivedAmclPose.header;

	temp_path.request.goal.header = W.receivedPose.header;
	temp_path.request.goal.pose = W.receivedPose.pose;
	temp_path.request.tolerance = 0.5;

	while (!arrived)
	{

		//choose the distance()meter
		//temp_path.request.tolerance = 0.5;

		//temp_path.response.plan.poses.resize(5);

		c.call(temp_path);

		ROS_INFO("Received Service");

		ros::Publisher temp_publisher = nh.advertise<nav_msgs::Path>("/sampling/goals", 1);

		bool publish = false;

		ROS_INFO("Client : Start Push!!");

		//geometry_msgs::PoseStamped temp_pub_WP;

		//visualization in Rviz

		while (!publish)
		{
			if (temp_publisher.getNumSubscribers() > 0)
			{
				temp_publisher.publish(temp_path.response.plan);
				ROS_INFO("new Plan Push!!");

				ros::Duration(1.0);
				publish = true;
			}
		}

		//go to wayPoint
		ROS_INFO("Start Navigation");

		while (!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}

		for (int i = 0; i < temp_path.response.plan.poses.size(); i++)
		{
			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position = temp_path.response.plan.poses[i].pose.position;
			goal.target_pose.pose.orientation = temp_path.response.plan.poses[i].pose.orientation;
			ac.sendGoal(goal);

			while (!ac.waitForResult(ros::Duration(2.0)))
			{
				if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
					ROS_INFO("Processing..");
			}

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("GOOD");
			else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
				ROS_INFO("LOST");
		}

		WPS::WP_nav T(0);
		T.getAMCLpose();

		if (T.receivedAmclPose.pose.position.x == temp_path.request.goal.pose.position.x &&
			T.receivedAmclPose.pose.position.y == temp_path.request.goal.pose.position.y &&
			T.receivedAmclPose.pose.position.z == temp_path.request.goal.pose.position.z)
		{

			arrived = true;
		}

		ROS_INFO("Received the Start Position Again");

		temp_path.request.start.pose = T.receivedAmclPose.pose;
		temp_path.request.start.header = T.receivedAmclPose.header;
	}

	//c.call()

	ROS_INFO("Sampling Service Client FINISHED");

	ros::spin();

	return 0;
}