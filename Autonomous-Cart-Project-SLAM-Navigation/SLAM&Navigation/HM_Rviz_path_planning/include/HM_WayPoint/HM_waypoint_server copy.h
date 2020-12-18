/*
HPC_LAB way_Point_Server
visualize way_Point received navigation in Rviz


It must run after run ROS navigation node
It uses Ros navigation node

*/
//basic ros header file
#include <ros/ros.h>

//Header files
#include <HM_Rviz_path_planning/HM_visualization.h>

//Message header file(it have to write related dependency file name in CMakeList and package)
#include <nav_msgs/OccupancyGrid.h>
//For visualization of shape in Rviz
#include <visualization_msgs/MarkerArray.h>
//For location with position, orientation, Covariance
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//For location(position, orientation)
#include <geometry_msgs/PoseStamped.h>
//For visualization of Color in Rviz
#include <std_msgs/ColorRGBA.h>
//For location(position)
#include <geometry_msgs/PointStamped.h>
//For move_base_msgs
#include <move_base_msgs/MoveBaseAction.h>


//For using Service of Movebase in Navigation
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>


//Action
#include <actionlib/client/simple_action_client.h>



#include <iostream>
#include <stdlib.h>

#include <tf/tf.h>

namespace WPS
{
#define MARKERMAXNUM 10

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


	class WP_nav
	{
		//Make the handle
		ros::NodeHandle nh;

		//receivedPoint
		ros::Subscriber receivedPointArray_BYRVIZ;

		//received Fake Goal from RVIZ
		ros::Subscriber receivedPosefromRVIZ;

		//received Temporal Start pose from RVIZ
		ros::Subscriber receivedStartPoseFromRVIZ;

		//received amcl_pose
		ros::Subscriber receivedfromAmclPose;

		//ServiceClient
		ros::ServiceClient clearCostMap;
		ros::ServiceClient receivedMapPlan;

		//MoveBaseClient
		//MoveBaseClient ac;

		//MoveBaseClient ac("move_base", true);
		HM_rviz_Marker test[MARKERMAXNUM] =
			{HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0),
			 HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0), HM_rviz_Marker(1.0, 1.0)};

		geometry_msgs::PoseStamped receivedPose;

		geometry_msgs::PoseStamped receivedStartPose;

		geometry_msgs::PoseStamped receivedAmclPose;

	public:
		int Current_nWay = 0;
		int nWay = 0;

		int cgoal = -1;
		int rgoal = 0;

		WP_nav(int a)
		{
			ROS_INFO("Make_WP_nav");
		}

		void start_sub_receivedPointArray()
		{
			receivedPointArray_BYRVIZ = nh.subscribe("/PointArray", 5, &WP_nav::receivedPointArray_Callback, this);
		}

		void finish_sub_receivedPointArray()
		{
			receivedPointArray_BYRVIZ.shutdown();
		}

		void receivedPointArray_Callback(const visualization_msgs::MarkerArrayPtr &_msg);

		void receivedPoseofRVIZ_Callback(const geometry_msgs::PoseStamped::ConstPtr &_msg);

		
		//Callback Function (Received current Pose by Rviz(initial pose))
		void Setinitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &_msg);

		//Callback Function (Received current Pose by AMCL(movebase))
		void SetAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &_msg);

		//when pick the point by Rviz, just send the point the movebase
		void sendPoint_forJustNavigation();

		//when pick the point by Rviz, send the point(cycle) the movebase
		void sendPoint_forCycleNavigation();


		//set the initial pose manually
		void receiveWaypointforMovebase();

		//set the initial pose automatically received from amcl
		void receiveWaypointforMovebase_ver2();

		//clear Map Service
		void callClearMapService();

		//Not clear Map Service
		void callClear_unknown_spaceService();
	};



	void WP_nav::receiveWaypointforMovebase()
	{
		this->receivedMapPlan = this->nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

		//우선은 목표점을 최종 목적지를 + 시작 포지션 받고
		this->receivedStartPoseFromRVIZ = this->nh.subscribe("/initialpose", 1, &WP_nav::Setinitialpose, this);
		this->receivedPosefromRVIZ = this->nh.subscribe("/move_base_simple/goalfake", 5, &WP_nav::receivedPoseofRVIZ_Callback, this);
		//this->receivedfromAmclPose = this->nh.subscribe("amcl_pose",1, &WP_nav::SetAmclPose, this);


		std::cout << "Start to pick Start Pose and arrived Pose" << std::endl;

		//waiting....
		//rgoal = received goal(counter)
		//cgoal = current goal(live)
		while (this->rgoal > this->cgoal)
		{
			//std::cout << "Listen the WayPoint : " << Current_nWay <<  " th" <<std::endl;
			//ros::Duration(2.0);

			//while (clock() - start < delay);
			//start = clock();
			ros::spinOnce();
		}

		this->rgoal++;
		this->receivedPosefromRVIZ.shutdown();
		this->receivedStartPoseFromRVIZ.shutdown();

		//CostMap Clear
		this->callClearMapService();
		//this->callClear_unknown_spaceService();

		//load map data by executing navigation
		MoveBaseClient ac("move_base", true);

		while (!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		move_base_msgs::MoveBaseGoal temp_goal;

		temp_goal.target_pose.header.frame_id = "map";
		temp_goal.target_pose.header.stamp = ros::Time::now();

		temp_goal.target_pose.pose = this->receivedStartPose.pose;

		ac.sendGoal(temp_goal);

		while (!ac.waitForResult(ros::Duration(2.0))){
			if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
				ROS_INFO("INIT..");
		}

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Complete!! + and start to receive wayPoint");


		//현재 위치와 최종 위치를 아는 상태에서
		//"/move_base/make_plan" Service 사용해서 plan을 받고
		this->receivedMapPlan = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

		nav_msgs::GetPlan receivedPlan;

		receivedPlan.request.start.pose = this->receivedStartPose.pose;
		receivedPlan.request.start.header = this->receivedStartPose.header;

		receivedPlan.request.goal.pose = this->receivedPose.pose;
		receivedPlan.request.goal.header = this->receivedPose.header;

		receivedPlan.request.tolerance = 0.5;

		if (this->receivedMapPlan.call(receivedPlan))
		{
			ROS_INFO("Received Navigation Plan!!");
			ROS_INFO("Plan Size : %d", (int)receivedPlan.response.plan.poses.size());
		}
		else
		{
			ROS_INFO("False Navigation Plan!!");
		}

		//wayPointSampling 하고

		geometry_msgs::PoseStamped wayPoint[5];
		int samplePoint = 5;
		int distance = (int)receivedPlan.response.plan.poses.size() / (samplePoint);

		

		for (int i = 0; i < samplePoint; i++)
		{
			wayPoint[i] = receivedPlan.response.plan.poses[distance * (i + 1)];

			ROS_INFO("Plan number : %d", (int)distance * (i + 1) );

			ROS_INFO("Received %f %f %f", receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.x,
					 receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.y, receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.z);
		}

		//Rviz상으로 표시하기(기존 movebase_goals로 보내어서 rviz로 표시하기)
		ros::Publisher temp_publisher = this->nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goals", 1);

		bool publish = false;
		while (!publish)
		{
			if (temp_publisher.getNumSubscribers() > 0)
			{

				for (int i = 0; i < samplePoint; i++)
				{

					temp_publisher.publish(wayPoint[i]);
					ROS_INFO("Push!!");

					ros::Duration(1.0);
				}

				publish = true;
			}
		}
	}

	void WP_nav::receiveWaypointforMovebase_ver2()
	{
		//for navigation Service
		this->receivedMapPlan = this->nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

		//우선은 목표점을 최종 목적지를 + 시작 포지션 받고
		//this->receivedStartPoseFromRVIZ = this->nh.subscribe("/initialpose", 1, &WP_nav::Setinitialpose, this);
		this->receivedPosefromRVIZ = this->nh.subscribe("/move_base_simple/goalfake", 5, &WP_nav::receivedPoseofRVIZ_Callback, this);
		this->receivedfromAmclPose = this->nh.subscribe("amcl_pose",1, &WP_nav::SetAmclPose, this);


		std::cout << "Start to pick Start Pose and arrived Pose" << std::endl;

		//waiting....
		//rgoal = received goal(counter)
		//cgoal = current goal(live)
		while (this->rgoal > this->cgoal)
		{
			//std::cout << "Listen the WayPoint : " << Current_nWay <<  " th" <<std::endl;
			//ros::Duration(2.0);

			//while (clock() - start < delay);
			//start = clock();
			ros::spinOnce();
		}

		this->rgoal++;
		this->receivedPosefromRVIZ.shutdown();
		this->receivedStartPoseFromRVIZ.shutdown();

		//CostMap Clear
		this->callClearMapService();
		//this->callClear_unknown_spaceService();


		//load map data by executing navigation
		MoveBaseClient ac("move_base", true);

		while (!ac.waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		move_base_msgs::MoveBaseGoal temp_goal;

		temp_goal.target_pose.header.frame_id = "map";
		temp_goal.target_pose.header.stamp = ros::Time::now();

		temp_goal.target_pose.pose = this->receivedStartPose.pose;

		ac.sendGoal(temp_goal);

		while (!ac.waitForResult(ros::Duration(2.0))){
			if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
				ROS_INFO("INIT..");
		}

		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Complete!! + and start to receive wayPoint");


		//현재 위치와 최종 위치를 아는 상태에서
		//"/move_base/make_plan" Service 사용해서 plan을 받고
		this->receivedMapPlan = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

		nav_msgs::GetPlan receivedPlan;

		
		//Set the start pose & the goal pose
		receivedPlan.request.start.pose = this->receivedAmclPose.pose;
		receivedPlan.request.start.header = this->receivedAmclPose.header;
		receivedPlan.request.goal.pose = this->receivedPose.pose;
		receivedPlan.request.goal.header = this->receivedPose.header;


		//Set the error rate
		receivedPlan.request.tolerance = 0.5;


		//Use the Service of Movebase Navigation function
		if (this->receivedMapPlan.call(receivedPlan))
		{
			ROS_INFO("Received Navigation Plan!!");
			ROS_INFO("Plan Size : %d", (int)receivedPlan.response.plan.poses.size());
		}
		else
		{
			ROS_INFO("False Navigation Plan!!");
		}


		//wayPointSampling 하고
		geometry_msgs::PoseStamped wayPoint[5];
		int samplePoint = 5;
		int distance = (int)receivedPlan.response.plan.poses.size() / (samplePoint);
		for (int i = 0; i < samplePoint; i++)
		{
			wayPoint[i] = receivedPlan.response.plan.poses[distance * (i + 1)];

			ROS_INFO("Plan number : %d", (int)distance * (i + 1) );

			ROS_INFO("Received %f %f %f", receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.x,
					 receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.y, receivedPlan.response.plan.poses[distance * (i + 1)].pose.position.z);
		}


		//Rviz상으로 표시하기(기존 movebase_goals(test4 노드 참고)로 보내어서 rviz로 표시하기)
		ros::Publisher temp_publisher = this->nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goals", 1);

		bool publish = false;
		while (!publish)
		{
			if (temp_publisher.getNumSubscribers() > 0)
			{

				for (int i = 0; i < samplePoint; i++)
				{

					temp_publisher.publish(wayPoint[i]);
					ROS_INFO("Push!!");

					ros::Duration(1.0);
				}

				publish = true;
			}
		}
	}




	void WP_nav::callClearMapService()
	{
		this->clearCostMap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

		std_srvs::Empty srv;

		if (this->clearCostMap.call(srv))
		{
			ROS_INFO("clearCostMap!!");
		}
		else
		{
			ROS_INFO("False clearCostMap!!");
		}
	}


	void WP_nav::callClear_unknown_spaceService()
	{
		this->clearCostMap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_unknown_space");

		std_srvs::Empty srv;

		if (this->clearCostMap.call(srv))
		{
			ROS_INFO("call Clear_unknown_spaceService!!");
		}
		else
		{
			ROS_INFO("False Clear_unknown_spaceService!!");
		}
	}

	void WP_nav::receivedPoseofRVIZ_Callback(const geometry_msgs::PoseStamped::ConstPtr &_msg)
	{

		ROS_INFO("Received /movebase_fake");
		this->receivedPose.header = _msg->header;
		this->receivedPose.pose = _msg->pose;

		this->cgoal++;
	}

	void WP_nav::receivedPointArray_Callback(const visualization_msgs::MarkerArrayPtr &_msg)
	{
		ROS_INFO("Received /PointArray");
		ROS_INFO("The number of MarkerArray is %d", (int)_msg->markers.size());
		int arrayNum = _msg->markers.size() - 1;

		this->test[arrayNum].set_Position(_msg->markers[arrayNum].pose.position.x, _msg->markers[arrayNum].pose.position.y);

		this->test[arrayNum].set_Orientation(_msg->markers[arrayNum].pose.orientation.x, _msg->markers[arrayNum].pose.orientation.y,
											 _msg->markers[arrayNum].pose.orientation.z, _msg->markers[arrayNum].pose.orientation.w);

		Current_nWay = _msg->markers.size();
	}

	void WP_nav::sendPoint_forJustNavigation()
	{
		MoveBaseClient ac("move_base", true);

		this->callClearMapService();

		ROS_INFO("Start send Point for Just Navigation, total %d waypoint", nWay);

		for (int i = 0; i < nWay; i++)
		{
			ROS_INFO("GoTo : %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;

			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			goal.target_pose.pose.orientation.x = test[i].msg_robot_model->pose.orientation.x;
			goal.target_pose.pose.orientation.y = test[i].msg_robot_model->pose.orientation.y;
			goal.target_pose.pose.orientation.z = test[i].msg_robot_model->pose.orientation.z;

			goal.target_pose.pose.orientation.w = test[i].msg_robot_model->pose.orientation.w;

			ROS_INFO("Sending  %d'th goal...", i);

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

		ac.~SimpleActionClient();
	}

	void WP_nav::sendPoint_forCycleNavigation()
	{
		MoveBaseClient ac("move_base", true);

		this->callClearMapService();

		ROS_INFO("Start send Point for Cycle Navigation, total %d waypoint", nWay);

		for (int i = 0; i < nWay; i++)
		{
			ROS_INFO("GoTo : %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;

			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			goal.target_pose.pose.orientation.x = test[i].msg_robot_model->pose.orientation.x;
			goal.target_pose.pose.orientation.y = test[i].msg_robot_model->pose.orientation.y;
			goal.target_pose.pose.orientation.z = test[i].msg_robot_model->pose.orientation.z;

			goal.target_pose.pose.orientation.w = test[i].msg_robot_model->pose.orientation.w;

			ROS_INFO("Sending  %d'th goal...", i);

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

		//방향을 반대로 하는 경우를 생각해서 코딩을 해야함....

		for (int i = nWay - 1; i >= 0; i--)
		{
			ROS_INFO("GoTo : Reverse %d'th Waypoint", i);

			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = test[i].msg_robot_model->pose.position.x;

			goal.target_pose.pose.position.y = test[i].msg_robot_model->pose.position.y;
			goal.target_pose.pose.position.z = test[i].msg_robot_model->pose.position.z;

			tf::Quaternion q(test[i].msg_robot_model->pose.orientation.x, test[i].msg_robot_model->pose.orientation.y,
							 test[i].msg_robot_model->pose.orientation.z, test[i].msg_robot_model->pose.orientation.w);

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			if (yaw <= 0)
				yaw = 3.14 + yaw;
			else
				yaw -= 3.14;

			tf::Quaternion new_q(tf::createQuaternionFromRPY(roll, pitch, yaw));

			goal.target_pose.pose.orientation.x = new_q.getX();
			goal.target_pose.pose.orientation.y = new_q.getY();
			goal.target_pose.pose.orientation.z = new_q.getZ();

			goal.target_pose.pose.orientation.w = new_q.getW();

			ROS_INFO("Sending  Reverse %d'th goal...", i);

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

		std::cout << "Congratulation!" << std::endl;

		//ac.~SimpleActionClient();
	}

	void WP_nav::Setinitialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &_msg)
	{

		ROS_INFO("Start initial position x = %f, position y = %f", _msg->pose.pose.position.x, _msg->pose.pose.position.y);

		ROS_INFO("Start initial Orientation : x = %lf, y = %lf, z = %lf, w = %lf", _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z, _msg->pose.pose.orientation.w);

		this->receivedStartPose.header.seq = 0;
		this->receivedStartPose.header.frame_id = "map";
		
		this->receivedStartPose.pose.position.x = _msg->pose.pose.position.x;
		this->receivedStartPose.pose.position.y = _msg->pose.pose.position.y;
		this->receivedStartPose.pose.position.z = _msg->pose.pose.position.z;

		this->receivedStartPose.pose.orientation.x = _msg->pose.pose.orientation.x;
		this->receivedStartPose.pose.orientation.y = _msg->pose.pose.orientation.y;
		this->receivedStartPose.pose.orientation.z = _msg->pose.pose.orientation.z;
		this->receivedStartPose.pose.orientation.w = _msg->pose.pose.orientation.w;
		

		//this->receivedStartPose.pose = _msg->pose;
	}

	void WP_nav::SetAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &_msg){
		ROS_INFO("Start AMCL initial position x = %f, position y = %f", _msg->pose.pose.position.x, _msg->pose.pose.position.y);

		ROS_INFO("Start AMCL initial Orientation : x = %lf, y = %lf, z = %lf, w = %lf", _msg->pose.pose.orientation.x, _msg->pose.pose.orientation.y, _msg->pose.pose.orientation.z, _msg->pose.pose.orientation.w);

		this->receivedAmclPose.header.stamp = ros::Time::now();
		this->receivedAmclPose.header.frame_id = "map";

		this->receivedAmclPose.pose.position.x = _msg->pose.pose.position.x;
		this->receivedAmclPose.pose.position.y = _msg->pose.pose.position.y;
		this->receivedAmclPose.pose.position.z = _msg->pose.pose.position.z;

		this->receivedAmclPose.pose.orientation.x = _msg->pose.pose.orientation.x;
		this->receivedAmclPose.pose.orientation.y = _msg->pose.pose.orientation.y;
		this->receivedAmclPose.pose.orientation.z = _msg->pose.pose.orientation.z;
		this->receivedAmclPose.pose.orientation.w = _msg->pose.pose.orientation.w;


	}

} // namespace WPS
