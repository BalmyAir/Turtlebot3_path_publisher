#!/usr/bin/python
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import rospy
import numpy as np
# import matplotlib.pyplot as plt
import threading
import copy
import math
# import Queue

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt16
# from tf.

import sys
sys.path.append(".")
try:
    import cubic_spline_planner
except:
    raise

def normalized(quaternion):
    norm = math.sqrt(quaternion.x ** 2 +  
                     quaternion.y ** 2 + 
                     quaternion.z ** 2 + 
                     quaternion.w ** 2 ) 
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return quaternion


pose_idx = 1
def publish_pose(path, past_time, pose_pub):
    global pose_idx
    
    elasped_time = rospy.Time.now().to_time() - past_time
    # print("pose_idx : {}".format(pose_idx))
    # print("elasped_time : {}".format(elasped_time))
    if elasped_time >= float(pose_idx * 0.00001):
        # print("if pose_idx : {}".format(pose_idx))
        # print("if elasped_time : {}".format(elasped_time))
        poseWithCovarianceStamped = PoseWithCovarianceStamped()
        poseWithCovarianceStamped.header.frame_id = "pose"
        poseWithCovarianceStamped.header.stamp = rospy.Time.now()
        poseWithCovarianceStamped.pose.pose = copy.deepcopy(path.poses[pose_idx-1].pose)
        mu = 0
        position_sigma = 0.1
        orientation_sigma = 0.001
        poseWithCovarianceStamped.pose.pose.position.x += np.random.normal(mu, position_sigma, size=1)
        poseWithCovarianceStamped.pose.pose.position.y += np.random.normal(mu, position_sigma, size=1)
        poseWithCovarianceStamped.pose.pose.orientation.z += np.random.normal(mu, orientation_sigma, size=1)
        poseWithCovarianceStamped.pose.pose.orientation.w += np.random.normal(mu, orientation_sigma, size=1)
        
        poseWithCovarianceStamped.pose.pose.orientation = normalized(poseWithCovarianceStamped.pose.pose.orientation)
        pose_idx += 1
        if pose_idx >= len(path.poses):
            pose_idx = len(path.poses) -1
        pose_pub.publish(poseWithCovarianceStamped)
    

def publish_all(path_pub, pose_pub, speed_pub):
    """
    path_pub = path_publisher, rospy.Publisher()
    pose_pub
    """
    rate10 = rospy.Rate(10)
    rate100 = rospy.Rate(100)
    past_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        
        cx, cy, cyaw = generate_path(past_time)

        path = Path()
        path.header.frame_id = 'cubic_spline_path'

        for i in range(0,len(cx)):
            poseStamped = PoseStamped()
            poseStamped.header = path.header
            # poseStamped.header.seq = i
            poseStamped.pose.position.x = cx[i]
            poseStamped.pose.position.y = cy[i]
            poseStamped.pose.position.z = 0
            # yaw_quat = numpy.empty((4, ), dtype=numpy.float64
            yaw_quat = quaternion_from_euler(0,0,cyaw[i])

            poseStamped.pose.orientation.x = yaw_quat[0]
            poseStamped.pose.orientation.y = yaw_quat[1]
            poseStamped.pose.orientation.z = yaw_quat[2]
            poseStamped.pose.orientation.w = yaw_quat[3]
            path.poses.append(poseStamped)

        path.header.stamp = rospy.Time.now()

        publish_pose(path, past_time, pose_pub=pose_pub)

        path_pub.publish(path)

        speed = UInt16()
        speed.data = 50
        speed_pub.publish(speed)

        rate100.sleep()

def generate_path(past_time):
    """
    past_time : float
    """
    # while not rospy.is_shutdown():
    
    elasped_time = rospy.Time.now().to_time() - past_time
    # print('time is {0}'.format(elasped_time))
    first_changing_time = 10000 + 0
    second_changing_time = 5 + first_changing_time
    third_changing_time = 5 + second_changing_time
    # fourth_changing_time = 4*first_changing_time
    if elasped_time <= first_changing_time:
        # ax = [0.0, 100.0, 100.0, 50.0, 60.0, 60.0, 100.0]
        # ay = [0.0, 0.0, -30.0, -20.0, 0.0, -10.0,  -20.0]
        ax = [0.0, 100.0, 100.0, 50.0, 60.0, 100.0]
        ay = [0.0, 0.0, -30.0, -20.0, 0.0,  -20.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)
    elif first_changing_time < elasped_time <= second_changing_time:
        ax = [0.0, 120.0, 100.0, 40.0, 60.0, 90.0]
        ay = [0.0, 0.0, -30.0, -20.0, 0.0, -30.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)
    elif second_changing_time < elasped_time <= third_changing_time:
        ax = [0.0, 110.0, 100.0, 30.0, 50.0, 70.0]
        ay = [0.0, 0.0, -20.0, -10.0, 10.0, -30.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)
    else: #elasped_time >= 40.0:
        ax = [0.0, 160.0, 120.0, 30.0, 40.0, 60.0]
        ay = [0.0, 0.0, -20.0, -10.0, 0.0, -10.0]
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.3)

    return cx,cy,cyaw

def main():
    """Publish a cubic spline."""
    rospy.init_node("path_and_pose_publisher", anonymous=False)
    path_pub = rospy.Publisher('path', Path, queue_size= 10)
    pose_pub = rospy.Publisher('pose', PoseWithCovarianceStamped, queue_size=10)
    speed_pub = rospy.Publisher('speed_r', UInt16, queue_size=10)

    publish_all(path_pub, pose_pub, speed_pub)
    
    # t_generate_path = threading.Thread(target=generate_path, args=())
    # t = threading.Thread(target=publish_all, args=(path_pub,))
    # t.daemon = True
    # t_generate_path.start()
    # t.start()
    # t_generate_path.join()
    # t.join()

if __name__ == '__main__':
    main()
