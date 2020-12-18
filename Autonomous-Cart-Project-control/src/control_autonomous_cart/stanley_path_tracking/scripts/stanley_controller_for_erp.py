#!/usr/bin/python
# -*- coding: utf-8 -*-
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import rospy
import numpy as np
import matplotlib.pyplot as plt
import threading

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# from can_msgs.msg import Frame
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import sys
sys.path.append(".")
try:
    import cubic_spline_planner
except:
    raise

k = 0.5  # control gain
Kp = 1  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.9  # [m] Wheel base of vehicle
max_steer = np.radians(28.0)  # [rad] max steering angle

show_animation = False

# cx = []
# cy = []
# cyaw = []

isSubscribePath = False
isSubscribePose = False
isSubscribeSpeed = False

g_current_path = Path()
g_current_pose = PoseWithCovarianceStamped()
g_current_speed = UInt16()

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, steering_angle = 0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.steering_angle = steering_angle
        # self.steering_angle_rate = 0.0
        self.steering_angle_Kp = 1/0.4
        self.k_soft = 1

    def update(self, x, y, yaw, speed):
        """
        Update the state of the vehicle.

        데이터 수신해서 업데이트
        """
        # delta = np.clip(delta, -max_steer, max_steer)
        # self.steering_angle += self.steering_angle_update(delta) * dt
        # print(np.rad2deg(delta))
        self.x = x
        self.y = y
        self.yaw = yaw
        self.yaw = normalize_angle(self.yaw)
        self.v = speed

    
    # def update(self, acceleration, delta):
    #     """
    #     Update the state of the vehicle.

    #     Stanley Control uses bicycle model.

    #     :param acceleration: (float) Acceleration
    #     :param delta: (float) Steering
    #     """
    #     delta = np.clip(delta, -max_steer, max_steer)
    #     # self.steering_angle += self.steering_angle_update(delta) * dt
    #     # print(np.rad2deg(delta))
    #     self.x += self.v * np.cos(self.yaw) * dt
    #     self.y += self.v * np.sin(self.yaw) * dt
    #     self.yaw += self.v / L * np.sin(delta) * dt
    #     self.yaw = normalize_angle(self.yaw)
    #     self.v += acceleration * dt

    def steering_angle_update(self, target_delta):
        return self.steering_angle_Kp * (target_delta - self.steering_angle)


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    
    theta_d = np.arctan2(k * error_front_axle, state.k_soft + state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    # print(fx)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    # print(target_idx)
    # print(len(cx))

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

# class path_converter():
#     def __init__():
#         pass
#     def 

def path_callback(msg):
    """
    msg : Path()
    """
    global isSubscribePath
    global g_current_path
    
    g_current_path = msg

    if not isSubscribePath:
        isSubscribePath = True

def pose_callback(msg):
    """
    msg : PoseWithCovarianceStamped()
    """
    global isSubscribePose
    global g_current_pose

    g_current_pose = msg

    if not isSubscribePose:
        isSubscribePose = True


def speed_callback(msg):
    """
    msg : UInt16()
    """
    global isSubscribeSpeed
    global g_current_speed

    g_current_speed = msg

    if not isSubscribeSpeed:
        isSubscribeSpeed = True

def path_converter():

    temp_current_path = g_current_path
    rx = []
    ry = []
    ryaw = []
    # print(len(msg.poses))
    for i in range(0, len(temp_current_path.poses)):
        rx.append(temp_current_path.poses[i].pose.position.x)
        ry.append(temp_current_path.poses[i].pose.position.y)

        quaternion = np.empty((4, ), dtype=np.float64)
        quaternion[0] = temp_current_path.poses[i].pose.orientation.x
        quaternion[1] = temp_current_path.poses[i].pose.orientation.y
        quaternion[2] = temp_current_path.poses[i].pose.orientation.z
        quaternion[3] = temp_current_path.poses[i].pose.orientation.w

        euler_angle = euler_from_quaternion(quaternion)
        ryaw.append(normalize_angle(euler_angle[2]))
    
    return rx,ry,ryaw

def pose_converter():

    cx = g_current_pose.pose.pose.position.x
    cy = g_current_pose.pose.pose.position.y

    quaternion = np.empty((4, ), dtype=np.float64)
    quaternion[0] = g_current_pose.pose.pose.orientation.x
    quaternion[1] = g_current_pose.pose.pose.orientation.y
    quaternion[2] = g_current_pose.pose.pose.orientation.z
    quaternion[3] = g_current_pose.pose.pose.orientation.w

    euler_angle = euler_from_quaternion(quaternion)

    cyaw = euler_angle[2]

    return cx,cy,cyaw
    
def kph_to_mps(speed):
    return speed / 3.6

def speed_converter():
    
    current_speed = kph_to_mps((g_current_speed.data / 10))
    return current_speed

# def assign_global_variable()
#     rate10 = rospy.Rate(10)
#     global cx, cy, cyaw
#     while not rospy.is_shutdown():
#         cx, cy, cyaw = path_converter()



def odom_publish(state, odom_pub):
    """
    state : State()
    """
    rate10 = rospy.Rate(10)

    while not rospy.is_shutdown():
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'vehicle'
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0
        yaw_qt = quaternion_from_euler(0,0,state.yaw)

        odom.pose.pose.orientation.x = yaw_qt[0]
        odom.pose.pose.orientation.y = yaw_qt[1]
        odom.pose.pose.orientation.z = yaw_qt[2]
        odom.pose.pose.orientation.w = yaw_qt[3]
        odom.twist.twist.linear.x = state.v
        odom_pub.publish(odom)


        rate10.sleep()    

def target_steering_angle_converter_from_float_to_UINT16(target_steering_angle):
    return np.round(target_steering_angle * (180/np.pi) * 71) + 2000

def main():
    """Plot an example of Stanley steering control on a cubic spline."""

    rospy.init_node("stanley_controller", anonymous=False)

    target_steering_pub = rospy.Publisher('control/target_steering_angle', UInt16, queue_size=10)
    target_speed_pub = rospy.Publisher('control/target_speed', UInt16, queue_size=10)
    path_sub = rospy.Subscriber('path', Path, path_callback)
    pose_sub = rospy.Subscriber('pose', PoseWithCovarianceStamped, pose_callback)
    speed_sub = rospy.Subscriber('speed_r', UInt16, speed_callback)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size= 10)

    while not (isSubscribePath and isSubscribePose and isSubscribeSpeed):
        rate1 = rospy.Rate(1)
        print('waiting for subscribing all...')
        print("isSubscribePath", isSubscribePath)
        print("isSubscribePose", isSubscribePose)
        print("isSubscribeSpeed", isSubscribeSpeed)
        rate1.sleep()
    # ax = [0.0, 100.0, 100.0, 50.0, 60.0, 60.0, 100.0]
    # ay = [0.0, 0.0, -30.0, -20.0, 0.0, -10.0,  -20.0]
    # cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    #     ax, ay, ds=0.1)
    # while 
    # t = threading.Thread(target=path_publish, args=(cx, cy, cyaw, path_pub))
    # t.start()
    cx,cy,cyaw = path_converter()
    
    # t1 = threading.Thread(target=assign_global_variable, args=())
    # t1.start()
    # print(len(cx),len(cy),len(cyaw),len(g_current_path.poses))

    target_speed = 30 / 3.6  # [m/s]
    
    # Initial state
    state = State(x=-10.0, y=20.0, yaw=np.radians(20.0), v=0.0)

    t2 = threading.Thread(target=odom_publish, args=(state, odom_pub))
    t2.daemon = True
    t2.start()

    last_idx = len(cx) - 1 # 이부분 나중에 수정해야할 것 같다. 아마 영역으로 해야 할듯
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while (last_idx > target_idx) and (not rospy.is_shutdown()):
        cx,cy,cyaw = path_converter()
        current_x, current_y, current_yaw = pose_converter()
        current_speed = speed_converter()
        # print(len(cx),len(cy),len(cyaw),len(g_current_path.poses))

        print("current_x", current_x)
        print("current_y", current_y)
        print("current_yaw", current_yaw)

        state.update(current_x, current_y, current_yaw, current_speed)

        print("state_x", state.x)
        print("state_y", state.y)
        print("state_yaw", state.yaw)

        last_idx = len(cx) - 1
        # ai = pid_control(target_speed, state.v)

        print("target_idx]", target_idx)

        print("cx[target_idx]", cx[target_idx])
        print("cy[target_idx]", cy[target_idx])
        print("cyaw[target_idx]", cyaw[target_idx])

        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        target_steering_angle_UINT16 = UInt16()
        # print((target_steering_angle_converter_from_float_to_UINT16(np.clip(di, -max_steer, max_steer))))
        target_steering_angle_UINT16.data = int(target_steering_angle_converter_from_float_to_UINT16(np.clip(di, -max_steer, max_steer)))
        target_steering_pub.publish( target_steering_angle_UINT16 )

        print("di", di)
        print("di[deg]", di * 180/np.pi)

        # rospy.loginfo(np.clip(di, -max_steer, max_steer)) 

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        # plt.pause(0.1)
        rospy.Rate(10).sleep()

    target_steering_pub.publish(0 + 2000)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    rospy.spin()


if __name__ == '__main__':
    main()
