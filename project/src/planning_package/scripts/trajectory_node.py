#!/usr/bin/env python3
# coding: utf-8

import rospy
import math
import tf
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from msg_interfaces.msg import Trajectory
from std_msgs.msg import Header

# 当前车辆状态
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

goal_index = 0

# 预设路径点
waypoints = [
    (0.0, 0.0),
    (3.0, 1.0),
    (6.0, 2.0),
    (9.0, 3.0),
    (12.0, 3.5)
]

def odom_callback(msg):
    global current_x, current_y, current_yaw
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    current_x = position.x
    current_y = position.y

    # 四元数 -> yaw
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    current_yaw = yaw

def occupancy_callback(msg):
    # 可用于扩展避障
    pass

def get_next_waypoints():
    """选取当前位置之后的3个目标点"""
    global goal_index
    lookahead = 3
    for i in range(goal_index, len(waypoints)):
        dist = math.hypot(waypoints[i][0] - current_x, waypoints[i][1] - current_y)
        if dist > 1.0:
            goal_index = i
            break
    return waypoints[goal_index:goal_index+lookahead]

def generate_trajectory():
    traj = Trajectory()
    traj.header = Header()
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = "map"  # 依据你们使用的tf坐标系

    poses = []
    velocities = []
    timestamps = []

    current_time = 0.0
    for i, (x, y) in enumerate(get_next_waypoints()):
        pose = Pose()
        pose.position = Point(x, y, 0.0)

        # 朝向：目标点与当前位置连线方向
        yaw = math.atan2(y - current_y, x - current_x)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation = Quaternion(*quat)

        poses.append(pose)
        velocities.append(1.5)       # 固定目标速度
        timestamps.append(current_time)
        current_time += 1.0          # 每秒一个点，可调节

    traj.poses = poses
    traj.velocities = velocities
    traj.timestamps = timestamps

    return traj

def main():
    rospy.init_node("trajectory_planner")
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/perception/occupancy_grid", OccupancyGrid, occupancy_callback)

    traj_pub = rospy.Publisher("/planning/trajectory", Trajectory, queue_size=10)

    rate = rospy.Rate(5)  # 发布频率 5Hz
    while not rospy.is_shutdown():
        traj = generate_trajectory()
        traj_pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass