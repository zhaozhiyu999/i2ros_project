# 🚗 Trajectory Planning Module (2.3)

Author: Tianhao  
Date: 2025  
ROS Version: Noetic (Ubuntu 20.04)

---

##  Overview

This module implements a basic trajectory planner for autonomous driving.  
It generates a dynamic trajectory with position, orientation, velocity, and timing based on current odometry and static waypoints.

---

##  Subscribed Topics

| Topic                     | Type                   | Description              |
|--------------------------|------------------------|--------------------------|
| `/odom`                  | `nav_msgs/Odometry`    | Vehicle position input   |
| `/perception/occupancy_grid` | `nav_msgs/OccupancyGrid` | Static obstacle map (optional) |

---

##  Published Topics

| Topic                   | Type                         | Description                   |
|------------------------|------------------------------|-------------------------------|
| `/planning/trajectory` | `msg_interfaces/Trajectory`  | Planned trajectory to control |

---

##  Message Format (`msg_interfaces/Trajectory`)

```msg
Header header
geometry_msgs/Pose[] poses
float32[] velocities
float32[] timestamps

Each index corresponds to one trajectory point.

## How to Run
        Make sure you have sourced your workspace: source ~/catkin_ws/devel/setup.bash
        Then launch: roslaunch planning planning.launch
        You can check output with: rostopic echo /planning/trajectory

## File Structure
planning/
├── launch/
│   └── planning.launch
├── scripts/
│   └── trajectory_node.py
├── README.md  ← ← ← (YOU ARE HERE)

Status
	•	Waypoint-based trajectory planner
	•	Publishes full Trajectory messages
	•	Not yet integrated with dynamic obstacle avoidance
	•	Not yet verified in final Unity simulation

⸻

Notes
	•	Trajectory points are hardcoded for now
	•	occupancy_grid is subscribed but not yet processed
	•	Recommend integration test with control module

⸻

To Do
	•	Visualize planned path in RViz
	•	Test interaction with /decision/emergency_stop
	•	Fine-tune parameters using real-time odometry