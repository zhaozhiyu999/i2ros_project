# Autonomous Driving Project – ROS @ TUM AAS 2025

This repository contains the full implementation of our team project for the "Introduction to ROS" course (TUM, Summer Term 2025). The system integrates perception, planning, decision-making, and control modules to drive a virtual car in Unity using ROS Noetic.

---
## Authors （Group7）
##### Youran Wang, Contact: go73hay@mytum.de
##### Zhiyu Zhao，Zhiyu.zhao@tum.de
##### Xu Yihang, go57bup@mytum.de
##### Hao Tian, Contact: go49pel@mytum.de
##### Yiping Zhou, Contact: go76may@mytum.de


Summer Term 2025
TUM – Introduction to ROS Course



## System Architecture

The project follows a modular architecture:

| Module             | Description |
|--------------------|-------------|
| `simulation/`      | Unity-based simulator bridge (provided by instructor) |
| `perception/`      | Processes camera/LiDAR input, builds occupancy grid, detects traffic lights |
| `planning_package/`        | Performs trajectory planning using DWA, based on perception input |
| `decision_making/` | Determines behavior like red-light stops, merging, and emergency braking |
| `control_package/`         | Converts trajectory into steering/throttle commands |
| `msg_interfaces/`  | Custom message definitions (e.g., `Trajectory.msg`) |
| `launch/`          | Contains `all.launch` to start the entire system |

---

## System Requirements

- **OS**: Ubuntu 20.04
- **ROS Version**: ROS Noetic Ninjemys
- **Other Tools**:
  - `git`, `catkin`, `roscore`, `rviz`
  - `git-lfs` (required to pull Unity binaries)
  - Python 3 & pip3

Dependency List

Run the following to check your installed Python libraries:
```bash
pip3 list
```
For reproducibility, you may also export the list:
```bash
pip3 freeze > requirements.txt
```

Install Git LFS:

```bash
sudo apt install git-lfs
git lfs install
```
Install required ROS packages:
```bash
sudo apt update
sudo apt install ros-noetic-octomap ros-noetic-image-common ros-noetic-ackermann-msgs
```


## Directory Structure
```
i2ros_project/
├── project/
│   ├── src/
│   │   ├── perception/
│   │   ├── planning_package/
│   │   ├── control_package/
│   │   ├── decision_making/
│   │   ├── msg_interfaces/
│   ├── launch/
│   │   └── all.launch
│   ├── setup_script.sh
│   └── README.md   <-- this file
├── build/        # ignored
├── devel/        # ignored
└── logs/         # ignored
```
.gitignore ensures build/, devel/, and logs/ are not committed.


#### Setup & Build Instructions

```bash
# Create a catkin workspace if it doesn't exist
mkdir -p ~/catkin_ws/src
```
Clone and Setup
```bash
cd ~/catkin_ws/src
git clone git@gitlab.lrz.de:zhiyu_zhao/i2ros_project.git i2ros_project
cd i2ros_project/project
bash project/setup_script.sh
```
Build the Workspace
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```


#### Running the Full System

You can launch everything using:
```bash
source devel/setup.bash
roslaunch launch all.launch
```
This starts Unity simulator, perception, decision_making, planning, control, and RViz in one go.

### Custom Messages

All custom messages are defined in msg_interfaces/:
	•	Trajectory.msg: Includes poses, velocities, timestamps
	•	All modules using custom interfaces should depend on msg_interfaces




Development Tips
	•	Always source devel/setup.bash before running any ROS node.
	•	Use catkin build instead of catkin_make for improved control and log clarity.
	•	Follow naming conventions defined in principle_docs/.



## License

This project is part of a university course and follows academic fair use policies. Do not redistribute outside course scope.



---