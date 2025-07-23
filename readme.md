# Introduction to ROS – Course Project Autonomous Driving

## Overview 
This repository is part of the "Introduction to ROS" course project **Autonomous Driving**. It consists of a Unity-based driving simulation package and a dummy controller to help you understand how to control the vehicle.

## Project Structure

| Package              | Description                                                                 |
|----------------------|-----------------------------------------------------------------------------|
| `simulation/`        | Unity-based simulation environment with ROS integration (provided by TUM)   |
| `perception/`        | Camera + sensor processing; traffic light detection; occupancy grid mapping |
| `planning/`          | Global path planning and local trajectory generation                        |
| `decision_making/`   | Behavioral logic, red light stop, emergency braking, etc.                   |
| `control/`           | Trajectory tracking and control command generation                          |
| `dummy_controller/`  | Simple controller to demonstrate ROS control messages                       |
| `msg_interfaces/`    | Custom ROS message types (e.g., `Trajectory.msg`)                            |
| `start/launch/`      | Launch files, including `all.launch` to run the complete system             |

---

## System Requirements

- **Operating System**: Ubuntu 20.04 (native or via WSL2)
- **ROS Version**: Noetic
- **Unity Simulator**: Precompiled Unity-based environment (provided)
- **Others**:
  - `catkin` build tools
  - Git + Git LFS

Install Git LFS if not installed:

```bash
sudo apt install git-lfs
git lfs install


⸻

ROS Dependency Installation

Install ROS packages required for this project:

sudo apt update
sudo apt install ros-noetic-octomap ros-noetic-image-common ros-noetic-ackermann-msgs

You may also need Python dependencies (check pip3 list for full requirements):

pip3 install -r requirements.txt  # If available


⸻

Build the Workspace

Assuming your workspace is named i2ros_project/ and all packages are in src/:

cd i2ros_project/
bash src/setup_script.sh      # Makes simulation files executable (optional)
catkin build
source devel/setup.bash

Be sure to add build/, devel/, and log/ to .gitignore.

⸻

How to Run the Project (Multi-Terminal Setup)

The system is designed to run in 6 terminals, each with a specific role.

Terminal 1: Launch Unity Simulation

cd i2ros_project/
source devel/setup.bash
roslaunch simulation simulation.launch

Make sure spawnIndex in Unity is set to 0.

⸻

Terminal 2: Launch Perception Module

cd i2ros_project/
source devel/setup.bash
roslaunch perception perception.launch


⸻

Terminal 3: Launch Decision Module

cd i2ros_project/
source devel/setup.bash
roslaunch decision_making decision.launch


⸻

Terminal 4: Launch Planning Module

cd i2ros_project/
source devel/setup.bash
roslaunch planning planning.launch


⸻

Terminal 5: Launch Control Module

cd i2ros_project/
source devel/setup.bash
roslaunch control control.launch


⸻

Terminal 6: Launch RViz for Visualization

cd i2ros_project/
source devel/setup.bash
rviz


⸻

One-Command Launch (Optional)

You can also launch the full system with a single command:

cd i2ros_project/
source devel/setup.bash
roslaunch start/launch/all.launch

Ensure the Unity simulation is already running before launching all.launch.

⸻

Recommended Folder Structure

i2ros_project/
├── src/
│   ├── simulation/
│   ├── perception/
│   ├── planning/
│   ├── decision_making/
│   ├── control/
│   ├── dummy_controller/
│   ├── msg_interfaces/
│   ├── start/
│   │   └── launch/
│   │       └── all.launch
│   ├── CMakeLists.txt
│   └── setup_script.sh
├── devel/
├── build/
├── logs/
├── .gitignore
└── README.md


⸻

Notes & Tips
	•	Always source devel/setup.bash before launching any node.
	•	Adjust camera parameters or planning goals as needed in your own launch files.
	•	All ROS topics, message types, and node names should match across packages.

⸻

Tools Used
	•	ROS Noetic
	•	Unity 3D
	•	Git LFS
	•	RViz
	•	Python 3 + NumPy + OpenCV

⸻

Contact

For questions or issues, please contact the teaching team or raise an issue in the course forum.

⸻

License

This project is provided for educational purposes within the TUM course “Introduction to ROS”.

---