# 🛣️ Autonomous Driving Project – ROS (TUM AAS 2025)

This is the official project repository for the TUM AAS "Introduction to ROS" summer term 2025 group project.

The goal is to control an autonomous vehicle in a simulated urban environment using ROS and Unity. The vehicle must follow a track, obey traffic lights, avoid collisions, and stop when necessary. This system is built upon ROS and interacts with a Unity-based simulator via a provided ROS-Unity bridge.

---

## 📦 Package Structure

| Package | Description |
|---------|-------------|
| `simulation/`         | Provided ROS-Unity bridge and simulator interface (do not modify core logic) |
| `perception/`         | Processes camera and sensor input; builds occupancy grid; detects traffic lights |
| `planning/`           | Selects short-term goals, generates global path and local trajectory |
| `decision_making/`    | Coordinates behavior (e.g. red light stop, merging vehicle, emergency stop) |
| `control/`            | Executes trajectory tracking and publishes velocity/steering commands |
| `msg_interfaces/`     | Contains custom ROS message definitions (e.g. `Trajectory.msg`) |
| `launch/`             | Master launch file to start the full system (`all.launch`) |

---

## 🚀 Getting Started

### ✅ Install ROS Dependencies

Make sure you're using **Ubuntu 20.04 + ROS Noetic**.

```bash
sudo apt update
sudo apt install ros-noetic-octomap ros-noetic-image-common ros-noetic-ackermann-msgs
