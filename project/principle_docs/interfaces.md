# 接口规范文档（ROS Topic / Service）

## 📦 感知模块（perception）

| 接口名 | 类型 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|------|-----------|---------|---------|------|
| `/perception/occupancy_grid` | topic | `nav_msgs/OccupancyGrid` | perception | planning | 环境地图 |
| `/perception/traffic_light_status` | topic | `std_msgs/String` | perception | decision_making | 红绿灯状态："red"/"green" |

---

## 📦 路径规划模块（planning）

| 接口名 | 类型 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|------|-----------|---------|---------|------|
| `/planning/trajectory` | topic | `msg_interfaces/Trajectory` | planning | control | 轨迹序列（包含多个 pose + 时间 + 速度） |

---

## 📦 决策模块（decision_making）

| 接口名 | 类型 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|------|-----------|---------|---------|------|
| `/decision/emergency_stop` | topic | `std_msgs/Bool` | decision_making | control | 是否紧急刹车 |
| `/decision/next_goal_pose` | service | `geometry_msgs/Pose`（或自定义） | decision_making | planning | 决策向规划请求下一目标位姿（可选） |

---

## 📦 控制模块（control）

| 接口名 | 类型 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|------|-----------|---------|---------|------|
| `/vehicle/cmd` | topic | `geometry_msgs/Twist` 或 `ackermann_msgs/AckermannDriveStamped` | control | simulation | 车辆控制指令发送至仿真 |

