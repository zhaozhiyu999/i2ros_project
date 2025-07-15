#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>  // 新增：红绿灯状态消息
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <msg_interfaces/Trajectory.h>
#include <cmath>
#include <vector>
#include <algorithm>

struct PathPoint {
    double x, y, yaw;
    bool is_safe;
};

class PathPlannerNode {
public:
    PathPlannerNode(ros::NodeHandle& nh) : tf_listener_(tf_buffer_) {
        // ========== 设置静态TF变换 ==========
        setupStaticTransforms();
        
        // ========== 订阅感知模块信息 ==========
        map_sub_ = nh.subscribe("/perception/occupancy_grid", 1, 
                               &PathPlannerNode::mapCallback, this);
        front_hazard_sub_ = nh.subscribe("/perception/front_hazard", 1, 
                                        &PathPlannerNode::frontHazardCallback, this);
        
        // 新增：订阅红绿灯状态话题
        traffic_light_sub_ = nh.subscribe("/perception/traffic_light_status", 1, 
                                         &PathPlannerNode::trafficLightCallback, this);
        
        // ========== 发布路径轨迹和简单可视化 ==========
        trajectory_pub_ = nh.advertise<msg_interfaces::Trajectory>("/planning/trajectory", 10);
        smooth_path_pub_ = nh.advertise<nav_msgs::Path>("/planning/smooth_path", 10);
        path_line_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/path_line", 10);
        
        // ========== 参数设置 ==========
        nh.param("planning_frequency", planning_frequency_, 10.0);
        nh.param("default_speed", default_speed_, 2.5);
        nh.param("trajectory_points", trajectory_points_, 6);  // 减少轨迹点
        nh.param("lookahead_distance", lookahead_distance_, 8.0);  // 减少前瞻距离
        nh.param("vehicle_width", vehicle_width_, 1.8);  // 稍微减小车辆宽度
        nh.param("safety_margin", safety_margin_, 0.3);  // 减小安全边距
        nh.param("avoidance_distance", avoidance_distance_, 2.5);  // 减小避障距离
        nh.param("obstacle_check_distance", obstacle_check_distance_, 6.0);  // 减小检测距离
        nh.param("path_interpolation_resolution", path_interpolation_resolution_, 0.3);  // 路径插值分辨率
        
        // ========== 状态初始化 ==========
        has_map_ = false;
        front_hazard_ = false;
        red_light_detected_ = false;  // 新增：红绿灯状态变量
        obstacle_detected_ = false;
        
        // ========== 预定义赛道路径点 ==========
        setupTrackWaypoints();
        current_waypoint_index_ = 0;
        
        // ========== 启动规划循环 ==========
        planning_timer_ = nh.createTimer(ros::Duration(1.0 / planning_frequency_), 
                                       &PathPlannerNode::planningLoop, this);
        
        ROS_INFO("Path Planner Node started with traffic light integration");
        ROS_INFO("Planning frequency: %.1f Hz, Default speed: %.1f m/s", 
                 planning_frequency_, default_speed_);
        ROS_INFO("Vehicle width: %.1f m, Safety margin: %.1f m", 
                 vehicle_width_, safety_margin_);
    }

private:
    // ========== ROS相关 ==========
    ros::Subscriber map_sub_, front_hazard_sub_, traffic_light_sub_;  // 新增traffic_light_sub_
    ros::Publisher trajectory_pub_, smooth_path_pub_, path_line_pub_;
    ros::Timer planning_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    // ========== 感知信息 ==========
    nav_msgs::OccupancyGrid current_map_;
    bool has_map_;
    bool front_hazard_;
    bool red_light_detected_;  // 新增：红绿灯状态变量
    bool obstacle_detected_;
    geometry_msgs::Point nearest_obstacle_;
    
    // ========== 路径规划参数 ==========
    double planning_frequency_;
    double default_speed_;
    int trajectory_points_;
    double lookahead_distance_;
    double vehicle_width_;
    double safety_margin_;
    double avoidance_distance_;
    double obstacle_check_distance_;
    double path_interpolation_resolution_;
    
    // ========== 赛道路径点 ==========
    std::vector<geometry_msgs::Pose> track_waypoints_;
    int current_waypoint_index_;

    // ========== TF设置 ==========
    void setupStaticTransforms() {
        std::vector<geometry_msgs::TransformStamped> static_transforms;
        
        // map -> base_link
        geometry_msgs::TransformStamped map_to_base;
        map_to_base.header.stamp = ros::Time::now();
        map_to_base.header.frame_id = "map";
        map_to_base.child_frame_id = "base_link";
        map_to_base.transform.translation.x = 0.0;
        map_to_base.transform.translation.y = 0.0;
        map_to_base.transform.translation.z = 0.0;
        map_to_base.transform.rotation.w = 1.0;
        static_transforms.push_back(map_to_base);
        
        // base_link -> OurCar/Sensors/INS
        geometry_msgs::TransformStamped base_to_ins;
        base_to_ins.header.stamp = ros::Time::now();
        base_to_ins.header.frame_id = "base_link";
        base_to_ins.child_frame_id = "OurCar/Sensors/INS";
        base_to_ins.transform.translation.x = 0.0;
        base_to_ins.transform.translation.y = 0.0;
        base_to_ins.transform.translation.z = 0.0;
        base_to_ins.transform.rotation.w = 1.0;
        static_transforms.push_back(base_to_ins);
        
        // base_link -> DepthCamera
        geometry_msgs::TransformStamped base_to_depth;
        base_to_depth.header.stamp = ros::Time::now();
        base_to_depth.header.frame_id = "base_link";
        base_to_depth.child_frame_id = "OurCar/Sensors/DepthCamera";
        base_to_depth.transform.translation.x = 2.0;
        base_to_depth.transform.translation.y = 0.0;
        base_to_depth.transform.translation.z = 1.5;
        base_to_depth.transform.rotation.w = 1.0;
        static_transforms.push_back(base_to_depth);
        
        static_tf_broadcaster_.sendTransform(static_transforms);
        ROS_INFO("Published static TF transforms");
    }

    // ========== 感知回调函数 ==========
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        current_map_ = *msg;
        has_map_ = true;
        ROS_INFO_ONCE("Received occupancy grid map for obstacle detection");
    }
    
    void frontHazardCallback(const std_msgs::Bool::ConstPtr& msg) {
        bool prev_hazard = front_hazard_;
        front_hazard_ = msg->data;
        
        if (prev_hazard != front_hazard_) {
            if (front_hazard_) {
                ROS_WARN("FRONT HAZARD DETECTED - Switching to avoidance mode");
            } else {
                ROS_INFO("Front hazard cleared - Resuming normal path planning");
            }
        }
    }
    
    // 新增：红绿灯状态回调函数
    void trafficLightCallback(const std_msgs::String::ConstPtr& msg) {
        bool prev_red_light = red_light_detected_;
        red_light_detected_ = (msg->data == "red");
        
        if (prev_red_light != red_light_detected_) {
            if (red_light_detected_) {
                ROS_WARN("RED LIGHT DETECTED - Planning stopping trajectory");
            } else {
                ROS_INFO("Traffic light is GREEN - Resuming normal planning");
            }
        }
    }
    
    // ========== 主规划循环 ==========
    void planningLoop(const ros::TimerEvent& event) {
        // 1. 获取当前车辆位置
        geometry_msgs::TransformStamped current_pose;
        if (!getCurrentPose(current_pose)) {
            ROS_WARN_THROTTLE(2.0, "Cannot get current pose, using default trajectory");
            publishDefaultTrajectory();
            return;
        }
        
        // 2. 检测前方障碍物
        detectObstaclesInPath(current_pose);
        
        // 3. 更新目标路径点
        updateCurrentWaypoint(current_pose);
        
        // 4. 生成路径（考虑红绿灯、前方危险和避障）
        std::vector<PathPoint> path = generatePath(current_pose);
        
        // 5. 发布轨迹和简单可视化
        publishTrajectory(path);
        publishSmoothPathVisualization(path);
        
        // 6. 调试信息
        std::string planning_mode = "NORMAL";
        if (red_light_detected_) {
            planning_mode = "RED_LIGHT_STOP";
        } else if (front_hazard_) {
            planning_mode = "HAZARD_AVOIDANCE";
        }
        
        ROS_INFO_THROTTLE(1.0, "Planning: Waypoint %d/%lu, Red Light: %s, Front Hazard: %s, Mode: %s, Path points: %lu", 
                         current_waypoint_index_, track_waypoints_.size(),
                         red_light_detected_ ? "TRUE" : "FALSE",
                         front_hazard_ ? "TRUE" : "FALSE",
                         planning_mode.c_str(),
                         path.size());
    }
    
    // ========== 位置获取 ==========
    bool getCurrentPose(geometry_msgs::TransformStamped& pose) {
        std::vector<std::string> possible_frames = {"base_link", "OurCar/Sensors/INS"};
        
        for (const auto& frame : possible_frames) {
            try {
                pose = tf_buffer_.lookupTransform("map", frame, ros::Time(0), ros::Duration(0.1));
                return true;
            } catch (tf2::TransformException& ex) {
                continue;
            }
        }
        return false;
    }
    
    // ========== 障碍物检测 ==========
    void detectObstaclesInPath(const geometry_msgs::TransformStamped& current_pose) {
        obstacle_detected_ = false;
        
        if (!has_map_) {
            return;
        }
        
        // 只有在前方危险信号为 true 时才需要检测障碍物位置用于可视化
        if (!front_hazard_) {
            return;  // 正常情况下不进行障碍物检测
        }
        
        double current_x = current_pose.transform.translation.x;
        double current_y = current_pose.transform.translation.y;
        
        // 获取当前朝向
        tf2::Quaternion q(current_pose.transform.rotation.x,
                         current_pose.transform.rotation.y,
                         current_pose.transform.rotation.z,
                         current_pose.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 检查前方路径是否有障碍物（用于可视化）
        double step_size = 0.5;
        for (double dist = 1.0; dist <= obstacle_check_distance_; dist += step_size) {
            // 检查车辆宽度范围内的点
            for (double lateral_offset = -vehicle_width_/2 - safety_margin_; 
                 lateral_offset <= vehicle_width_/2 + safety_margin_; 
                 lateral_offset += 0.3) {
                
                double check_x = current_x + dist * cos(yaw) + lateral_offset * cos(yaw + M_PI/2);
                double check_y = current_y + dist * sin(yaw) + lateral_offset * sin(yaw + M_PI/2);
                
                if (isPointOccupied(check_x, check_y)) {
                    obstacle_detected_ = true;
                    nearest_obstacle_.x = check_x;
                    nearest_obstacle_.y = check_y;
                    nearest_obstacle_.z = 0.0;
                    
                    ROS_WARN("Obstacle detected at (%.2f, %.2f), distance: %.2f m", 
                             check_x, check_y, dist);
                    return;
                }
            }
        }
    }
    
    // ========== 检查点是否被占用 ==========
    bool isPointOccupied(double x, double y) {
        if (!has_map_) return false;
        
        int grid_x = (x - current_map_.info.origin.position.x) / current_map_.info.resolution;
        int grid_y = (y - current_map_.info.origin.position.y) / current_map_.info.resolution;
        
        if (grid_x < 0 || grid_x >= current_map_.info.width || 
            grid_y < 0 || grid_y >= current_map_.info.height) {
            return false;  // 边界外认为是自由空间，不是障碍物
        }
        
        int index = grid_y * current_map_.info.width + grid_x;
        if (index >= current_map_.data.size()) return false;
        
        // 提高占用阈值，只有很确定的障碍物才认为是占用
        return current_map_.data[index] > 70;  // 从50提高到70
    }
    
    // ========== 赛道路径点设置 ==========
    void setupTrackWaypoints() {
        track_waypoints_.clear();
        
        // 基于8字形赛道的示例路径点
        // 实际使用时应根据真实赛道调整
        
        // 直线段
        for (int i = 0; i < 20; ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = i * 2.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            track_waypoints_.push_back(pose);
        }
        
        ROS_INFO("Setup %lu track waypoints", track_waypoints_.size());
    }
    
    // ========== 更新当前目标路径点 ==========
    void updateCurrentWaypoint(const geometry_msgs::TransformStamped& current_pose) {
        if (track_waypoints_.empty()) return;
        
        double current_x = current_pose.transform.translation.x;
        double current_y = current_pose.transform.translation.y;
        
        // 寻找最近的未到达路径点
        for (int i = current_waypoint_index_; i < track_waypoints_.size(); ++i) {
            double dx = track_waypoints_[i].position.x - current_x;
            double dy = track_waypoints_[i].position.y - current_y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance < 3.0 && i == current_waypoint_index_) {
                current_waypoint_index_ = std::min(i + 1, (int)track_waypoints_.size() - 1);
                ROS_INFO("Advanced to waypoint %d", current_waypoint_index_);
                break;
            }
        }
    }
    
    // ========== 生成路径（新增红绿灯逻辑） ==========
    std::vector<PathPoint> generatePath(const geometry_msgs::TransformStamped& current_pose) {
        std::vector<PathPoint> path;
        
        double current_x = current_pose.transform.translation.x;
        double current_y = current_pose.transform.translation.y;
        
        // 获取当前朝向
        tf2::Quaternion q(current_pose.transform.rotation.x,
                         current_pose.transform.rotation.y,
                         current_pose.transform.rotation.z,
                         current_pose.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 优先级：红绿灯 > 前方危险 > 正常路径
        if (red_light_detected_) {
            ROS_WARN("Red light detected - generating stopping trajectory");
            path = generateStoppingPath(current_x, current_y, yaw);
        } else if (front_hazard_) {
            ROS_WARN("Front hazard detected - performing avoidance maneuver");
            path = generateAvoidanceManeuver(current_x, current_y, yaw);
        } else {
            // 正常路径规划
            path = generateNormalPath(current_x, current_y, yaw);
        }
        
        return path;
    }
    
    // ========== 新增：生成停车路径 ==========
    std::vector<PathPoint> generateStoppingPath(double start_x, double start_y, double start_yaw) {
        std::vector<PathPoint> path;
        
        ROS_WARN("Generating red light stopping trajectory");
        
        // 生成短距离的减速停车路径
        double stopping_distance = 3.0;  // 停车距离
        double step_distance = stopping_distance / trajectory_points_;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            
            double progress = (double)i / trajectory_points_;
            double forward_dist = i * step_distance;
            
            // 直线前进，逐渐减速至停车
            point.x = start_x + forward_dist * cos(start_yaw);
            point.y = start_y + forward_dist * sin(start_yaw);
            point.yaw = start_yaw;
            point.is_safe = true;  // 停车路径认为是安全的
            
            path.push_back(point);
        }
        
        ROS_INFO("Generated red light stopping path with %lu points", path.size());
        return path;
    }
    
    // ========== 生成避障机动 ==========
    std::vector<PathPoint> generateAvoidanceManeuver(double start_x, double start_y, double start_yaw) {
        std::vector<PathPoint> path;
        
        // 更灵活的避障方案：多种策略组合
        std::vector<double> avoidance_directions = {1.0, -1.0, 1.0, -1.0};  // 右，左，右，左
        std::vector<double> avoidance_distances = {1.5, 2.0, 3.0, 4.0};    // 更小的起始距离
        std::vector<double> forward_speeds = {1.0, 0.8, 0.6, 0.4};         // 不同前进速度
        
        bool found_safe_path = false;
        double best_direction = 1.0;
        double best_distance = 2.0;
        double best_speed = 1.0;
        
        // 尝试找到安全的避障路径
        for (int i = 0; i < avoidance_directions.size(); ++i) {
            double direction = avoidance_directions[i];
            double distance = avoidance_distances[i];
            double speed = forward_speeds[i];
            
            if (checkAvoidancePathSafety(start_x, start_y, start_yaw, direction, distance, speed)) {
                best_direction = direction;
                best_distance = distance;
                best_speed = speed;
                found_safe_path = true;
                ROS_INFO("Found safe avoidance: direction=%.1f, distance=%.1f, speed=%.1f", 
                         direction, distance, speed);
                break;
            }
        }
        
        // 如果还是找不到安全路径，尝试更简单的策略
        if (!found_safe_path) {
            ROS_WARN("Standard avoidance failed, trying simplified strategies");
            
            // 策略1：仅侧移不回归
            if (checkSimpleAvoidance(start_x, start_y, start_yaw, 1.0, 2.0)) {
                best_direction = 1.0;
                best_distance = 2.0;
                found_safe_path = true;
                ROS_INFO("Using simplified RIGHT avoidance");
            } else if (checkSimpleAvoidance(start_x, start_y, start_yaw, -1.0, 2.0)) {
                best_direction = -1.0;
                best_distance = 2.0;
                found_safe_path = true;
                ROS_INFO("Using simplified LEFT avoidance");
            }
        }
        
        // 最后手段：强制生成路径
        if (!found_safe_path) {
            ROS_ERROR("All avoidance strategies failed! Generating forced emergency path");
            return generateForcedAvoidancePath(start_x, start_y, start_yaw);
        }
        
        ROS_WARN("Performing %s avoidance: distance %.1fm", 
                 best_direction > 0 ? "RIGHT" : "LEFT", best_distance);
        
        // 生成避障轨迹
        return generateSmoothAvoidancePath(start_x, start_y, start_yaw, best_direction, best_distance);
    }
    
    // ========== 检查简化避障方案 ==========
    bool checkSimpleAvoidance(double start_x, double start_y, double start_yaw, 
                              double direction, double distance) {
        int safety_checks = 4;  // 减少检查点
        double step = lookahead_distance_ / safety_checks;
        
        ROS_DEBUG("Checking simple avoidance: dir=%.1f", direction);
        
        for (int i = 1; i <= safety_checks; ++i) {
            double forward_dist = i * step;
            // 修正坐标变换
            double check_x = start_x + forward_dist * cos(start_yaw) + direction * distance * cos(start_yaw - M_PI/2);
            double check_y = start_y + forward_dist * sin(start_yaw) + direction * distance * sin(start_yaw - M_PI/2);
            
            // 只检查核心点，不检查车辆宽度
            if (isPointOccupied(check_x, check_y)) {
                return false;
            }
        }
        return true;
    }
    
    // ========== 生成强制避障路径 ==========
    std::vector<PathPoint> generateForcedAvoidancePath(double start_x, double start_y, double start_yaw) {
        std::vector<PathPoint> path;
        ROS_ERROR("Generating forced avoidance path - ignoring some obstacles");
        
        // 强制右避障，即使可能不完全安全
        double direction = 1.0;
        double distance = 3.0;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            
            double progress = (double)i / trajectory_points_;
            double forward_dist = i * (lookahead_distance_ / trajectory_points_);
            
            // 简单的侧移策略
            double lateral_offset = 0.0;
            if (progress < 0.5) {
                lateral_offset = direction * distance * progress * 2.0;  // 前半段侧移
            } else {
                lateral_offset = direction * distance;  // 后半段保持偏移
            }
            
            // 修正坐标变换
            point.x = start_x + forward_dist * cos(start_yaw) + lateral_offset * cos(start_yaw - M_PI/2);
            point.y = start_y + forward_dist * sin(start_yaw) + lateral_offset * sin(start_yaw - M_PI/2);
            point.yaw = start_yaw;
            point.is_safe = true;  // 强制认为安全
            
            path.push_back(point);
        }
        
        return path;
    }
    
    // ========== 生成平滑避障路径 ==========
    std::vector<PathPoint> generateSmoothAvoidancePath(double start_x, double start_y, double start_yaw,
                                                       double direction, double distance) {
        std::vector<PathPoint> path;
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        ROS_INFO("Generating avoidance path: start=(%.2f,%.2f), yaw=%.2f, direction=%.1f(1=RIGHT,-1=LEFT), distance=%.1f", 
                 start_x, start_y, start_yaw * 180.0 / M_PI, direction, distance);
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            
            double progress = (double)i / trajectory_points_;
            double forward_dist = i * step_distance;
            
            // 改进的避障曲线：更平滑的S形轨迹
            double lateral_offset = 0.0;
            if (progress < 0.4) {
                // 阶段1：平滑侧移
                double t = progress / 0.4;
                lateral_offset = direction * distance * (0.5 * (1 - cos(M_PI * t)));
            } else if (progress < 0.6) {
                // 阶段2：保持偏移
                lateral_offset = direction * distance;
            } else {
                // 阶段3：平滑回归
                double t = (progress - 0.6) / 0.4;
                lateral_offset = direction * distance * (0.5 * (1 + cos(M_PI * t)));
            }
            
            // 正确的坐标变换：
            // 向前：沿着yaw方向
            // 向右：沿着yaw + (-π/2)方向（右手坐标系中向右是-π/2）
            point.x = start_x + forward_dist * cos(start_yaw) + lateral_offset * cos(start_yaw - M_PI/2);
            point.y = start_y + forward_dist * sin(start_yaw) + lateral_offset * sin(start_yaw - M_PI/2);
            point.yaw = start_yaw;
            point.is_safe = !isPointOccupied(point.x, point.y);
            
            ROS_DEBUG("Point %d: forward=%.2f, lateral=%.2f, pos=(%.2f,%.2f)", 
                     i, forward_dist, lateral_offset, point.x, point.y);
            
            path.push_back(point);
        }
        
        ROS_INFO("Generated %s avoidance path with %lu points", 
                 direction > 0 ? "RIGHT" : "LEFT", path.size());
        
        return path;
    }
    
    // ========== 检查避障路径安全性 ==========
    bool checkAvoidancePathSafety(double start_x, double start_y, double start_yaw, 
                                  double direction, double distance, double speed_factor = 1.0) {
        int safety_checks = 5;  // 减少检查点数量
        double step = (lookahead_distance_ * speed_factor) / safety_checks;
        
        ROS_DEBUG("Checking avoidance safety: dir=%.1f, dist=%.1f", direction, distance);
        
        for (int i = 1; i <= safety_checks; ++i) {
            double progress = (double)i / safety_checks;
            double forward_dist = i * step;
            
            // 计算这个点的横向偏移
            double lateral_offset = 0.0;
            if (progress < 0.4) {
                double t = progress / 0.4;
                lateral_offset = direction * distance * (0.5 * (1 - cos(M_PI * t)));
            } else if (progress < 0.6) {
                lateral_offset = direction * distance;
            } else {
                double t = (progress - 0.6) / 0.4;
                lateral_offset = direction * distance * (0.5 * (1 + cos(M_PI * t)));
            }
            
            // 修正坐标变换 - 右侧是 -π/2 方向
            double check_x = start_x + forward_dist * cos(start_yaw) + lateral_offset * cos(start_yaw - M_PI/2);
            double check_y = start_y + forward_dist * sin(start_yaw) + lateral_offset * sin(start_yaw - M_PI/2);
            
            // 放宽安全检查：只检查车辆中心线附近
            double reduced_width = vehicle_width_ * 0.7;  // 减小检查宽度
            for (double w_offset = -reduced_width/2; w_offset <= reduced_width/2; w_offset += 0.8) {
                double final_x = check_x + w_offset * cos(start_yaw - M_PI/2);
                double final_y = check_y + w_offset * sin(start_yaw - M_PI/2);
                
                if (isPointOccupied(final_x, final_y)) {
                    ROS_DEBUG("Safety check failed at point %d: (%.2f,%.2f)", i, final_x, final_y);
                    return false;
                }
            }
        }
        ROS_DEBUG("Safety check passed for direction %.1f", direction);
        return true;
    }
    
    // ========== 生成正常路径 ==========
    std::vector<PathPoint> generateNormalPath(double start_x, double start_y, double start_yaw) {
        std::vector<PathPoint> path;
        
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        // 生成直线前进路径
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            
            double forward_dist = i * step_distance;
            point.x = start_x + forward_dist * cos(start_yaw);
            point.y = start_y + forward_dist * sin(start_yaw);
            point.yaw = start_yaw;
            point.is_safe = !isPointOccupied(point.x, point.y);
            
            path.push_back(point);
        }
        
        return path;
    }
    
    // ========== 发布默认轨迹 ==========
    void publishDefaultTrajectory() {
        msg_interfaces::Trajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "map";
        
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = i * step_distance;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            
            traj_msg.poses.push_back(pose);
            traj_msg.velocities.push_back(default_speed_);
            traj_msg.timestamps.push_back(i * (step_distance / default_speed_));
        }
        
        trajectory_pub_.publish(traj_msg);
    }
    
    // ========== 轨迹发布（修改速度处理） ==========
    void publishTrajectory(const std::vector<PathPoint>& path) {
        if (path.empty()) {
            ROS_WARN("Empty path, not publishing");
            return;
        }
        
        msg_interfaces::Trajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "map";
        
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        for (int i = 0; i < path.size(); ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = path[i].x;
            pose.position.y = path[i].y;
            pose.position.z = 0.0;
            
            // 设置朝向
            tf2::Quaternion q;
            q.setRPY(0, 0, path[i].yaw);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            
            traj_msg.poses.push_back(pose);
            
            // 根据规划模式设置速度
            double target_speed = default_speed_;
            if (red_light_detected_) {
                // 红灯时：逐渐减速至停车
                double progress = (double)(i + 1) / path.size();
                target_speed = default_speed_ * (1.0 - progress);  // 线性减速至0
            } else if (front_hazard_) {
                // 避障时：稍微减速
                target_speed = default_speed_ * 0.7;
            }
            
            traj_msg.velocities.push_back(target_speed);
            traj_msg.timestamps.push_back(i * (step_distance / std::max(0.1, target_speed)));
        }
        
        trajectory_pub_.publish(traj_msg);
        
        std::string mode = "NORMAL";
        if (red_light_detected_) mode = "RED_LIGHT_STOPPING";
        else if (front_hazard_) mode = "HAZARD_AVOIDANCE";
        
        ROS_DEBUG("Published %s trajectory: %lu points", mode.c_str(), path.size());
    }
    
    // ========== 简单平滑路径可视化 ==========
    void publishSmoothPathVisualization(const std::vector<PathPoint>& path) {
        if (path.empty()) return;
        
        ros::Time now = ros::Time::now();
        
        // 1. 发布平滑路径
        publishSmoothPath(path, now);
        
        // 2. 发布路径线标记
        publishPathLineMarker(path, now);
    }
    
    // ========== 平滑路径生成 ==========
    void publishSmoothPath(const std::vector<PathPoint>& path, ros::Time stamp) {
        nav_msgs::Path smooth_path;
        smooth_path.header.stamp = stamp;
        smooth_path.header.frame_id = "map";
        
        // 生成插值点以获得平滑曲线
        std::vector<geometry_msgs::PoseStamped> interpolated_poses = 
            interpolatePathPoints(path);
        
        smooth_path.poses = interpolated_poses;
        smooth_path_pub_.publish(smooth_path);
    }
    
    // ========== 路径点插值 ==========
    std::vector<geometry_msgs::PoseStamped> interpolatePathPoints(const std::vector<PathPoint>& path) {
        std::vector<geometry_msgs::PoseStamped> interpolated;
        
        if (path.size() < 2) return interpolated;
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            PathPoint start = path[i];
            PathPoint end = path[i + 1];
            
            // 计算两点间距离
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            // 计算插值点数量
            int num_points = std::max(2, (int)(distance / path_interpolation_resolution_));
            
            // 生成平滑插值点
            for (int j = 0; j <= num_points; ++j) {
                double t = (double)j / num_points;
                
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                
                // 使用平滑的三次插值
                double smooth_t = smoothStep(t);
                pose_stamped.pose.position.x = start.x + smooth_t * dx;
                pose_stamped.pose.position.y = start.y + smooth_t * dy;
                pose_stamped.pose.position.z = 0.05;  // 稍微抬高
                
                // 计算朝向
                tf2::Quaternion q;
                q.setRPY(0, 0, start.yaw + smooth_t * (end.yaw - start.yaw));
                pose_stamped.pose.orientation.x = q.x();
                pose_stamped.pose.orientation.y = q.y();
                pose_stamped.pose.orientation.z = q.z();
                pose_stamped.pose.orientation.w = q.w();
                
                interpolated.push_back(pose_stamped);
            }
        }
        
        return interpolated;
    }
    
    // ========== 平滑步骤函数 ==========
    double smoothStep(double t) {
        // 使用平滑的S形曲线 (smoothstep函数)
        return t * t * (3.0 - 2.0 * t);
    }
    
    // ========== 路径线标记（修改颜色显示不同模式） ==========
    void publishPathLineMarker(const std::vector<PathPoint>& path, ros::Time stamp) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = stamp;
        line_marker.ns = "planned_path";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        // 线条样式
        line_marker.scale.x = 0.15;  // 线宽
        
        // 根据路径类型设置颜色
        if (red_light_detected_) {
            // 红灯停车路径 - 红色
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.9;
        } else if (front_hazard_) {
            // 避障路径 - 橙红色
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.4;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.9;
        } else {
            // 正常路径 - 蓝绿色
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.8;
            line_marker.color.b = 1.0;
            line_marker.color.a = 0.8;
        }
        
        // 添加路径点
        for (const auto& point : path) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.1;  // 稍微抬高便于观察
            line_marker.points.push_back(p);
        }
        
        // 添加起点和终点标记球
        if (!path.empty()) {
            // 起点标记
            visualization_msgs::Marker start_marker;
            start_marker.header.frame_id = "map";
            start_marker.header.stamp = stamp;
            start_marker.ns = "path_endpoints";
            start_marker.id = 1;
            start_marker.type = visualization_msgs::Marker::SPHERE;
            start_marker.action = visualization_msgs::Marker::ADD;
            
            start_marker.pose.position.x = path[0].x;
            start_marker.pose.position.y = path[0].y;
            start_marker.pose.position.z = 0.15;
            start_marker.pose.orientation.w = 1.0;
            
            start_marker.scale.x = 0.3;
            start_marker.scale.y = 0.3;
            start_marker.scale.z = 0.3;
            
            start_marker.color.r = 0.0;
            start_marker.color.g = 1.0;
            start_marker.color.b = 0.0;
            start_marker.color.a = 1.0;
            
            // 终点标记
            visualization_msgs::Marker end_marker = start_marker;
            end_marker.id = 2;
            end_marker.pose.position.x = path.back().x;
            end_marker.pose.position.y = path.back().y;
            end_marker.color.r = 1.0;
            end_marker.color.g = 0.0;
            end_marker.color.b = 0.0;
            
            // 先发布线条，再发布点
            path_line_pub_.publish(line_marker);
            
            // 发布起点终点（使用不同的时间戳避免冲突）
            ros::Duration(0.01).sleep();
            path_line_pub_.publish(start_marker);
            ros::Duration(0.01).sleep();
            path_line_pub_.publish(end_marker);
        } else {
            path_line_pub_.publish(line_marker);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    
    PathPlannerNode planner(nh);
    
    ROS_INFO("Path Planner Node with traffic light integration and obstacle avoidance ready");
    ros::spin();
    
    return 0;
}