#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <msg_interfaces/Trajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <deque>

struct PathPoint {
    double x, y, yaw;
    double velocity;
    ros::Time timestamp;
};

struct TrailPoint {
    double x, y, z;
    ros::Time timestamp;
    double speed;
};

class VehicleFollowingPlanningNode
{
public:
    VehicleFollowingPlanningNode()
    {
        // ========== 订阅车辆位置和决策信息 ==========
        vehicle_pose_sub_ = nh_.subscribe("/vehicle/pose", 1, 
                                         &VehicleFollowingPlanningNode::vehiclePoseCallback, this);
        vehicle_odom_sub_ = nh_.subscribe("/vehicle/odometry", 1,
                                         &VehicleFollowingPlanningNode::vehicleOdomCallback, this);
        
        // ========== 订阅决策模块的两个输出话题 ==========
        behavior_command_sub_ = nh_.subscribe("/decision/behavior_command", 1,
                                             &VehicleFollowingPlanningNode::behaviorCommandCallback, this);
        emergency_stop_sub_ = nh_.subscribe("/decision/emergency_stop", 1,
                                           &VehicleFollowingPlanningNode::emergencyStopCallback, this);

        // ========== 发布轨迹和可视化 ==========
        trajectory_pub_ = nh_.advertise<msg_interfaces::Trajectory>("/planning/trajectory", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/path", 10);
        planned_path_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planning/planned_path_visualization", 1);
        vehicle_trail_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planning/vehicle_trail", 1);
        planning_info_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/info_display", 1);

        // ========== 参数设置 ==========
        nh_.param("planning_frequency", planning_frequency_, 10.0);
        nh_.param("lookahead_distance", lookahead_distance_, 20.0);  // 增加前瞻距离
        nh_.param("trajectory_points", trajectory_points_, 25);     // 增加路径点数
        nh_.param("default_speed", default_speed_, 8.0);  // 默认速度 8 m/s
        nh_.param("emergency_decel", emergency_decel_, 3.0);  // 紧急减速度
        nh_.param("trail_max_points", trail_max_points_, 200);
        nh_.param("trail_update_distance", trail_update_distance_, 0.5);

        // ========== 状态初始化 ==========
        vehicle_pose_received_ = false;
        is_emergency_stop_ = false;
        behavior_command_ = "normal";  // 默认正常行为
        current_speed_ = 0.0;
        
        // 避让状态初始化
        is_avoiding_ = false;
        avoid_start_time_ = 0.0;
        avoid_offset_ = 0.0;
        last_behavior_command_ = "normal";
        
        // 车辆状态
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        
        // 轨迹历史
        trail_points_.clear();
        last_trail_x_ = 0.0;
        last_trail_y_ = 0.0;

        // ========== 启动定时器 ==========
        planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_),
                                         &VehicleFollowingPlanningNode::planningTimerCallback, this);
        
        visualization_timer_ = nh_.createTimer(ros::Duration(0.1),  // 10Hz更新可视化
                                              &VehicleFollowingPlanningNode::visualizationTimerCallback, this);

        ROS_INFO("🚗 Vehicle Following Planning Node started");
        ROS_INFO("📋 Parameters: freq=%.1fHz, lookahead=%.1fm, speed=%.1fm/s", 
                 planning_frequency_, lookahead_distance_, default_speed_);
    }

private:
    // ========== ROS相关 ==========
    ros::NodeHandle nh_;
    
    // 订阅器
    ros::Subscriber vehicle_pose_sub_;
    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber behavior_command_sub_;
    ros::Subscriber emergency_stop_sub_;
    
    // 发布器
    ros::Publisher trajectory_pub_;
    ros::Publisher path_pub_;
    ros::Publisher planned_path_vis_pub_;
    ros::Publisher vehicle_trail_pub_;
    ros::Publisher planning_info_pub_;
    
    // 定时器
    ros::Timer planning_timer_;
    ros::Timer visualization_timer_;

    // ========== 状态变量 ==========
    bool vehicle_pose_received_;
    bool is_emergency_stop_;        // 来自决策模块的紧急停车指令
    std::string behavior_command_;  // 来自决策模块的行为指令
    double current_speed_;
    
    // 避让状态记录
    bool is_avoiding_;
    double avoid_start_time_;
    double avoid_offset_;
    std::string last_behavior_command_;
    
    // 当前车辆状态
    double current_x_, current_y_, current_yaw_;
    
    // 当前规划路径
    std::vector<PathPoint> current_planned_path_;
    
    // 车辆行驶轨迹
    std::deque<TrailPoint> trail_points_;
    double last_trail_x_, last_trail_y_;

    // ========== 参数 ==========
    double planning_frequency_;
    double lookahead_distance_;
    int trajectory_points_;
    double default_speed_;
    double emergency_decel_;
    int trail_max_points_;
    double trail_update_distance_;

    // ========== 回调函数 ==========
    void vehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
        
        // 提取yaw角
        tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                         msg->pose.orientation.z, msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        if (!vehicle_pose_received_) {
            ROS_INFO("✅ Vehicle pose received: (%.2f, %.2f, %.1f°)", 
                     current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
            vehicle_pose_received_ = true;
        }
        
        // 更新轨迹点
        updateTrail(current_x_, current_y_, 0.0, current_speed_);
    }

    void vehicleOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + 
                             pow(msg->twist.twist.linear.y, 2));
    }

    void behaviorCommandCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (behavior_command_ != msg->data) {
            last_behavior_command_ = behavior_command_;
            behavior_command_ = msg->data;
            
            // 检测避让行为的开始
            if ((behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") && 
                (last_behavior_command_ != "avoid_left" && last_behavior_command_ != "avoid_right")) {
                is_avoiding_ = true;
                avoid_start_time_ = ros::Time::now().toSec();
                avoid_offset_ = (behavior_command_ == "avoid_left") ? 3.0 : -3.0;  // 左避让+3m，右避让-3m
                ROS_INFO("🎯 Starting Avoidance: %s (offset: %.1fm)", behavior_command_.c_str(), avoid_offset_);
                
                // 立即重新生成路径
                generateAndPublishPath();
            }
            // 检测避让行为的结束
            else if ((last_behavior_command_ == "avoid_left" || last_behavior_command_ == "avoid_right") && 
                     behavior_command_ == "normal") {
                is_avoiding_ = false;
                avoid_offset_ = 0.0;
                ROS_INFO("✅ Avoidance Complete, returning to normal");
                
                // 立即重新生成路径
                generateAndPublishPath();
            }
            
            ROS_INFO("🎯 Behavior Command: %s → %s", last_behavior_command_.c_str(), behavior_command_.c_str());
        }
    }

    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        bool new_emergency_state = msg->data;
        if (new_emergency_state != is_emergency_stop_) {
            is_emergency_stop_ = new_emergency_state;
            if (is_emergency_stop_) {
                ROS_INFO("🚨 Emergency Stop: ACTIVATED");
            } else {
                ROS_INFO("✅ Emergency Stop: DEACTIVATED");
            }
        }
    }

    // ========== 轨迹更新 ==========
    void updateTrail(double x, double y, double z, double speed)
    {
        // 检查是否需要添加新的轨迹点
        double distance_from_last = sqrt(pow(x - last_trail_x_, 2) + pow(y - last_trail_y_, 2));
        
        if (distance_from_last >= trail_update_distance_ || trail_points_.empty()) {
            TrailPoint new_point;
            new_point.x = x;
            new_point.y = y;
            new_point.z = z;
            new_point.timestamp = ros::Time::now();
            new_point.speed = speed;
            
            trail_points_.push_back(new_point);
            last_trail_x_ = x;
            last_trail_y_ = y;
            
            // 限制轨迹点数量
            while (trail_points_.size() > trail_max_points_) {
                trail_points_.pop_front();
            }
        }
    }

    // ========== 定时器回调 ==========
    void planningTimerCallback(const ros::TimerEvent& event)
    {
        if (!vehicle_pose_received_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for vehicle pose...");
            return;
        }
        
        generateAndPublishPath();
    }

    void visualizationTimerCallback(const ros::TimerEvent& event)
    {
        if (vehicle_pose_received_) {
            publishPlannedPathVisualization();
            publishVehicleTrailVisualization();
            publishPlanningInfo();
        }
    }

    // ========== 路径规划 ==========
    void generateAndPublishPath()
    {
        std::vector<PathPoint> path = generateForwardPath();
        current_planned_path_ = path;
        
        publishTrajectory(path);
        publishPath(path);
        
        ROS_DEBUG("Generated path with %lu points", path.size());
    }

    std::vector<PathPoint> generateForwardPath()
    {
        std::vector<PathPoint> path;
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        // 根据决策模块的两个指令确定速度和路径特性
        double target_speed = calculateTargetSpeed();
        
        ROS_INFO_THROTTLE(1.0, "🔄 Generating path: behavior=%s, emergency=%s", 
                         behavior_command_.c_str(), is_emergency_stop_ ? "true" : "false");
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            double distance = i * step_distance;
            double progress = (double)i / trajectory_points_;
            
            // 基本前进路径
            point.x = current_x_ + distance * cos(current_yaw_);
            point.y = current_y_ + distance * sin(current_yaw_);
            point.yaw = current_yaw_;
            
            // 根据行为指令调整路径
            if (behavior_command_ == "avoid_left") {
                // 左避让：S形路径 - 向左偏移后回中心
                double lateral_offset;
                if (progress < 0.3) {
                    // 前30%：向左偏移
                    lateral_offset = 4.0 * (progress / 0.3);
                } else if (progress < 0.7) {
                    // 中间40%：保持偏移
                    lateral_offset = 4.0;
                } else {
                    // 后30%：回到中心
                    lateral_offset = 4.0 * (1.0 - (progress - 0.7) / 0.3);
                }
                
                point.x += lateral_offset * cos(current_yaw_ + M_PI/2);
                point.y += lateral_offset * sin(current_yaw_ + M_PI/2);
                
                ROS_INFO_THROTTLE(2.0, "🔄 Avoid Left: point %d, progress=%.2f, offset=%.2f, pos=(%.1f,%.1f)", 
                                 i, progress, lateral_offset, point.x, point.y);
                
            } else if (behavior_command_ == "avoid_right") {
                // 右避让：S形路径 - 向右偏移后回中心
                double lateral_offset;
                if (progress < 0.3) {
                    // 前30%：向右偏移
                    lateral_offset = -4.0 * (progress / 0.3);
                } else if (progress < 0.7) {
                    // 中间40%：保持偏移
                    lateral_offset = -4.0;
                } else {
                    // 后30%：回到中心
                    lateral_offset = -4.0 * (1.0 - (progress - 0.7) / 0.3);
                }
                
                point.x += lateral_offset * cos(current_yaw_ + M_PI/2);
                point.y += lateral_offset * sin(current_yaw_ + M_PI/2);
                
                ROS_INFO_THROTTLE(2.0, "🔄 Avoid Right: point %d, progress=%.2f, offset=%.2f, pos=(%.1f,%.1f)", 
                                 i, progress, lateral_offset, point.x, point.y);
            }
            
            // 根据决策指令调整速度
            if (is_emergency_stop_) {
                // 紧急停车：指数衰减
                point.velocity = target_speed * exp(-progress * emergency_decel_);
            } else if (behavior_command_ == "slow_down") {
                // 减速行驶
                point.velocity = target_speed * 0.6;
            } else if (behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") {
                // 避让时适当减速
                point.velocity = target_speed * 0.7;
            } else {
                // 正常行驶
                point.velocity = target_speed;
            }
            
            point.timestamp = ros::Time::now();
            path.push_back(point);
        }
        
        ROS_INFO_THROTTLE(2.0, "✅ Generated %s path with %lu points, first point: (%.1f, %.1f)", 
                          behavior_command_.c_str(), path.size(), 
                          path.empty() ? 0.0 : path[0].x, path.empty() ? 0.0 : path[0].y);
        
        return path;
    }

    double calculateTargetSpeed()
    {
        if (is_emergency_stop_) {
            return 0.0;  // 紧急停车
        } else if (behavior_command_ == "slow_down") {
            return default_speed_ * 0.5;  // 减速
        } else {
            return default_speed_;  // 正常速度
        }
    }

    double calculatePathCurvature()
    {
        // 简单的路径曲率计算（可以后续与地图集成）
        if (is_emergency_stop_ || behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") {
            return 0.0;  // 紧急情况或避让时不添加额外曲率
        }
        
        // 根据当前位置添加一些变化
        static double curve_phase = 0.0;
        curve_phase += 0.05;
        return sin(curve_phase) * 2.0;  // 轻微S型路径
    }

    // ========== 发布函数 ==========
    void publishTrajectory(const std::vector<PathPoint>& path)
    {
        if (path.empty()) return;

        msg_interfaces::Trajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "map";

        for (size_t i = 0; i < path.size(); ++i) {
            geometry_msgs::Pose pose;
            pose.position.x = path[i].x;
            pose.position.y = path[i].y;
            pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, path[i].yaw);
            pose.orientation = tf2::toMsg(q);

            traj_msg.poses.push_back(pose);
            traj_msg.velocities.push_back(path[i].velocity);
            traj_msg.timestamps.push_back(i * 0.5);
        }

        trajectory_pub_.publish(traj_msg);
    }

    void publishPath(const std::vector<PathPoint>& path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            
            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = 0.1;

            tf2::Quaternion q;
            q.setRPY(0, 0, point.yaw);
            pose_stamped.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_.publish(path_msg);
    }

    void publishPlannedPathVisualization()
    {
        if (current_planned_path_.empty()) {
            ROS_WARN_THROTTLE(2.0, "❌ Cannot publish visualization: path is empty");
            return;
        }
        
        visualization_msgs::MarkerArray marker_array;
        
        // 路径线条
        visualization_msgs::Marker path_line;
        path_line.header.frame_id = "map";
        path_line.header.stamp = ros::Time::now();
        path_line.ns = "planned_path";
        path_line.id = 0;
        path_line.type = visualization_msgs::Marker::LINE_STRIP;
        path_line.action = visualization_msgs::Marker::ADD;
        
        path_line.scale.x = 0.5;  // 增加线条宽度以便更好可视化
        
        // 根据决策状态设置颜色
        if (is_emergency_stop_) {
            path_line.color.r = 1.0; path_line.color.g = 0.0; path_line.color.b = 0.0;  // 红色：紧急停车
        } else if (behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") {
            path_line.color.r = 1.0; path_line.color.g = 1.0; path_line.color.b = 0.0;  // 黄色：避让
        } else if (behavior_command_ == "slow_down") {
            path_line.color.r = 1.0; path_line.color.g = 0.5; path_line.color.b = 0.0;  // 橙色：减速
        } else {
            path_line.color.r = 0.0; path_line.color.g = 1.0; path_line.color.b = 0.0;  // 绿色：正常
        }
        path_line.color.a = 0.8;
        
        ROS_INFO_THROTTLE(2.0, "🎨 Publishing path visualization: behavior=%s, color=(%.1f,%.1f,%.1f), points=%lu",
                         behavior_command_.c_str(), path_line.color.r, path_line.color.g, path_line.color.b, 
                         current_planned_path_.size());
        
        for (const auto& point : current_planned_path_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.8;  // 提高路径高度以便更好可视化
            path_line.points.push_back(p);
        }
        
        // 输出第一个和最后一个点的信息
        if (!current_planned_path_.empty()) {
            auto first = current_planned_path_.front();
            auto last = current_planned_path_.back();
            ROS_INFO_THROTTLE(2.0, "📍 Path range: first=(%.1f,%.1f) last=(%.1f,%.1f)",
                             first.x, first.y, last.x, last.y);
        }
        
        marker_array.markers.push_back(path_line);
        
        // 路径点（速度可视化）
        visualization_msgs::Marker path_points;
        path_points.header.frame_id = "map";
        path_points.header.stamp = ros::Time::now();
        path_points.ns = "planned_points";
        path_points.id = 1;
        path_points.type = visualization_msgs::Marker::SPHERE_LIST;
        path_points.action = visualization_msgs::Marker::ADD;
        
        path_points.scale.x = 0.6;  // 增加点的大小
        path_points.scale.y = 0.6;
        path_points.scale.z = 0.6;
        
        for (const auto& point : current_planned_path_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 1.0;  // 提高点的高度
            path_points.points.push_back(p);
            
            // 根据速度设置颜色
            std_msgs::ColorRGBA color;
            double speed_ratio = std::min(1.0, point.velocity / default_speed_);
            color.r = 1.0 - speed_ratio;  // 速度越高红色越少
            color.g = speed_ratio;        // 速度越高绿色越多
            color.b = 0.2;
            color.a = 0.9;  // 增加透明度
            path_points.colors.push_back(color);
        }
        
        marker_array.markers.push_back(path_points);
        
        planned_path_vis_pub_.publish(marker_array);
        
        ROS_INFO_THROTTLE(2.0, "✅ Published MarkerArray with %lu markers to /planning/planned_path_visualization", 
                         marker_array.markers.size());
    }

    void publishVehicleTrailVisualization()
    {
        if (trail_points_.empty()) return;
        
        visualization_msgs::MarkerArray marker_array;
        
        // 轨迹线条
        visualization_msgs::Marker trail_line;
        trail_line.header.frame_id = "map";
        trail_line.header.stamp = ros::Time::now();
        trail_line.ns = "vehicle_trail";
        trail_line.id = 0;
        trail_line.type = visualization_msgs::Marker::LINE_STRIP;
        trail_line.action = visualization_msgs::Marker::ADD;
        
        trail_line.scale.x = 0.2;
        trail_line.color.r = 0.0;
        trail_line.color.g = 0.0;
        trail_line.color.b = 1.0;
        trail_line.color.a = 0.6;
        
        ros::Time current_time = ros::Time::now();
        
        for (const auto& trail_point : trail_points_) {
            geometry_msgs::Point p;
            p.x = trail_point.x;
            p.y = trail_point.y;
            p.z = 0.1;
            trail_line.points.push_back(p);
        }
        
        marker_array.markers.push_back(trail_line);
        
        // 轨迹点（带时间衰减效果）
        visualization_msgs::Marker trail_points_marker;
        trail_points_marker.header.frame_id = "map";
        trail_points_marker.header.stamp = ros::Time::now();
        trail_points_marker.ns = "trail_points";
        trail_points_marker.id = 1;
        trail_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        trail_points_marker.action = visualization_msgs::Marker::ADD;
        
        trail_points_marker.scale.x = 0.15;
        trail_points_marker.scale.y = 0.15;
        trail_points_marker.scale.z = 0.15;
        
        for (size_t i = 0; i < trail_points_.size(); ++i) {
            const auto& trail_point = trail_points_[i];
            
            geometry_msgs::Point p;
            p.x = trail_point.x;
            p.y = trail_point.y;
            p.z = 0.15;
            trail_points_marker.points.push_back(p);
            
            // 时间衰减效果
            double age_ratio = (double)i / trail_points_.size();
            std_msgs::ColorRGBA color;
            color.r = 0.2;
            color.g = 0.2;
            color.b = 1.0;
            color.a = age_ratio * 0.8;  // 越新的点越亮
            trail_points_marker.colors.push_back(color);
        }
        
        marker_array.markers.push_back(trail_points_marker);
        
        vehicle_trail_pub_.publish(marker_array);
    }

    void publishPlanningInfo()
    {
        visualization_msgs::Marker info_marker;
        info_marker.header.frame_id = "map";
        info_marker.header.stamp = ros::Time::now();
        info_marker.ns = "planning_info";
        info_marker.id = 0;
        info_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        info_marker.action = visualization_msgs::Marker::ADD;
        
        // 在车辆上方显示信息
        info_marker.pose.position.x = current_x_;
        info_marker.pose.position.y = current_y_;
        info_marker.pose.position.z = 4.0;
        info_marker.pose.orientation.w = 1.0;
        
        info_marker.scale.z = 0.8;
        
        // 构建信息文本
        std::string info_text = "🎯 PLANNING";
        info_text += "\n📍 Pos: (" + std::to_string((int)current_x_) + "," + std::to_string((int)current_y_) + ")";
        info_text += "\n⚡ Speed: " + std::to_string((int)(current_speed_ * 3.6)) + " km/h";
        info_text += "\n🛣️ Path: " + std::to_string(current_planned_path_.size()) + " points";
        info_text += "\n👣 Trail: " + std::to_string(trail_points_.size()) + " points";
        info_text += "\n🎭 Behavior: " + behavior_command_;
        if (behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") {
            info_text += " (4m offset)";
        }
        
        // 状态指示
        if (is_emergency_stop_) {
            info_text += "\n🚨 EMERGENCY STOP";
            info_marker.color.r = 1.0; info_marker.color.g = 0.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "avoid_left") {
            info_text += "\n⬅️ AVOIDING LEFT";
            info_marker.color.r = 1.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "avoid_right") {
            info_text += "\n➡️ AVOIDING RIGHT";
            info_marker.color.r = 1.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "slow_down") {
            info_text += "\n🐌 SLOWING DOWN";
            info_marker.color.r = 1.0; info_marker.color.g = 0.5; info_marker.color.b = 0.0;
        } else {
            info_text += "\n✅ NORMAL DRIVING";
            info_marker.color.r = 0.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        }
        info_marker.color.a = 1.0;
        
        info_marker.text = info_text;
        planning_info_pub_.publish(info_marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_following_planning_node");
    
    VehicleFollowingPlanningNode planning_node;
    
    ROS_INFO("===== Vehicle Following Planning Node Ready =====");
    ROS_INFO("📡 Subscribed Topics:");
    ROS_INFO("  - /vehicle/pose (车辆位置)");
    ROS_INFO("  - /vehicle/odometry (车辆速度)");
    ROS_INFO("  - /decision/behavior_command (行为指令)");
    ROS_INFO("  - /decision/emergency_stop (紧急停车)");
    ROS_INFO("📤 Published Topics:");
    ROS_INFO("  - /planning/trajectory (轨迹)");
    ROS_INFO("  - /planning/path (路径)");
    ROS_INFO("  - /planning/planned_path_visualization (前方路径可视化)");
    ROS_INFO("  - /planning/vehicle_trail (车辆轨迹可视化)");
    ROS_INFO("  - /planning/info_display (规划信息显示)");
    ROS_INFO("🎯 Features:");
    ROS_INFO("  ✅ 跟随车辆位置规划前方路径");
    ROS_INFO("  ✅ 记录和显示车辆行驶轨迹");
    ROS_INFO("  ✅ 响应决策模块的行为指令和紧急停车");
    ROS_INFO("  ✅ 支持避让、减速、正常行驶等行为");
    ROS_INFO("================================================");
    
    ros::spin();
    
    return 0;
}