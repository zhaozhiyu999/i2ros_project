#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <msg_interfaces/Trajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

struct PathPoint {
    double x, y, yaw;
    double velocity;
    bool is_safe;
};

class PlanningNode
{
public:
    PlanningNode() : tf_listener_(tf_buffer_)
    {
        // ========== 立即发布静态TF ========== 
        setupStaticTransforms();
        
        // ========== 订阅决策和感知模块 ==========
        behavior_command_sub_ = nh_.subscribe("/decision/behavior_command", 1,
                                             &PlanningNode::behaviorCommandCallback, this);
        occupancy_grid_sub_ = nh_.subscribe("/perception/occupancy_grid", 1,
                                           &PlanningNode::occupancyGridCallback, this);

        // ========== 发布轨迹和可视化 ==========
        trajectory_pub_ = nh_.advertise<msg_interfaces::Trajectory>("/planning/trajectory", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/path", 10);
        
        // ========== 地图可视化发布器 ==========
        map_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/map_visualization", 1);
        inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/planning/inflated_map", 1);
        
        // ========== 路径可视化发布器 ==========
        path_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/path_visualization", 1);
        behavior_indicator_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/behavior_indicator", 1);
        
        // ========== 新增：车辆模型可视化 ==========
        vehicle_model_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/vehicle_model", 1);

        // ========== 参数设置 ==========
        nh_.param("planning_frequency", planning_frequency_, 5.0);
        nh_.param("lookahead_distance", lookahead_distance_, 10.0);
        nh_.param("trajectory_points", trajectory_points_, 8);
        nh_.param("default_speed", default_speed_, 3.0);
        nh_.param("avoidance_lateral_distance", avoidance_lateral_distance_, 2.5);
        nh_.param("vehicle_width", vehicle_width_, 1.8);
        nh_.param("safety_margin", safety_margin_, 0.5);

        // ========== 状态初始化 ==========
        current_behavior_ = "STRAIGHT";
        has_map_ = false;
        initialized_ = false;
        
        // 使用简单的位置跟踪，不依赖外部TF
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;

        // ========== 启动定时器 ==========
        planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_),
                                         &PlanningNode::planningTimerCallback, this);
        
        visualization_timer_ = nh_.createTimer(ros::Duration(0.2),  // 5Hz可视化更新，更加流畅
                                              &PlanningNode::visualizationTimerCallback, this);

        ROS_INFO("Planning Node started with enhanced visualization");
        ROS_INFO("Planning frequency: %.1f Hz, Lookahead: %.1f m", planning_frequency_, lookahead_distance_);
        
        // 延迟3秒后开始发布，等待其他节点启动
        ros::Duration(3.0).sleep();
        ROS_INFO("Planning node ready - starting visualization");
    }

private:
    // ========== ROS相关 ==========
    ros::NodeHandle nh_;
    ros::Subscriber behavior_command_sub_;
    ros::Subscriber occupancy_grid_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher path_pub_;
    ros::Publisher map_visualization_pub_;
    ros::Publisher inflated_map_pub_;
    ros::Publisher path_visualization_pub_;
    ros::Publisher behavior_indicator_pub_;
    ros::Publisher vehicle_model_pub_;  // 新增车辆模型发布器
    
    ros::Timer planning_timer_;
    ros::Timer visualization_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformBroadcaster dynamic_tf_broadcaster_;

    // ========== 状态变量 ==========
    std::string current_behavior_;
    nav_msgs::OccupancyGrid current_map_;
    bool has_map_;
    bool initialized_;
    std::vector<PathPoint> current_path_;
    
    // 简化的位置跟踪
    double current_x_, current_y_, current_yaw_;

    // ========== 参数 ==========
    double planning_frequency_;
    double lookahead_distance_;
    int trajectory_points_;
    double default_speed_;
    double avoidance_lateral_distance_;
    double vehicle_width_;
    double safety_margin_;

    // ========== 优化：TF设置更加健壮 ========== 
    void setupStaticTransforms()
    {
        std::vector<geometry_msgs::TransformStamped> static_transforms;
        
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
        ROS_INFO("Published static TF transforms for sensors");
        
        // 等待TF可用
        ros::Duration(1.0).sleep();
    }

    // ========== 修复：动态TF发布更加频繁和稳定 ==========
    void publishDynamicTF()
    {
        geometry_msgs::TransformStamped map_to_base;
        map_to_base.header.stamp = ros::Time::now();
        map_to_base.header.frame_id = "map";
        map_to_base.child_frame_id = "base_link";
        
        // 使用当前车辆位置
        map_to_base.transform.translation.x = current_x_;
        map_to_base.transform.translation.y = current_y_;
        map_to_base.transform.translation.z = 0.0;
        
        // 使用当前车辆朝向
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        map_to_base.transform.rotation.x = q.x();
        map_to_base.transform.rotation.y = q.y();
        map_to_base.transform.rotation.z = q.z();
        map_to_base.transform.rotation.w = q.w();
        
        dynamic_tf_broadcaster_.sendTransform(map_to_base);
    }

    // ========== 回调函数 ==========
    void behaviorCommandCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string new_behavior = msg->data;
        
        if (new_behavior != current_behavior_) {
            ROS_INFO("🔄 Planning behavior changed: %s → %s", 
                     current_behavior_.c_str(), new_behavior.c_str());
            current_behavior_ = new_behavior;
            generateAndPublishPath();
        }
    }

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        current_map_ = *msg;
        has_map_ = true;
        ROS_INFO_ONCE("📍 Received occupancy grid map for planning");
        ROS_INFO_THROTTLE(5.0, "Map updated: %dx%d cells, resolution=%.3f", 
                         msg->info.width, msg->info.height, msg->info.resolution);
    }

    // ========== 定时器回调，确保高频率更新 ==========
    void planningTimerCallback(const ros::TimerEvent& event)
    {
        generateAndPublishPath();
        publishDynamicTF();  // 每次都发布TF
    }

    void visualizationTimerCallback(const ros::TimerEvent& event)
    {
        // 始终发布车辆模型
        publishVehicleModel();
        
        if (has_map_) {
            publishMapVisualization();
            publishInflatedMap();
        }
        
        if (!current_path_.empty()) {
            publishPathVisualization();
            publishBehaviorIndicator();
        }
        
        // 高频发布TF
        publishDynamicTF();
    }

    // ========== 位置获取优化 ==========
    bool getCurrentPose(double& x, double& y, double& yaw)
    {
        // 首先尝试从外部TF获取位置
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
            
            current_x_ = transform.transform.translation.x;
            current_y_ = transform.transform.translation.y;
            
            tf2::Quaternion q(transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z,
                             transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, current_yaw_);
            
            x = current_x_;
            y = current_y_;
            yaw = current_yaw_;
            
            if (!initialized_) {
                ROS_INFO("✅ Received external vehicle position: (%.2f, %.2f, %.2f°)", 
                        x, y, yaw * 180.0 / M_PI);
                initialized_ = true;
            }
            
            return true;
            
        } catch (tf2::TransformException& ex) {
            // 如果没有外部定位，基于地图中心初始化
            if (!initialized_ && has_map_) {
                current_x_ = current_map_.info.origin.position.x + 
                            (current_map_.info.width * current_map_.info.resolution) / 2.0;
                current_y_ = current_map_.info.origin.position.y + 
                            (current_map_.info.height * current_map_.info.resolution) / 2.0;
                current_yaw_ = 0.0;
                initialized_ = true;
                
                ROS_INFO("🎯 Initialized vehicle at map center: (%.2f, %.2f)", current_x_, current_y_);
            } else if (!has_map_) {
                // 使用模拟运动
                static double sim_time = 0.0;
                sim_time += 0.2;
                
                current_x_ = 10.0 * cos(sim_time * 0.05);  // 更大的圆形路径
                current_y_ = 10.0 * sin(sim_time * 0.05);
                current_yaw_ = sim_time * 0.05 + M_PI/2;
                
                ROS_INFO_THROTTLE(2.0, "🚗 Simulated vehicle position: (%.2f, %.2f, %.1f°)", 
                                 current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
            }
            
            x = current_x_;
            y = current_y_;
            yaw = current_yaw_;
            
            return true;
        }
    }

    // ========== 核心路径规划 ==========
    void generateAndPublishPath()
    {
        double current_x, current_y, current_yaw;
        if (!getCurrentPose(current_x, current_y, current_yaw)) {
            return;
        }

        std::vector<PathPoint> path;
        if (current_behavior_ == "STRAIGHT") {
            path = generateStraightPath(current_x, current_y, current_yaw);
        } else if (current_behavior_ == "AVOIDANCE") {
            path = generateAvoidancePath(current_x, current_y, current_yaw);
        } else if (current_behavior_ == "EMERGENCY_STOP") {
            path = generateStoppingPath(current_x, current_y, current_yaw);
        } else {
            path = generateStraightPath(current_x, current_y, current_yaw);
        }

        current_path_ = path;
        publishTrajectory(path);
        publishPath(path);

        ROS_DEBUG("Generated %s path with %lu points at (%.2f, %.2f)", 
                 current_behavior_.c_str(), path.size(), current_x, current_y);
    }

    // ========== 路径生成函数 ==========
    std::vector<PathPoint> generateStraightPath(double start_x, double start_y, double start_yaw)
    {
        std::vector<PathPoint> path;
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            double distance = i * step_distance;
            point.x = start_x + distance * cos(start_yaw);
            point.y = start_y + distance * sin(start_yaw);
            point.yaw = start_yaw;
            point.velocity = default_speed_;
            point.is_safe = !isPointOccupied(point.x, point.y);
            path.push_back(point);
        }
        return path;
    }

    std::vector<PathPoint> generateAvoidancePath(double start_x, double start_y, double start_yaw)
    {
        std::vector<PathPoint> path;
        double step_distance = lookahead_distance_ / trajectory_points_;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            double progress = (double)i / trajectory_points_;
            double forward_distance = i * step_distance;
            
            // S形避障轨迹
            double lateral_offset = 0.0;
            if (progress < 0.3) {
                double t = progress / 0.3;
                lateral_offset = avoidance_lateral_distance_ * (0.5 * (1 - cos(M_PI * t)));
            } else if (progress < 0.7) {
                lateral_offset = avoidance_lateral_distance_;
            } else {
                double t = (progress - 0.7) / 0.3;
                lateral_offset = avoidance_lateral_distance_ * (0.5 * (1 + cos(M_PI * t)));
            }
            
            point.x = start_x + forward_distance * cos(start_yaw) + lateral_offset * cos(start_yaw + M_PI/2);
            point.y = start_y + forward_distance * sin(start_yaw) + lateral_offset * sin(start_yaw + M_PI/2);
            point.yaw = start_yaw;
            point.velocity = default_speed_ * 0.8;
            point.is_safe = !isPointOccupied(point.x, point.y);
            path.push_back(point);
        }
        return path;
    }

    std::vector<PathPoint> generateStoppingPath(double start_x, double start_y, double start_yaw)
    {
        std::vector<PathPoint> path;
        double stopping_distance = 3.0;
        double step_distance = stopping_distance / trajectory_points_;
        
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            double progress = (double)i / trajectory_points_;
            double distance = i * step_distance;
            
            point.x = start_x + distance * cos(start_yaw);
            point.y = start_y + distance * sin(start_yaw);
            point.yaw = start_yaw;
            point.velocity = default_speed_ * (1.0 - progress);
            point.is_safe = true;
            path.push_back(point);
        }
        return path;
    }

    // ========== 检查点是否被占用 ==========
    bool isPointOccupied(double x, double y)
    {
        if (!has_map_) return false;
        
        int grid_x = (x - current_map_.info.origin.position.x) / current_map_.info.resolution;
        int grid_y = (y - current_map_.info.origin.position.y) / current_map_.info.resolution;
        
        if (grid_x < 0 || grid_x >= current_map_.info.width || 
            grid_y < 0 || grid_y >= current_map_.info.height) {
            return false;
        }
        
        int index = grid_y * current_map_.info.width + grid_x;
        if (index >= current_map_.data.size()) return false;
        
        return current_map_.data[index] > 70;
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
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

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
            pose_stamped.pose.position.z = 0.05;

            tf2::Quaternion q;
            q.setRPY(0, 0, point.yaw);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_.publish(path_msg);
    }

    // ========== 修复：地图可视化显著优化 ==========
    void publishMapVisualization()
    {
        if (!has_map_) {
            ROS_WARN_THROTTLE(5.0, "No map available for visualization");
            return;
        }
        
        visualization_msgs::Marker map_marker;
        map_marker.header.frame_id = "map";
        map_marker.header.stamp = ros::Time::now();
        map_marker.ns = "occupancy_map";
        map_marker.id = 0;
        map_marker.type = visualization_msgs::Marker::CUBE_LIST;
        map_marker.action = visualization_msgs::Marker::ADD;

        // 增大立方体尺寸以便观察
        map_marker.scale.x = current_map_.info.resolution * 2.0;  // 放大2倍
        map_marker.scale.y = current_map_.info.resolution * 2.0;
        map_marker.scale.z = 0.5; // 增加高度

        // 优化采样策略 - 降低采样率但确保覆盖
        int sample_rate = std::max(1, (int)(current_map_.info.width / 100)); // 保证至少100个采样点
        int points_added = 0;
        int occupied_points = 0;

        ROS_INFO_THROTTLE(10.0, "Map info: %dx%d, resolution=%.3f, origin=(%.2f,%.2f), sample_rate=%d", 
                         current_map_.info.width, current_map_.info.height, current_map_.info.resolution,
                         current_map_.info.origin.position.x, current_map_.info.origin.position.y, sample_rate);

        for (int y = 0; y < current_map_.info.height; y += sample_rate) {
            for (int x = 0; x < current_map_.info.width; x += sample_rate) {
                int index = y * current_map_.info.width + x;
                if (index >= current_map_.data.size()) continue;

                int8_t value = current_map_.data[index];
                
                // 显示所有非未知区域
                if (value < 0) continue; // 跳过未知区域
                
                geometry_msgs::Point point;
                point.x = current_map_.info.origin.position.x + x * current_map_.info.resolution;
                point.y = current_map_.info.origin.position.y + y * current_map_.info.resolution;
                point.z = 0.25; // 抬高一些

                std_msgs::ColorRGBA color;
                if (value > 80) {
                    // 高占用区域 - 亮红色
                    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
                    occupied_points++;
                } else if (value > 50) {
                    // 中等占用 - 橙色
                    color.r = 1.0; color.g = 0.5; color.b = 0.0; color.a = 0.8;
                    occupied_points++;
                } else if (value <= 20) {
                    // 自由区域 - 绿色，但透明度低
                    color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.3;
                } else {
                    // 中间值 - 黄色
                    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 0.6;
                }

                map_marker.points.push_back(point);
                map_marker.colors.push_back(color);
                points_added++;
            }
        }

        ROS_INFO_THROTTLE(10.0, "Map visualization: %d total points, %d occupied points", 
                         points_added, occupied_points);
        
        if (points_added > 0) {
            map_visualization_pub_.publish(map_marker);
            ROS_INFO_THROTTLE(10.0, "✅ Published map visualization with %d cubes", points_added);
        } else {
            ROS_WARN_THROTTLE(5.0, "❌ No valid map points to visualize");
        }
    }

    void publishInflatedMap()
    {
        nav_msgs::OccupancyGrid inflated_map = current_map_;
        
        double inflation_radius = vehicle_width_ / 2.0 + safety_margin_;
        int inflation_cells = ceil(inflation_radius / current_map_.info.resolution);
        
        std::vector<int8_t> inflated_data = current_map_.data;
        
        for (int y = 0; y < current_map_.info.height; ++y) {
            for (int x = 0; x < current_map_.info.width; ++x) {
                int index = y * current_map_.info.width + x;
                
                if (current_map_.data[index] > 70) {
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (nx >= 0 && nx < current_map_.info.width &&
                                ny >= 0 && ny < current_map_.info.height) {
                                
                                double dist = sqrt(dx*dx + dy*dy) * current_map_.info.resolution;
                                if (dist <= inflation_radius) {
                                    int new_index = ny * current_map_.info.width + nx;
                                    if (inflated_data[new_index] < 70) {
                                        inflated_data[new_index] = 90;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        inflated_map.data = inflated_data;
        inflated_map.header.stamp = ros::Time::now();
        inflated_map_pub_.publish(inflated_map);
    }

    // ========== 新增：车辆模型可视化 ==========
    void publishVehicleModel()
    {
        visualization_msgs::Marker vehicle_marker;
        vehicle_marker.header.frame_id = "base_link";  // 相对于车辆坐标系
        vehicle_marker.header.stamp = ros::Time::now();
        vehicle_marker.ns = "vehicle";
        vehicle_marker.id = 0;
        vehicle_marker.type = visualization_msgs::Marker::CUBE;
        vehicle_marker.action = visualization_msgs::Marker::ADD;

        // 车辆模型位置（在base_link中心）
        vehicle_marker.pose.position.x = 0.0;
        vehicle_marker.pose.position.y = 0.0;
        vehicle_marker.pose.position.z = 0.5;
        vehicle_marker.pose.orientation.w = 1.0;

        // 车辆尺寸（根据接口文档）
        vehicle_marker.scale.x = 4.4;  // 长度
        vehicle_marker.scale.y = 1.8;  // 宽度
        vehicle_marker.scale.z = 1.0;  // 高度

        // 车辆颜色 - 根据状态变化
        if (current_behavior_ == "STRAIGHT") {
            vehicle_marker.color.r = 0.0; vehicle_marker.color.g = 0.8; vehicle_marker.color.b = 1.0;
        } else if (current_behavior_ == "AVOIDANCE") {
            vehicle_marker.color.r = 1.0; vehicle_marker.color.g = 0.6; vehicle_marker.color.b = 0.0;
        } else if (current_behavior_ == "EMERGENCY_STOP") {
            vehicle_marker.color.r = 1.0; vehicle_marker.color.g = 0.0; vehicle_marker.color.b = 0.0;
        } else {
            vehicle_marker.color.r = 0.5; vehicle_marker.color.g = 0.5; vehicle_marker.color.b = 0.5;
        }
        vehicle_marker.color.a = 0.8;

        vehicle_model_pub_.publish(vehicle_marker);
    }

    // ========== 路径可视化优化 ==========
    void publishPathVisualization()
    {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "planned_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;

        path_marker.scale.x = 0.3;  // 增加线条粗细

        if (current_behavior_ == "STRAIGHT") {
            path_marker.color.r = 0.0; path_marker.color.g = 0.8; path_marker.color.b = 1.0;
        } else if (current_behavior_ == "AVOIDANCE") {
            path_marker.color.r = 1.0; path_marker.color.g = 0.6; path_marker.color.b = 0.0;
        } else if (current_behavior_ == "EMERGENCY_STOP") {
            path_marker.color.r = 1.0; path_marker.color.g = 0.0; path_marker.color.b = 0.0;
        }
        path_marker.color.a = 1.0;  // 完全不透明

        for (const auto& point : current_path_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.3;  // 抬高路径线
            path_marker.points.push_back(p);
        }

        path_visualization_pub_.publish(path_marker);
        
        // 额外发布路径点球体
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "path_points";
        points_marker.id = 1;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        
        points_marker.scale.x = 0.4;
        points_marker.scale.y = 0.4;
        points_marker.scale.z = 0.4;
        
        for (size_t i = 0; i < current_path_.size(); ++i) {
            geometry_msgs::Point p;
            p.x = current_path_[i].x;
            p.y = current_path_[i].y;
            p.z = 0.5;
            points_marker.points.push_back(p);
            
            std_msgs::ColorRGBA color;
            if (current_path_[i].is_safe) {
                color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.8;
            } else {
                color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 0.8;
            }
            points_marker.colors.push_back(color);
        }
        
        path_visualization_pub_.publish(points_marker);
    }

    void publishBehaviorIndicator()
    {
        visualization_msgs::Marker indicator;
        indicator.header.frame_id = "base_link";  // 相对于车辆
        indicator.header.stamp = ros::Time::now();
        indicator.ns = "behavior_indicator";
        indicator.id = 0;
        indicator.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        indicator.action = visualization_msgs::Marker::ADD;

        // 显示在车辆上方
        indicator.pose.position.x = 0.0;
        indicator.pose.position.y = 0.0;
        indicator.pose.position.z = 3.0;  // 车辆上方3米
        indicator.pose.orientation.w = 1.0;

        indicator.scale.z = 1.0;  // 增大文字

        std::string text = "🚗 PLANNING: " + current_behavior_;
        if (current_behavior_ == "STRAIGHT") {
            text += "\n➡️  Normal Driving";
            indicator.color.r = 0.0; indicator.color.g = 1.0; indicator.color.b = 0.0;
        } else if (current_behavior_ == "AVOIDANCE") {
            text += "\n🔀 Obstacle Avoidance";
            indicator.color.r = 1.0; indicator.color.g = 0.6; indicator.color.b = 0.0;
        } else if (current_behavior_ == "EMERGENCY_STOP") {
            text += "\n🛑 Emergency Stop";
            indicator.color.r = 1.0; indicator.color.g = 0.0; indicator.color.b = 0.0;
        }
        indicator.color.a = 1.0;

        text += "\n📍 Pos: (" + std::to_string((int)current_x_) + "," + std::to_string((int)current_y_) + ")";
        text += "\n🎯 Points: " + std::to_string(current_path_.size());
        
        if (has_map_) {
            text += "\n🗺️  Map: " + std::to_string(current_map_.info.width) + "x" + std::to_string(current_map_.info.height);
        }

        indicator.text = text;
        behavior_indicator_pub_.publish(indicator);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_node");
    
    PlanningNode planning_node;
    
    ROS_INFO("===== Enhanced Planning Node Ready =====");
    ROS_INFO("📡 Subscribed Topics:");
    ROS_INFO("  - /decision/behavior_command");
    ROS_INFO("  - /perception/occupancy_grid");
    ROS_INFO("📤 Published Topics:");
    ROS_INFO("  - /planning/trajectory");
    ROS_INFO("  - /planning/path");
    ROS_INFO("  - /planning/map_visualization");
    ROS_INFO("  - /planning/inflated_map");
    ROS_INFO("  - /planning/path_visualization");
    ROS_INFO("  - /planning/behavior_indicator");
    ROS_INFO("  - /planning/vehicle_model");
    ROS_INFO("🔧 Features:");
    ROS_INFO("  ✅ RViz auto-follow via dynamic TF");
    ROS_INFO("  ✅ Enhanced map visualization");
    ROS_INFO("  ✅ Real-time vehicle model");
    ROS_INFO("  ✅ Path safety indicators");
    ROS_INFO("========================================");
    
    ros::spin();
    
    return 0;
}

    //