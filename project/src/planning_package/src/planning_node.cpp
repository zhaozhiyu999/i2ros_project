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
        
        // 使用简单的位置跟踪，不依赖外部TF
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;

        // ========== 启动定时器 ==========
        planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_),
                                         &PlanningNode::planningTimerCallback, this);
        
        visualization_timer_ = nh_.createTimer(ros::Duration(0.5),  // 2Hz可视化更新
                                              &PlanningNode::visualizationTimerCallback, this);

        ROS_INFO("Planning Node started with integrated visualization");
        ROS_INFO("Planning frequency: %.1f Hz, Lookahead: %.1f m", planning_frequency_, lookahead_distance_);
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
    
    ros::Timer planning_timer_;
    ros::Timer visualization_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // ========== 状态变量 ==========
    std::string current_behavior_;
    nav_msgs::OccupancyGrid current_map_;
    bool has_map_;
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

    // ========== 立即设置静态TF ========== 
    void setupStaticTransforms()
    {
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
        
        // 等待TF可用
        ros::Duration(0.5).sleep();
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
    }

    // ========== 定时器回调 ==========
    void planningTimerCallback(const ros::TimerEvent& event)
    {
        generateAndPublishPath();
    }

    void visualizationTimerCallback(const ros::TimerEvent& event)
    {
        if (has_map_) {
            publishMapVisualization();
            publishInflatedMap();
        }
        
        if (!current_path_.empty()) {
            publishPathVisualization();
            publishBehaviorIndicator();
        }
    }

    // ========== 简化的位置获取 ==========
    bool getCurrentPose(double& x, double& y, double& yaw)
    {
        // 尝试从TF获取，如果失败则使用模拟位置
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
            
            return true;
            
        } catch (tf2::TransformException& ex) {
            // TF不可用，使用模拟的车辆移动
            static double sim_time = 0.0;
            sim_time += 0.2;  // 200ms步长
            
            // 简单的圆形路径模拟
            current_x_ = 5.0 * cos(sim_time * 0.1);
            current_y_ = 5.0 * sin(sim_time * 0.1);
            current_yaw_ = sim_time * 0.1 + M_PI/2;
            
            x = current_x_;
            y = current_y_;
            yaw = current_yaw_;
            
            ROS_DEBUG_THROTTLE(5.0, "Using simulated vehicle position: (%.2f, %.2f, %.2f)", x, y, yaw);
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

    // ========== 地图可视化 ==========
    void publishMapVisualization()
        {
            if (!has_map_) {
                ROS_WARN_THROTTLE(2.0, "No map available for visualization");
                return;
            }
            
            ROS_INFO_THROTTLE(5.0, "Publishing map visualization...");
            ROS_INFO_THROTTLE(5.0, "Map: %dx%d, resolution=%.3f, origin=(%.2f, %.2f)", 
                    current_map_.info.width, current_map_.info.height, 
                    current_map_.info.resolution,
                    current_map_.info.origin.position.x, 
                    current_map_.info.origin.position.y);
            
            visualization_msgs::Marker map_marker;
            map_marker.header.frame_id = "map";
            map_marker.header.stamp = ros::Time::now();
            map_marker.ns = "occupancy_map";
            map_marker.id = 0;
            map_marker.type = visualization_msgs::Marker::CUBE_LIST;
            map_marker.action = visualization_msgs::Marker::ADD;

            // 使用较大的立方体以便观察
            map_marker.scale.x = current_map_.info.resolution;
            map_marker.scale.y = current_map_.info.resolution;
            map_marker.scale.z = 0.2; // 稍微高一些

            // 降低采样率以减少计算量
            int sample_rate = 10; // 每10个像素采样一次
            int points_added = 0;

            for (int y = 0; y < current_map_.info.height; y += sample_rate) {
                for (int x = 0; x < current_map_.info.width; x += sample_rate) {
                    int index = y * current_map_.info.width + x;
                    if (index >= current_map_.data.size()) continue;

                    int8_t value = current_map_.data[index];
                    
                    // 只显示明确的占用和自由区域
                    if (value < 0) continue; // 跳过未知区域
                    
                    geometry_msgs::Point point;
                    point.x = current_map_.info.origin.position.x + x * current_map_.info.resolution;
                    point.y = current_map_.info.origin.position.y + y * current_map_.info.resolution;
                    point.z = 0.1;

                    std_msgs::ColorRGBA color;
                    if (value > 70) {
                        // 占用区域 - 明亮红色
                        color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
                    } else if (value <= 30) {
                        // 自由区域 - 明亮绿色
                        color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.6;
                    } else {
                        // 中间值 - 黄色
                        color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 0.8;
                    }

                    map_marker.points.push_back(point);
                    map_marker.colors.push_back(color);
                    points_added++;
                }
            }

            ROS_INFO_THROTTLE(5.0, "Map visualization: %d points added", points_added);
            
            if (points_added > 0) {
                map_visualization_pub_.publish(map_marker);
                ROS_INFO_THROTTLE(5.0, "Published map marker with %d cubes", points_added);
            } else {
                ROS_WARN_THROTTLE(5.0, "No valid map points to visualize");
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

    // ========== 路径可视化 ==========
    void publishPathVisualization()
    {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "planned_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;

        path_marker.scale.x = 0.2;

        if (current_behavior_ == "STRAIGHT") {
            path_marker.color.r = 0.0; path_marker.color.g = 0.8; path_marker.color.b = 1.0;
        } else if (current_behavior_ == "AVOIDANCE") {
            path_marker.color.r = 1.0; path_marker.color.g = 0.6; path_marker.color.b = 0.0;
        } else if (current_behavior_ == "EMERGENCY_STOP") {
            path_marker.color.r = 1.0; path_marker.color.g = 0.0; path_marker.color.b = 0.0;
        }
        path_marker.color.a = 0.9;

        for (const auto& point : current_path_) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.15;
            path_marker.points.push_back(p);
        }

        path_visualization_pub_.publish(path_marker);
    }

    void publishBehaviorIndicator()
    {
        visualization_msgs::Marker indicator;
        indicator.header.frame_id = "map";
        indicator.header.stamp = ros::Time::now();
        indicator.ns = "behavior_indicator";
        indicator.id = 0;
        indicator.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        indicator.action = visualization_msgs::Marker::ADD;

        indicator.pose.position.x = current_x_;
        indicator.pose.position.y = current_y_;
        indicator.pose.position.z = 2.0;
        indicator.pose.orientation.w = 1.0;

        indicator.scale.z = 0.8;

        std::string text = "PLANNING: " + current_behavior_;
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

        text += "\nPath Points: " + std::to_string(current_path_.size());
        text += "\nPos: (" + std::to_string((int)current_x_) + "," + std::to_string((int)current_y_) + ")";

        indicator.text = text;
        behavior_indicator_pub_.publish(indicator);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_node");
    
    PlanningNode planning_node;
    
    ROS_INFO("===== Planning Node Ready =====");
    ROS_INFO("Subscribed Topics:");
    ROS_INFO("  - /decision/behavior_command");
    ROS_INFO("  - /perception/occupancy_grid");
    ROS_INFO("Published Topics:");
    ROS_INFO("  - /planning/trajectory");
    ROS_INFO("  - /planning/path");
    ROS_INFO("  - /planning/map_visualization");
    ROS_INFO("  - /planning/inflated_map");
    ROS_INFO("  - /planning/path_visualization");
    ROS_INFO("  - /planning/behavior_indicator");
    ROS_INFO("================================");
    
    ros::spin();
    
    return 0;
}