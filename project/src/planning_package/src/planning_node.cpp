#include <ros/ros.h>
#include <ros/package.h> 
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
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
#include <algorithm>
#include <limits>
#include <fstream>
#include <nlohmann/json.hpp>

//------------------------------------------------------------
// Path / Trail structs (unchanged)
//------------------------------------------------------------
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

//------------------------------------------------------------
// Global waypoint struct
//------------------------------------------------------------
struct GlobalWp {
    double x{0}, y{0}, yaw{0};
};

//------------------------------------------------------------
// Helper utilities
//------------------------------------------------------------
static inline double wrapAngle(double a){
    while (a > M_PI)  a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}

//------------------------------------------------------------
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
        // 感知模块 Occupancy Grid (用于 DWA clearance)
        occupancy_grid_sub_ = nh_.subscribe("/perception/occupancy_grid", 1,
                                           &VehicleFollowingPlanningNode::occupancyGridCallback, this);
        
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
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/global_path", 1, true); // 新增: 全局路径

        // ========== 参数设置 ==========
        nh_.param("planning_frequency", planning_frequency_, 10.0);
        nh_.param("lookahead_distance", lookahead_distance_, 20.0);  // 前瞻距离 (保留, fallback / 可视化)
        nh_.param("trajectory_points", trajectory_points_, 25);      // 发布点数量 (用于速度采样)
        nh_.param("default_speed", default_speed_, 8.0);             // 默认速度 m/s
        nh_.param("emergency_decel", emergency_decel_, 3.0);         // 紧急减速度
        nh_.param("trail_max_points", trail_max_points_, 200);
        nh_.param("trail_update_distance", trail_update_distance_, 0.5);

        // --- 全局航点参数 ---
        if (!pnh_.getParam("global_waypoints_file", wp_file_))      // 先找私有  ~
            nh_.param("global_waypoints_file", wp_file_, std::string("waypoints.json"));  // 再找全局
        ROS_INFO_STREAM("[Param] global_waypoints_file = " << wp_file_);
        nh_.param("global_interp_ds", ds_global_, 1.0);
        nh_.param("local_lookahead_m", local_lookahead_m_, 20.0);
        nh_.param("wp_reach_thresh", wp_reach_thresh_, 3.0);

        // --- DWA 参数 ---
        nh_.param("dwa/dt", dwa_dt_, 0.1);
        nh_.param("dwa/predict_time", dwa_predict_time_, 3.0);
        nh_.param("dwa/v_samples", dwa_v_samples_, 5);
        nh_.param("dwa/w_samples", dwa_w_samples_, 5);
        nh_.param("dwa/acc_v", dwa_acc_v_, 3.0);
        nh_.param("dwa/acc_w", dwa_acc_w_, 1.5);
        nh_.param("dwa/v_min", dwa_v_min_param_, 0.0);
        double v_max_param_default = default_speed_;
        nh_.param("dwa/v_max", dwa_v_max_param_, v_max_param_default);
        nh_.param("dwa/w_min", dwa_w_min_param_, -0.6);
        nh_.param("dwa/w_max", dwa_w_max_param_, 0.6);
        nh_.param("dwa/cost_heading", dwa_cost_heading_, 1.0);
        nh_.param("dwa/cost_clear", dwa_cost_clear_, 2.0);
        nh_.param("dwa/cost_vel", dwa_cost_vel_, 0.5);
        nh_.param("dwa/obstacle_range", dwa_obstacle_range_, 30.0);
        nh_.param("dwa/inflation_radius", dwa_inflation_radius_, 1.5);
        nh_.param("dwa/cost_path", dwa_cost_path_, 2.0);   // 可在 YAML 调


        // ========== 状态初始化 ==========
        vehicle_pose_received_ = false;
        is_emergency_stop_ = false;
        behavior_command_ = "normal";  // 默认正常行为
        current_speed_ = 0.0;
        current_yaw_rate_ = 0.0;
        
        // 避让状态初始化 (保留, 供信息显示)
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

        // 地图状态
        occupancy_grid_received_ = false;

        // 加载全局航点 (若失败则使用直线 fallback)
        if (loadWaypointsFromJson(wp_file_)) {
            densifyGlobalPath();
            publishGlobalPath();
            global_loaded_ = true;
            ROS_INFO(" Global waypoints loaded: %zu sparse → %zu dense", global_sparse_.size(), global_dense_.size());
        } else {
            global_loaded_ = false;
            ROS_ERROR(" Could not load waypoints file '%s'. Will use straight-line fallback.", wp_file_.c_str());
        }

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
    ros::NodeHandle pnh_{"~"}; 
    // 订阅器
    ros::Subscriber vehicle_pose_sub_;
    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber occupancy_grid_sub_; // 新增
    ros::Subscriber behavior_command_sub_;
    ros::Subscriber emergency_stop_sub_;
    
    // 发布器
    ros::Publisher trajectory_pub_;
    ros::Publisher path_pub_;
    ros::Publisher planned_path_vis_pub_;
    ros::Publisher vehicle_trail_pub_;
    ros::Publisher planning_info_pub_;
    ros::Publisher global_path_pub_; // 新增
    
    // 定时器
    ros::Timer planning_timer_;
    ros::Timer visualization_timer_;

    // ========== 状态变量 ==========
    bool vehicle_pose_received_;
    bool is_emergency_stop_;        // 来自决策模块的紧急停车指令
    std::string behavior_command_;  // 来自决策模块的行为指令
    double current_speed_;
    double current_yaw_rate_;
    
    // 避让状态记录 (保留用于状态显示)
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

    // --- 全局航点参数 ---
    std::string wp_file_;
    double ds_global_;
    double local_lookahead_m_;
    double wp_reach_thresh_;
    bool   global_loaded_{false};
    
    std::vector<GlobalWp> global_sparse_;
    std::vector<GlobalWp> global_dense_;
    std::vector<GlobalWp> global_dense_active_;  // 新增：仅保留当前有效路径段
    size_t dense_progress_idx_{0};   // 新增
    const double dense_reach_thresh_ = 2.0;
    size_t global_progress_idx_{0};

    // --- DWA 参数 ---
    double dwa_dt_;
    double dwa_predict_time_;
    int    dwa_v_samples_;
    int    dwa_w_samples_;
    double dwa_acc_v_;
    double dwa_acc_w_;
    double dwa_v_min_param_;
    double dwa_v_max_param_;
    double dwa_w_min_param_;
    double dwa_w_max_param_;
    double dwa_cost_heading_;
    double dwa_cost_clear_;
    double dwa_cost_vel_;
    double dwa_obstacle_range_;
    double dwa_inflation_radius_;
    double dwa_cost_path_;   // 新增：路径贴合权重


    // 地图
    nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;
    bool occupancy_grid_received_;

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
        // 线速度幅值
        current_speed_ = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) +
                                   std::pow(msg->twist.twist.linear.y, 2));
        current_yaw_rate_ = msg->twist.twist.angular.z;
    }

    void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& msg)
    {
        occupancy_grid_ = msg;
        occupancy_grid_received_ = true;
    }

    void behaviorCommandCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (behavior_command_ != msg->data) {
            last_behavior_command_ = behavior_command_;
            behavior_command_ = msg->data;
            
            // 保留日志与状态标记
            if ((behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") &&
                (last_behavior_command_ != "avoid_left" && last_behavior_command_ != "avoid_right")) {
                is_avoiding_ = true;
                avoid_start_time_ = ros::Time::now().toSec();
                avoid_offset_ = (behavior_command_ == "avoid_left") ? 3.0 : -3.0;  // 仅用于显示
                ROS_INFO("🎯 Starting Avoidance: %s (offset: %.1fm)", behavior_command_.c_str(), avoid_offset_);
                generateAndPublishPath();
            }
            else if ((last_behavior_command_ == "avoid_left" || last_behavior_command_ == "avoid_right") &&
                     behavior_command_ == "normal") {
                is_avoiding_ = false;
                avoid_offset_ = 0.0;
                ROS_INFO("✅ Avoidance Complete, returning to normal");
                generateAndPublishPath();
            }
            
            ROS_INFO("🎭 Behavior Command: %s → %s", last_behavior_command_.c_str(), behavior_command_.c_str());
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
        double distance_from_last = std::sqrt(std::pow(x - last_trail_x_, 2) + std::pow(y - last_trail_y_, 2));
        
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
    void planningTimerCallback(const ros::TimerEvent& /*event*/)
    {
        if (!vehicle_pose_received_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for vehicle pose...");
            return;
        }
        
        generateAndPublishPath();
    }

    void visualizationTimerCallback(const ros::TimerEvent& /*event*/)
    {
        if (vehicle_pose_received_) {
            publishPlannedPathVisualization();
            publishVehicleTrailVisualization();
            publishPlanningInfo();
        }
    }

    // ========================================================
    // 路径规划主入口
    // ========================================================
    void generateAndPublishPath()
    {
        std::vector<PathPoint> path = generateForwardPath();
        current_planned_path_ = path;
        
        publishTrajectory(path);
        publishPath(path);
        
        ROS_DEBUG("Generated path with %lu points", path.size());
    }

    // ========================================================
    // 新版前向路径生成：优先使用 全局航点 + DWA
    // 当全局未加载或 DWA 失败时，退回原直线 (无 S 避障)
    // ========================================================
    std::vector<PathPoint> generateForwardPath()
    {
        if (is_emergency_stop_) {
            return buildEmergencyStopPath();
        }
        
        if (global_loaded_ && vehicle_pose_received_) {
            // 更新稀疏目标进度 (仅用于统计)
            while (global_progress_idx_+1 < global_sparse_.size()){
                double dx = current_x_ - global_sparse_[global_progress_idx_].x;
                double dy = current_y_ - global_sparse_[global_progress_idx_].y;
                if (std::hypot(dx,dy) < wp_reach_thresh_) ++global_progress_idx_; else break;
            }
            // 1）找离当前位置最近的稠密点索引
            size_t nearest_dense = findNearestDenseIdx();

            // 2）只有当它在 “当前进度点之后” 且真到达了，才推进
            double dense_dx = current_x_ - global_dense_[dense_progress_idx_].x;
            double dense_dy = current_y_ - global_dense_[dense_progress_idx_].y;
            if (nearest_dense > dense_progress_idx_ &&
                std::hypot(dense_dx, dense_dy) < dense_reach_thresh_) {
                dense_progress_idx_ = nearest_dense;
            }

            // 3）用 dense_progress_idx_ 来裁剪 & 生成 active 段
            size_t dense_start = dense_progress_idx_;
            size_t dense_end   = std::min(dense_start + 200, global_dense_.size());
            global_dense_active_.assign(global_dense_.begin()+dense_start,
                            global_dense_.begin()+dense_end);


            GlobalWp goal = pickLocalGoal(); // 根据行为指令自动横向偏移
            std::vector<PathPoint> dwa_path = runDWA(goal);
            if (!dwa_path.empty()) {
                applyBehaviorSpeedMod(dwa_path); // 根据行为指令调速
                return dwa_path;
            }
            ROS_WARN_THROTTLE(2.0, "DWA returned empty path, falling back to straight-line.");
        }
        
        // Fallback (原 S-避障逻辑删除 → 简单直线)
        return generateStraightFallbackPath();
    }

    //--------------------------------------------------------
    // 行为指令速度调节
    //--------------------------------------------------------
    void applyBehaviorSpeedMod(std::vector<PathPoint>& path)
    {
        double scale = 1.0;
        if (behavior_command_ == "slow_down") scale = 0.5;
        else if (behavior_command_ == "avoid_left" || behavior_command_ == "avoid_right") scale = 0.7;
        for (auto &pt : path) pt.velocity *= scale;
    }

    //--------------------------------------------------------
    // 构造紧急停车轨迹：沿当前朝向减速至 0
    //--------------------------------------------------------
    std::vector<PathPoint> buildEmergencyStopPath()
    {
        std::vector<PathPoint> path;
        // 估算制动距离 s = v^2 / (2a)
        double v0 = current_speed_;
        double a = std::max(0.1, emergency_decel_); // 避免除零
        double stop_dist = (v0*v0) / (2.0 * a);
        if (stop_dist < 1.0) stop_dist = 1.0; // 至少 1m 方便可视化

        int N = trajectory_points_;
        double ds = stop_dist / N;
        for (int i=1;i<=N;++i){
            double d = i*ds;
            PathPoint pt;
            pt.x = current_x_ + d*std::cos(current_yaw_);
            pt.y = current_y_ + d*std::sin(current_yaw_);
            pt.yaw = current_yaw_;
            double ratio = double(i)/N;
            pt.velocity = v0 * std::exp(-ratio * emergency_decel_); // 指数衰减
            if (pt.velocity < 0.2) pt.velocity = 0.0;
            pt.timestamp = ros::Time::now();
            path.push_back(pt);
        }
        return path;
    }

    //--------------------------------------------------------
    // 简单直线路径 (无 S 避障) Fallback
    //--------------------------------------------------------
    std::vector<PathPoint> generateStraightFallbackPath()
    {
        std::vector<PathPoint> path;
        double step_distance = lookahead_distance_ / trajectory_points_;
        double target_speed = calculateTargetSpeed();
        for (int i = 1; i <= trajectory_points_; ++i) {
            PathPoint point;
            double distance = i * step_distance;
            point.x = current_x_ + distance * std::cos(current_yaw_);
            point.y = current_y_ + distance * std::sin(current_yaw_);
            point.yaw = current_yaw_;
            point.velocity = target_speed;
            point.timestamp = ros::Time::now();
            path.push_back(point);
        }
        return path;
    }

    //--------------------------------------------------------
    // 目标速度 (保留原逻辑)
    //--------------------------------------------------------
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

    //--------------------------------------------------------
    // ========== 发布函数 ==========
    //--------------------------------------------------------
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
            // 时间戳字段: 简化为采样步 (s)
            traj_msg.timestamps.push_back(i * dwa_dt_);
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
        
        // 根据决策状态设置颜色 (保留原配色)
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
        
        ROS_INFO_THROTTLE(2.0, "🎨 Publishing path visualization: behavior=%s, points=%lu",
                         behavior_command_.c_str(), current_planned_path_.size());
        
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
        info_marker.color.r = 1.0;
        info_marker.color.g = 1.0;
        info_marker.color.b = 1.0;
        info_marker.color.a = 1.0;
        
        std::string info_text;
        info_text.reserve(256);
        info_text = "Planning Node";

        // C++ 字符串要显式写换行符: "\n". 为避免编码问题, 这里使用 ASCII 文本.
        info_text += "\nPos: (" + std::to_string(static_cast<int>(current_x_)) + "," +
                    std::to_string(static_cast<int>(current_y_)) + ")";
        info_text += "\nSpeed: " + std::to_string(static_cast<int>(current_speed_ * 3.6)) + " km/h";
        info_text += "\nPath: " + std::to_string(current_planned_path_.size()) + " pts";
        info_text += "\nTrail: " + std::to_string(trail_points_.size()) + " pts";
        info_text += "\nBehavior: " + behavior_command_;

        // 状态指示 & 颜色
        if (is_emergency_stop_) {
            info_text += "\nEMERGENCY STOP";
            info_marker.color.r = 1.0; info_marker.color.g = 0.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "avoid_left") {
            info_text += "\nAVOIDING LEFT";
            info_marker.color.r = 1.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "avoid_right") {
            info_text += "\nAVOIDING RIGHT";
            info_marker.color.r = 1.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        } else if (behavior_command_ == "slow_down") {
            info_text += "\nSLOWING DOWN";
            info_marker.color.r = 1.0; info_marker.color.g = 0.5; info_marker.color.b = 0.0;
        } else {
            info_text += "\nNORMAL DRIVING";
            info_marker.color.r = 0.0; info_marker.color.g = 1.0; info_marker.color.b = 0.0;
        }
        info_marker.color.a = 1.0;

        info_marker.text = info_text;
        planning_info_pub_.publish(info_marker);
    }
    // ========================================================
    // --------- 全局航点 / 路径函数 --------------------------
    // ========================================================
    bool loadWaypointsFromJson(const std::string& filename)
    {
        ROS_INFO_STREAM("[WpLoader] trying to open file: " << filename);
        std::ifstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }

        // ✅ 直接读成 string 再 parse，避免重复 seekg
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();
        ROS_INFO_STREAM("[WpLoader] File content begins:\n" << content);

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(content);
        } catch (nlohmann::json::parse_error &e) {
            ROS_ERROR("JSON parse error in %s : %s", filename.c_str(), e.what());
            return false;
        }

    
       

        for (auto &w : j) {
            GlobalWp p;
            p.x = w["position"]["x"].get<double>();
            p.y = w["position"]["y"].get<double>();
            tf2::Quaternion q(
                w["orientation"]["x"].get<double>(),
                w["orientation"]["y"].get<double>(),
                w["orientation"]["z"].get<double>(),
                w["orientation"]["w"].get<double>());
            double roll,pitch,yaw; tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
            p.yaw = yaw;
            global_sparse_.push_back(p);
        }
        return !global_sparse_.empty();
    }

    void densifyGlobalPath()
    {
        global_dense_.clear();
        if (global_sparse_.empty()) return;
        for (size_t i=0;i+1<global_sparse_.size();++i){
            const auto &a = global_sparse_[i];
            const auto &b = global_sparse_[i+1];
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double dist = std::hypot(dx,dy);
            size_t steps = std::max<size_t>(1, std::floor(dist / ds_global_));
            for (size_t k=0;k<steps;++k){
                double t = static_cast<double>(k) / steps;
                GlobalWp p;
                p.x = a.x + t*dx;
                p.y = a.y + t*dy;
                p.yaw = std::atan2(dy,dx);
                global_dense_.push_back(p);
            }
        }
        global_dense_.push_back(global_sparse_.back());
    }

    void publishGlobalPath()
    {
        if (global_dense_.empty()) return;
        nav_msgs::Path msg; msg.header.frame_id="map"; msg.header.stamp=ros::Time::now();
        for (const auto &p : global_dense_){
            geometry_msgs::PoseStamped ps; ps.header = msg.header;
            ps.pose.position.x = p.x; ps.pose.position.y = p.y; ps.pose.position.z = 0.0;
            tf2::Quaternion q; q.setRPY(0,0,p.yaw); ps.pose.orientation = tf2::toMsg(q);
            msg.poses.push_back(ps);
        }
        global_path_pub_.publish(msg);
    }

    // 最近 idx
    size_t findNearestDenseIdx() const
    {
        size_t nearest = 0; double best = std::numeric_limits<double>::max();
        for (size_t i=0;i<global_dense_.size();++i){
            double d = std::hypot(global_dense_[i].x - current_x_, global_dense_[i].y - current_y_);
            if (d < best){ best = d; nearest = i; }
        }
        return nearest;
    }
    // 从指定的 global_sparse_ 索引出发，找在 global_dense_ 中最接近该航点的索引
    size_t findNearestDenseIdxFrom(size_t sparse_idx) const {
        if (global_sparse_.empty() || global_dense_.empty()) return 0;
        const auto& ref = global_sparse_[sparse_idx];
        size_t best_idx = 0;
        double best_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < global_dense_.size(); ++i) {
            double d = std::hypot(ref.x - global_dense_[i].x, ref.y - global_dense_[i].y);
            if (d < best_dist) {
                best_dist = d;
                best_idx = i;
            }
        }
        return best_idx;
    }

    GlobalWp pickLocalGoal()
    {
        size_t nearest = findNearestDenseIdx();
        
        // 沿稠密路径前瞻
        double acc = 0.0; size_t goal_idx = nearest;
        for (size_t i=nearest; i+1<global_dense_.size(); ++i){
            double seg = std::hypot(global_dense_[i+1].x - global_dense_[i].x,
                                    global_dense_[i+1].y - global_dense_[i].y);
            acc += seg; goal_idx = i+1; if (acc >= local_lookahead_m_) break;
        }
        GlobalWp goal = global_dense_[goal_idx];
        // 行为指令横向偏移 (简化替代 S 型)
        double lateral = 0.0;
        if (behavior_command_ == "avoid_left") lateral = +4.0; // 可参数化
        else if (behavior_command_ == "avoid_right") lateral = -4.0;
        if (std::fabs(lateral) > 1e-3){
            // 法向方向 (yaw + 90deg)
            goal.x += lateral * std::cos(goal.yaw + M_PI/2.0);
            goal.y += lateral * std::sin(goal.yaw + M_PI/2.0);
        }
        return goal;
    }

    //--------------------------------------------------------
    // DWA Implementation (unicycle model)
    //--------------------------------------------------------
    std::vector<PathPoint> runDWA(const GlobalWp& goal)
    {
        // 构建障碍点集
        std::vector<std::pair<double,double>> obstacles;
        if (occupancy_grid_received_ && occupancy_grid_){
            const auto &g = *occupancy_grid_;
            double res = g.info.resolution;
            double org_x = g.info.origin.position.x;
            double org_y = g.info.origin.position.y;
            int w = g.info.width;
            int h = g.info.height;
            double range2 = dwa_obstacle_range_ * dwa_obstacle_range_;
            for (int iy=0; iy<h; ++iy){
                for (int ix=0; ix<w; ++ix){
                    int idx = iy*w + ix;
                    if (g.data[idx] > 50){
                        double wx = org_x + (ix + 0.5)*res;
                        double wy = org_y + (iy + 0.5)*res;
                        double dx = wx - current_x_;
                        double dy = wy - current_y_;
                        if (dx*dx + dy*dy <= range2){
                            obstacles.emplace_back(wx, wy);
                        }
                    }
                }
            }
        }

        // 动窗 (速度 / 角速度 限制)
        double v_min = std::max(dwa_v_min_param_, current_speed_ - dwa_acc_v_ * dwa_dt_);
        double v_max = std::min(dwa_v_max_param_, current_speed_ + dwa_acc_v_ * dwa_dt_);
        if (v_min < 0.0) v_min = 0.0; // 无倒车
        double w_min = std::max(dwa_w_min_param_, current_yaw_rate_ - dwa_acc_w_ * dwa_dt_);
        double w_max = std::min(dwa_w_max_param_, current_yaw_rate_ + dwa_acc_w_ * dwa_dt_);

        double best_cost = std::numeric_limits<double>::max();
        std::vector<PathPoint> best_traj;

        int vN = std::max(1, dwa_v_samples_);
        int wN = std::max(1, dwa_w_samples_);

        for (int i=0;i<=vN;++i){
            double v = v_min + (v_max - v_min) * i / double(vN);
            for (int j=0;j<=wN;++j){
                double w = w_min + (w_max - w_min) * j / double(wN);

                // rollout
                double x = current_x_;
                double y = current_y_;
                double yaw = current_yaw_;
                double t = 0.0;
                double min_obst_dist = std::numeric_limits<double>::max();
                std::vector<PathPoint> traj;
                while (t < dwa_predict_time_){
                    x += v * std::cos(yaw) * dwa_dt_;
                    y += v * std::sin(yaw) * dwa_dt_;
                    yaw = wrapAngle(yaw + w * dwa_dt_);
                    t += dwa_dt_;
                    PathPoint pt; pt.x=x; pt.y=y; pt.yaw=yaw; pt.velocity=v; pt.timestamp=ros::Time::now();
                    traj.push_back(pt);
                    if (!obstacles.empty()){
                        for (auto &ob : obstacles){
                            double d = std::hypot(x - ob.first, y - ob.second) - dwa_inflation_radius_;
                            if (d < min_obst_dist) min_obst_dist = d;
                        }
                    }
                }
                if (obstacles.empty()) min_obst_dist = 10.0; // no obstacles
                // ==== 修改后的路径贴合度（只匹配前方路径） ====
                

                double path_cost = 0.0;
                for (const auto &pt_i : traj) {
                    double best = std::numeric_limits<double>::max();
                    for (const auto &wp : global_dense_active_) {
                        double d = std::hypot(pt_i.x - wp.x, pt_i.y - wp.y);
                        if (d < best) best = d;
                    }
                    path_cost += best;
                }
                
                path_cost /= traj.size();


                // heading cost: dist from end to goal
                double hd_cost = std::hypot(x - goal.x, y - goal.y);
                // clearance cost: inverse of min distance (bigger distance => smaller cost)
                double clr_cost = (min_obst_dist > 0.0) ? 1.0 / min_obst_dist : 1e9;
                // velocity cost: prefer faster
                double vel_cost = dwa_v_max_param_ - v;
                double cost = dwa_cost_heading_ * hd_cost +dwa_cost_path_ * path_cost + dwa_cost_clear_ * clr_cost + dwa_cost_vel_ * vel_cost;

                if (cost < best_cost){
                    best_cost = cost;
                    best_traj = traj;
                }
            }
        }

        // 确保起点包含当前位姿
        if (!best_traj.empty()){
            PathPoint cur; cur.x=current_x_; cur.y=current_y_; cur.yaw=current_yaw_; cur.velocity=current_speed_; cur.timestamp=ros::Time::now();
            best_traj.insert(best_traj.begin(), cur);
        }
        return best_traj;
    }

}; // end class

//------------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "vehicle_following_planning_node");
    
    VehicleFollowingPlanningNode planning_node;
    
    ROS_INFO("===== Vehicle Following Planning Node Ready =====");
    ROS_INFO("📡 Subscribed Topics:");
    ROS_INFO("  - /vehicle/pose (车辆位置)");
    ROS_INFO("  - /vehicle/odometry (车辆速度)");
    ROS_INFO("  - /perception/occupancy_grid (感知栅格)");
    ROS_INFO("  - /decision/behavior_command (行为指令)");
    ROS_INFO("  - /decision/emergency_stop (紧急停车)");
    ROS_INFO("📤 Published Topics:");
    ROS_INFO("  - /planning/trajectory (轨迹)");
    ROS_INFO("  - /planning/path (路径)");
    ROS_INFO("  - /planning/planned_path_visualization (前方路径可视化)");
    ROS_INFO("  - /planning/vehicle_trail (车辆轨迹可视化)");
    ROS_INFO("  - /planning/info_display (规划信息显示)");
    ROS_INFO("  - /planning/global_path (全局路径)");
    ROS_INFO("🎯 Features:");
    ROS_INFO("  ✅ 全局航点 + 局部DWA");
    ROS_INFO("  ✅ 保留原可视化与速度行为接口");
    ROS_INFO("  ✅ 紧急停车、减速、避让映射到DWA");
    ROS_INFO("================================================");
    
    ros::spin();
    
    return 0;
}
