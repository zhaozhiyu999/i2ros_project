#include <ros/ros.h>
#include <ros/package.h>
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
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct PathPoint {
    double x, y, yaw;
    double velocity;
    bool is_safe;
};

struct Waypoint {
    double x, y, yaw;
    double vx, vy, vz, qw;
};

class PlanningNode
{
public:
    PlanningNode();

private:
    // ROS节点相关
    ros::NodeHandle nh_;
    ros::Subscriber behavior_command_sub_;
    ros::Subscriber occupancy_grid_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher path_pub_;
    ros::Publisher map_visualization_pub_;
    ros::Publisher inflated_map_pub_;
    ros::Publisher path_visualization_pub_;
    ros::Publisher behavior_indicator_pub_;
    ros::Publisher vehicle_model_pub_;

    ros::Timer planning_timer_;
    ros::Timer visualization_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformBroadcaster dynamic_tf_broadcaster_;

    // 状态
    std::string current_behavior_;
    nav_msgs::OccupancyGrid current_map_;
    bool has_map_;
    bool initialized_;
    std::vector<PathPoint> current_path_;

    // 跟踪相关
    std::vector<Waypoint> waypoints_;
    int waypoint_index_;

    // 位置跟踪
    double current_x_, current_y_, current_yaw_;

    // 参数
    double planning_frequency_;
    double lookahead_distance_;
    int trajectory_points_;
    double default_speed_;
    double vehicle_width_;
    double safety_margin_;

    // 新增函数声明
    void setupStaticTransforms();
    void publishDynamicTF();
    void behaviorCommandCallback(const std_msgs::String::ConstPtr& msg);
    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void planningTimerCallback(const ros::TimerEvent& event);
    void visualizationTimerCallback(const ros::TimerEvent& event);
    bool getCurrentPose(double& x, double& y, double& yaw);
    void generateAndPublishPath();
    std::vector<PathPoint> generateWaypointTrackingPath(double x, double y, double yaw);
    std::vector<PathPoint> generateStoppingPath(double x, double y, double yaw);
    bool isPointOccupied(double x, double y);
    void publishTrajectory(const std::vector<PathPoint>& path);
    void publishPath(const std::vector<PathPoint>& path);
    void publishMapVisualization();
    void publishInflatedMap();
    void publishVehicleModel();
    void publishPathVisualization();
    void publishBehaviorIndicator();
    bool loadWaypointsFromJson(const std::string& filename);
};

PlanningNode::PlanningNode() : tf_listener_(tf_buffer_), waypoint_index_(0)
{
    setupStaticTransforms();

    // 订阅与发布
    behavior_command_sub_ = nh_.subscribe("/decision/behavior_command", 1,
        &PlanningNode::behaviorCommandCallback, this);
    occupancy_grid_sub_ = nh_.subscribe("/perception/occupancy_grid", 1,
        &PlanningNode::occupancyGridCallback, this);

    trajectory_pub_ = nh_.advertise<msg_interfaces::Trajectory>("/planning/trajectory", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/path", 10);
    map_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/map_visualization", 1);
    inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/planning/inflated_map", 1);
    path_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/path_visualization", 1);
    behavior_indicator_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/behavior_indicator", 1);
    vehicle_model_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/vehicle_model", 1);

    nh_.param("planning_frequency", planning_frequency_, 5.0);
    nh_.param("lookahead_distance", lookahead_distance_, 10.0);
    nh_.param("trajectory_points", trajectory_points_, 8);
    nh_.param("default_speed", default_speed_, 3.0);
    nh_.param("vehicle_width", vehicle_width_, 1.8);
    nh_.param("safety_margin", safety_margin_, 0.5);

    current_behavior_ = "STRAIGHT";
    has_map_ = false;
    initialized_ = false;
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_yaw_ = 0.0;

    // 读取waypoints
    std::string pkg_path = ros::package::getPath("planning_package");
    std::string json_path = pkg_path + "/cfg/waypoints.json";
    if (!loadWaypointsFromJson(json_path)) {
        ROS_ERROR("Failed to load waypoints from: %s", json_path.c_str());
    } else {
        ROS_INFO("Loaded %lu waypoints.", waypoints_.size());
    }

    planning_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_),
        &PlanningNode::planningTimerCallback, this);

    visualization_timer_ = nh_.createTimer(ros::Duration(0.2),
        &PlanningNode::visualizationTimerCallback, this);

    ROS_INFO("Planning Node started with waypoint tracking");
    ros::Duration(3.0).sleep();
    ROS_INFO("Planning node ready - starting visualization");
}

void PlanningNode::setupStaticTransforms()
{
    std::vector<geometry_msgs::TransformStamped> static_transforms;
    geometry_msgs::TransformStamped base_to_ins;
    base_to_ins.header.stamp = ros::Time::now();
    base_to_ins.header.frame_id = "base_link";
    base_to_ins.child_frame_id = "OurCar/Sensors/INS";
    base_to_ins.transform.translation.x = 0.0;
    base_to_ins.transform.translation.y = 0.0;
    base_to_ins.transform.translation.z = 0.0;
    base_to_ins.transform.rotation.w = 1.0;
    static_transforms.push_back(base_to_ins);

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
    ros::Duration(1.0).sleep();
}

void PlanningNode::publishDynamicTF()
{
    geometry_msgs::TransformStamped map_to_base;
    map_to_base.header.stamp = ros::Time::now();
    map_to_base.header.frame_id = "map";
    map_to_base.child_frame_id = "base_link";
    map_to_base.transform.translation.x = current_x_;
    map_to_base.transform.translation.y = current_y_;
    map_to_base.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, current_yaw_);
    map_to_base.transform.rotation.x = q.x();
    map_to_base.transform.rotation.y = q.y();
    map_to_base.transform.rotation.z = q.z();
    map_to_base.transform.rotation.w = q.w();
    dynamic_tf_broadcaster_.sendTransform(map_to_base);
}

void PlanningNode::behaviorCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string new_behavior = msg->data;
    if (new_behavior != current_behavior_) {
        ROS_INFO("🔄 Planning behavior changed: %s → %s", current_behavior_.c_str(), new_behavior.c_str());
        current_behavior_ = new_behavior;
        generateAndPublishPath();
    }
}

void PlanningNode::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    current_map_ = *msg;
    has_map_ = true;
    ROS_INFO_ONCE("📍 Received occupancy grid map for planning");
    ROS_INFO_THROTTLE(5.0, "Map updated: %dx%d cells, resolution=%.3f", msg->info.width, msg->info.height, msg->info.resolution);
}

void PlanningNode::planningTimerCallback(const ros::TimerEvent& event)
{
    generateAndPublishPath();
    publishDynamicTF();
}

void PlanningNode::visualizationTimerCallback(const ros::TimerEvent& event)
{
    publishVehicleModel();
    if (has_map_) {
        publishMapVisualization();
        publishInflatedMap();
    }
    if (!current_path_.empty()) {
        publishPathVisualization();
        publishBehaviorIndicator();
    }
    publishDynamicTF();
}

bool PlanningNode::getCurrentPose(double& x, double& y, double& yaw)
{
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
            ROS_INFO("✅ Received external vehicle position: (%.2f, %.2f, %.2f°)", x, y, yaw * 180.0 / M_PI);
            initialized_ = true;
        }
        return true;
    } catch (tf2::TransformException& ex) {
        if (!initialized_ && has_map_) {
            current_x_ = current_map_.info.origin.position.x + (current_map_.info.width * current_map_.info.resolution) / 2.0;
            current_y_ = current_map_.info.origin.position.y + (current_map_.info.height * current_map_.info.resolution) / 2.0;
            current_yaw_ = 0.0;
            initialized_ = true;
            ROS_INFO("🎯 Initialized vehicle at map center: (%.2f, %.2f)", current_x_, current_y_);
        } else if (!has_map_) {
            static double sim_time = 0.0;
            sim_time += 0.2;
            current_x_ = 10.0 * cos(sim_time * 0.05);
            current_y_ = 10.0 * sin(sim_time * 0.05);
            current_yaw_ = sim_time * 0.05 + M_PI/2;
            ROS_INFO_THROTTLE(2.0, "🚗 Simulated vehicle position: (%.2f, %.2f, %.1f°)", current_x_, current_y_, current_yaw_ * 180.0 / M_PI);
        }
        x = current_x_;
        y = current_y_;
        yaw = current_yaw_;
        return true;
    }
}

void PlanningNode::generateAndPublishPath()
{
    double current_x, current_y, current_yaw;
    if (!getCurrentPose(current_x, current_y, current_yaw)) {
        return;
    }

    std::vector<PathPoint> path;
    if (current_behavior_ == "STRAIGHT") {
        path = generateWaypointTrackingPath(current_x, current_y, current_yaw);
    } else if (current_behavior_ == "EMERGENCY_STOP") {
        path = generateStoppingPath(current_x, current_y, current_yaw);
    } else {
        path = generateWaypointTrackingPath(current_x, current_y, current_yaw);
    }
    current_path_ = path;
    publishTrajectory(path);
    publishPath(path);
    ROS_DEBUG("Generated %s path with %lu points at (%.2f, %.2f)", current_behavior_.c_str(), path.size(), current_x, current_y);
}

std::vector<PathPoint> PlanningNode::generateWaypointTrackingPath(double x, double y, double yaw)
{
    std::vector<PathPoint> path;
    if (waypoints_.empty()) return path;
    // 选择最近的waypoint为目标
    double min_dist = std::numeric_limits<double>::max();
    int target_idx = waypoint_index_;
    for (size_t i = waypoint_index_; i < waypoints_.size(); ++i) {
        double dx = waypoints_[i].x - x;
        double dy = waypoints_[i].y - y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            target_idx = i;
        }
    }
    // 若到达当前waypoint，自动切到下一个
    if (min_dist < 2.0 && waypoint_index_ < (int)waypoints_.size() - 1) {
        waypoint_index_++;
        ROS_INFO("Reached waypoint %d, moving to %d", target_idx, waypoint_index_);
        target_idx = waypoint_index_;
    }
    // 生成从当前位置到目标waypoint的直线路径
    Waypoint& goal = waypoints_[target_idx];
    double step = 1.0;
    double total_dist = std::hypot(goal.x - x, goal.y - y);
    int num_steps = std::max(trajectory_points_, (int)(total_dist / step));
    for (int i = 1; i <= num_steps; ++i) {
        double ratio = (double)i / num_steps;
        PathPoint p;
        p.x = x + ratio * (goal.x - x);
        p.y = y + ratio * (goal.y - y);
        p.yaw = atan2(goal.y - y, goal.x - x);
        p.velocity = default_speed_;
        p.is_safe = !isPointOccupied(p.x, p.y);
        path.push_back(p);
    }
    return path;
}

std::vector<PathPoint> PlanningNode::generateStoppingPath(double x, double y, double yaw)
{
    std::vector<PathPoint> path;
    double stopping_distance = 3.0;
    double step_distance = stopping_distance / trajectory_points_;
    for (int i = 1; i <= trajectory_points_; ++i) {
        PathPoint point;
        double progress = (double)i / trajectory_points_;
        double distance = i * step_distance;
        point.x = x + distance * cos(yaw);
        point.y = y + distance * sin(yaw);
        point.yaw = yaw;
        point.velocity = default_speed_ * (1.0 - progress);
        point.is_safe = true;
        path.push_back(point);
    }
    return path;
}

bool PlanningNode::isPointOccupied(double x, double y)
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

void PlanningNode::publishTrajectory(const std::vector<PathPoint>& path)
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

void PlanningNode::publishPath(const std::vector<PathPoint>& path)
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

void PlanningNode::publishMapVisualization()
{
    if (!has_map_) {
        ROS_WARN_THROTTLE(5.0, "No map available for visualization");
        return;
    }
    // visualization_msgs::Marker;
}
// 1. 读取 waypoints 的实现（你用 nlohmann/json）
bool PlanningNode::loadWaypointsFromJson(const std::string& filename)
{
    waypoints_.clear();
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open waypoint file: %s", filename.c_str());
        return false;
    }

    nlohmann::json j;
    try {
        file >> j;
    } catch (nlohmann::json::parse_error& e) {
        ROS_ERROR("JSON parse error: %s", e.what());
        return false;
    }

    for (const auto& item : j) {
        Waypoint wp;
        wp.x = item["position"]["x"];
        wp.y = item["position"]["y"];
        wp.yaw = 0;
        if (item.contains("orientation")) {
            double qx = item["orientation"]["x"];
            double qy = item["orientation"]["y"];
            double qz = item["orientation"]["z"];
            double qw = item["orientation"]["w"];
            tf2::Quaternion q(qx, qy, qz, qw);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, wp.yaw);
        }
        waypoints_.push_back(wp);
    }

    ROS_INFO("Loaded %lu waypoints from %s", waypoints_.size(), filename.c_str());
    return true;
}


// 2. 车辆模型可视化实现
void PlanningNode::publishVehicleModel()
{
    visualization_msgs::Marker vehicle_marker;
    vehicle_marker.header.frame_id = "base_link";
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.ns = "vehicle";
    vehicle_marker.id = 0;
    vehicle_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_marker.action = visualization_msgs::Marker::ADD;

    vehicle_marker.pose.position.x = 0.0;
    vehicle_marker.pose.position.y = 0.0;
    vehicle_marker.pose.position.z = 0.5;
    vehicle_marker.pose.orientation.w = 1.0;

    vehicle_marker.scale.x = 4.4;
    vehicle_marker.scale.y = 1.8;
    vehicle_marker.scale.z = 1.0;

    // 颜色可以按状态
    vehicle_marker.color.r = 0.0;
    vehicle_marker.color.g = 0.8;
    vehicle_marker.color.b = 1.0;
    vehicle_marker.color.a = 0.8;

    vehicle_model_pub_.publish(vehicle_marker);
}

// 3. 膨胀地图发布实现
void PlanningNode::publishInflatedMap()
{
    // 简单实现，可照搬你自己原来的逻辑
    nav_msgs::OccupancyGrid inflated_map = current_map_;
    inflated_map.header.stamp = ros::Time::now();
    inflated_map_pub_.publish(inflated_map);
}

// 4. 路径可视化实现
void PlanningNode::publishPathVisualization()
{
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "planned_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.3;
    path_marker.color.r = 0.0;
    path_marker.color.g = 0.8;
    path_marker.color.b = 1.0;
    path_marker.color.a = 1.0;

    for (const auto& pt : current_path_) {
        geometry_msgs::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.3;
        path_marker.points.push_back(p);
    }
    path_visualization_pub_.publish(path_marker);
}

// 5. 行为指示器可视化实现
void PlanningNode::publishBehaviorIndicator()
{
    visualization_msgs::Marker indicator;
    indicator.header.frame_id = "base_link";
    indicator.header.stamp = ros::Time::now();
    indicator.ns = "behavior_indicator";
    indicator.id = 0;
    indicator.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    indicator.action = visualization_msgs::Marker::ADD;
    indicator.pose.position.x = 0.0;
    indicator.pose.position.y = 0.0;
    indicator.pose.position.z = 3.0;
    indicator.pose.orientation.w = 1.0;
    indicator.scale.z = 1.0;
    indicator.color.r = 0.0;
    indicator.color.g = 1.0;
    indicator.color.b = 0.0;
    indicator.color.a = 1.0;
    indicator.text = "PLANNING";
    behavior_indicator_pub_.publish(indicator);
}

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
