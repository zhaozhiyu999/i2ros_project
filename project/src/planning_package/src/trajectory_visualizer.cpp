#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <msg_interfaces/Trajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class SimpleTrajectoryVisualizer {
public:
    SimpleTrajectoryVisualizer(ros::NodeHandle& nh) : tf_listener_(tf_buffer_) {
        // ========== 订阅轨迹数据 ==========
        trajectory_sub_ = nh.subscribe("/planning/trajectory", 1, 
                                     &SimpleTrajectoryVisualizer::trajectoryCallback, this);
        
        // ========== 发布简化可视化 ==========
        smooth_path_pub_ = nh.advertise<nav_msgs::Path>("/visualization/smooth_path", 1);
        trajectory_markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/visualization/trajectory_viz", 1);
        
        // ========== 参数设置 ==========
        nh.param("interpolation_resolution", interpolation_resolution_, 0.2);  // 插值分辨率
        nh.param("path_line_width", path_line_width_, 0.15);
        nh.param("show_vehicle", show_vehicle_, true);
        nh.param("show_direction_arrows", show_arrows_, true);
        
        has_trajectory_ = false;
        
        ROS_INFO("Simple Trajectory Visualizer started");
        ROS_INFO("Publishing smooth trajectory to: /visualization/smooth_path");
        ROS_INFO("Publishing markers to: /visualization/trajectory_viz");
    }

private:
    // ROS相关
    ros::Subscriber trajectory_sub_;
    ros::Publisher smooth_path_pub_;
    ros::Publisher trajectory_markers_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 数据
    msg_interfaces::Trajectory current_trajectory_;
    bool has_trajectory_;
    
    // 参数
    double interpolation_resolution_;
    double path_line_width_;
    bool show_vehicle_;
    bool show_arrows_;

    // ========== 轨迹回调 ==========
    void trajectoryCallback(const msg_interfaces::Trajectory::ConstPtr& msg) {
        current_trajectory_ = *msg;
        has_trajectory_ = true;
        
        // 立即发布可视化
        publishSmoothPath();
        publishTrajectoryMarkers();
        
        ROS_INFO_THROTTLE(1.0, "Visualizing trajectory with %lu waypoints", 
                         current_trajectory_.poses.size());
    }
    
    // ========== 发布平滑路径 ==========
    void publishSmoothPath() {
        if (!has_trajectory_ || current_trajectory_.poses.size() < 2) {
            return;
        }
        
        nav_msgs::Path smooth_path;
        smooth_path.header.stamp = ros::Time::now();
        smooth_path.header.frame_id = "map";
        
        // 使用样条插值生成平滑路径
        std::vector<geometry_msgs::PoseStamped> interpolated_poses = 
            interpolateTrajectory(current_trajectory_.poses);
        
        smooth_path.poses = interpolated_poses;
        smooth_path_pub_.publish(smooth_path);
    }
    
    // ========== 轨迹插值 ==========
    std::vector<geometry_msgs::PoseStamped> interpolateTrajectory(
        const std::vector<geometry_msgs::Pose>& waypoints) {
        
        std::vector<geometry_msgs::PoseStamped> interpolated;
        
        if (waypoints.size() < 2) return interpolated;
        
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            geometry_msgs::Pose start = waypoints[i];
            geometry_msgs::Pose end = waypoints[i + 1];
            
            // 计算两点间距离
            double dx = end.position.x - start.position.x;
            double dy = end.position.y - start.position.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            // 计算插值点数量
            int num_points = std::max(2, (int)(distance / interpolation_resolution_));
            
            // 生成插值点
            for (int j = 0; j <= num_points; ++j) {
                double t = (double)j / num_points;
                
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                
                // 位置插值（使用平滑的Hermite插值）
                pose_stamped.pose.position.x = cubicInterpolate(start.position.x, end.position.x, t);
                pose_stamped.pose.position.y = cubicInterpolate(start.position.y, end.position.y, t);
                pose_stamped.pose.position.z = start.position.z + t * (end.position.z - start.position.z);
                
                // 朝向插值
                pose_stamped.pose.orientation = interpolateOrientation(start.orientation, end.orientation, t);
                
                interpolated.push_back(pose_stamped);
            }
        }
        
        return interpolated;
    }
    
    // ========== 三次插值 ==========
    double cubicInterpolate(double start, double end, double t) {
        // 使用平滑的S形曲线
        double smooth_t = t * t * (3.0 - 2.0 * t);  // smoothstep函数
        return start + smooth_t * (end - start);
    }
    
    // ========== 姿态插值 ==========
    geometry_msgs::Quaternion interpolateOrientation(
        const geometry_msgs::Quaternion& start, 
        const geometry_msgs::Quaternion& end, 
        double t) {
        
        // 简化：使用线性插值然后归一化
        geometry_msgs::Quaternion result;
        result.x = start.x + t * (end.x - start.x);
        result.y = start.y + t * (end.y - start.y);
        result.z = start.z + t * (end.z - start.z);
        result.w = start.w + t * (end.w - start.w);
        
        // 归一化
        double norm = sqrt(result.x*result.x + result.y*result.y + 
                          result.z*result.z + result.w*result.w);
        if (norm > 0) {
            result.x /= norm;
            result.y /= norm;
            result.z /= norm;
            result.w /= norm;
        } else {
            result.w = 1.0;
        }
        
        return result;
    }
    
    // ========== 发布简化标记 ==========
    void publishTrajectoryMarkers() {
        if (!has_trajectory_) return;
        
        visualization_msgs::MarkerArray marker_array;
        
        // 清除之前的标记
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 1. 轨迹线标记
        publishTrajectoryLine(marker_array);
        
        // 2. 起点和终点标记
        publishWaypointMarkers(marker_array);
        
        // 3. 方向箭头（可选）
        if (show_arrows_) {
            publishDirectionArrows(marker_array);
        }
        
        // 4. 车辆位置（可选）
        if (show_vehicle_) {
            publishVehicleMarker(marker_array);
        }
        
        trajectory_markers_pub_.publish(marker_array);
    }
    
    // ========== 轨迹线 ==========
    void publishTrajectoryLine(visualization_msgs::MarkerArray& marker_array) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "trajectory_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.scale.x = path_line_width_;
        
        // 动态颜色：根据速度变化
        line_marker.color.r = 0.2;
        line_marker.color.g = 0.8;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.9;
        
        for (const auto& pose : current_trajectory_.poses) {
            geometry_msgs::Point point;
            point.x = pose.position.x;
            point.y = pose.position.y;
            point.z = pose.position.z + 0.05;  // 稍微抬高
            line_marker.points.push_back(point);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    // ========== 路径点标记 ==========
    void publishWaypointMarkers(visualization_msgs::MarkerArray& marker_array) {
        if (current_trajectory_.poses.empty()) return;
        
        // 起点标记
        visualization_msgs::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = ros::Time::now();
        start_marker.ns = "waypoints";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::Marker::SPHERE;
        start_marker.action = visualization_msgs::Marker::ADD;
        
        start_marker.pose = current_trajectory_.poses[0];
        start_marker.pose.position.z += 0.1;
        
        start_marker.scale.x = 0.4;
        start_marker.scale.y = 0.4;
        start_marker.scale.z = 0.4;
        
        start_marker.color.r = 0.0;
        start_marker.color.g = 1.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        
        marker_array.markers.push_back(start_marker);
        
        // 终点标记
        visualization_msgs::Marker end_marker = start_marker;
        end_marker.id = 2;
        end_marker.pose = current_trajectory_.poses.back();
        end_marker.pose.position.z += 0.1;
        
        end_marker.color.r = 1.0;
        end_marker.color.g = 0.0;
        end_marker.color.b = 0.0;
        
        marker_array.markers.push_back(end_marker);
    }
    
    // ========== 方向箭头 ==========
    void publishDirectionArrows(visualization_msgs::MarkerArray& marker_array) {
        // 只在几个关键点显示方向箭头
        int arrow_interval = std::max(1, (int)current_trajectory_.poses.size() / 4);
        
        for (size_t i = 0; i < current_trajectory_.poses.size(); i += arrow_interval) {
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "direction_arrows";
            arrow.id = i / arrow_interval + 10;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            arrow.pose = current_trajectory_.poses[i];
            arrow.pose.position.z += 0.2;
            
            arrow.scale.x = 0.8;  // 长度
            arrow.scale.y = 0.1;  // 宽度
            arrow.scale.z = 0.1;  // 高度
            
            arrow.color.r = 1.0;
            arrow.color.g = 0.5;
            arrow.color.b = 0.0;
            arrow.color.a = 0.8;
            
            marker_array.markers.push_back(arrow);
        }
    }
    
    // ========== 车辆位置 ==========
    void publishVehicleMarker(visualization_msgs::MarkerArray& marker_array) {
        // 获取当前车辆位置
        geometry_msgs::TransformStamped current_pose;
        std::vector<std::string> possible_frames = {"base_link", "OurCar/Sensors/INS"};
        
        bool pose_found = false;
        for (const auto& frame : possible_frames) {
            try {
                current_pose = tf_buffer_.lookupTransform("map", frame, ros::Time(0), ros::Duration(0.1));
                pose_found = true;
                break;
            } catch (tf2::TransformException& ex) {
                continue;
            }
        }
        
        if (!pose_found) return;
        
        visualization_msgs::Marker vehicle_marker;
        vehicle_marker.header.frame_id = "map";
        vehicle_marker.header.stamp = ros::Time::now();
        vehicle_marker.ns = "vehicle";
        vehicle_marker.id = 100;
        vehicle_marker.type = visualization_msgs::Marker::CUBE;
        vehicle_marker.action = visualization_msgs::Marker::ADD;
        
        // 车辆位置和姿态
        vehicle_marker.pose.position.x = current_pose.transform.translation.x;
        vehicle_marker.pose.position.y = current_pose.transform.translation.y;
        vehicle_marker.pose.position.z = 0.75;  // 车辆高度中心
        vehicle_marker.pose.orientation = current_pose.transform.rotation;
        
        // 车辆尺寸（简化）
        vehicle_marker.scale.x = 4.0;  // 长度
        vehicle_marker.scale.y = 1.8;  // 宽度
        vehicle_marker.scale.z = 1.5;  // 高度
        
        // 车辆颜色 - 醒目的橙色
        vehicle_marker.color.r = 1.0;
        vehicle_marker.color.g = 0.5;
        vehicle_marker.color.b = 0.0;
        vehicle_marker.color.a = 0.9;
        
        marker_array.markers.push_back(vehicle_marker);
        
        // 添加车辆前方指示器
        visualization_msgs::Marker front_indicator;
        front_indicator.header.frame_id = "map";
        front_indicator.header.stamp = ros::Time::now();
        front_indicator.ns = "vehicle_front";
        front_indicator.id = 101;
        front_indicator.type = visualization_msgs::Marker::CYLINDER;
        front_indicator.action = visualization_msgs::Marker::ADD;
        
        // 前方指示器位置（车辆前端）
        tf2::Quaternion q(current_pose.transform.rotation.x,
                         current_pose.transform.rotation.y,
                         current_pose.transform.rotation.z,
                         current_pose.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        front_indicator.pose.position.x = current_pose.transform.translation.x + 2.2 * cos(yaw);
        front_indicator.pose.position.y = current_pose.transform.translation.y + 2.2 * sin(yaw);
        front_indicator.pose.position.z = 1.0;
        front_indicator.pose.orientation = current_pose.transform.rotation;
        
        front_indicator.scale.x = 0.3;
        front_indicator.scale.y = 0.3;
        front_indicator.scale.z = 0.5;
        
        front_indicator.color.r = 0.0;
        front_indicator.color.g = 1.0;
        front_indicator.color.b = 0.0;
        front_indicator.color.a = 1.0;
        
        marker_array.markers.push_back(front_indicator);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_trajectory_visualizer");
    ros::NodeHandle nh;
    
    SimpleTrajectoryVisualizer visualizer(nh);
    
    ROS_INFO("========================================");
    ROS_INFO("Simple Trajectory Visualizer started!");
    ROS_INFO("========================================");
    ROS_INFO("RViz displays to add:");
    ROS_INFO("  1. Path: /visualization/smooth_path");
    ROS_INFO("     - Color: Blue/Cyan");
    ROS_INFO("     - Line width: 0.15m");
    ROS_INFO("  2. MarkerArray: /visualization/trajectory_viz");
    ROS_INFO("     - Shows waypoints, vehicle, arrows");
    ROS_INFO("========================================");
    
    ros::spin();
    
    return 0;
}