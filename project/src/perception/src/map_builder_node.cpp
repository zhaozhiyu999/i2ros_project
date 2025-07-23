#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

// OctoMap相关头文件
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class EnhancedMapBuilder {
public:
    EnhancedMapBuilder(ros::NodeHandle& nh) : tf_listener(tf_buffer) {
        // 原有的地图构建订阅和发布
        depth_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1, 
                                &EnhancedMapBuilder::depthCallback, this);
        cam_info_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1, 
                                   &EnhancedMapBuilder::cameraInfoCallback, this);
        
        // 发布传统的2D占用栅格地图（与现有系统兼容）
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/perception/occupancy_grid", 1, true);
        
        // 新增：OctoMap相关发布
        octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_full", 1, true);
        octomap_binary_pub = nh.advertise<octomap_msgs::Octomap>("/octomap_binary", 1, true);

        // 新增：车辆位置和可视化发布
        vehicle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vehicle/pose", 1);
        vehicle_pose_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vehicle/pose_with_covariance", 1);
        vehicle_odom_pub = nh.advertise<nav_msgs::Odometry>("/vehicle/odometry", 1);
        vehicle_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/vehicle/visualization", 1);

        // 定时器
        vehicle_timer = nh.createTimer(ros::Duration(0.1), &EnhancedMapBuilder::vehicleTimerCallback, this);
        octomap_timer = nh.createTimer(ros::Duration(1.0), &EnhancedMapBuilder::publishOctomapCallback, this);

        // 参数设置
        resolution = 0.2;
        width = 500;
        height = 500;
        map_initialized = false;

        // 初始化OctoMap
        octree = std::make_shared<octomap::OcTree>(resolution);
        octree->setProbHit(0.9);   // 命中概率
        octree->setProbMiss(0.1);  // 未命中概率
        octree->setClampingThresMin(0.12);  // 最小概率阈值
        octree->setClampingThresMax(0.97);  // 最大概率阈值

        // 初始化传统2D地图
        map.header.frame_id = "map";
        map.info.resolution = resolution;
        map.info.width = width;
        map.info.height = height;
        map.info.origin.position.z = 0;
        map.info.origin.orientation.w = 1.0;
        map.data.assign(width * height, -1);

        ROS_INFO("Enhanced Map Builder with OctoMap initialized");
    }

private:
    // 原有成员变量
    ros::Subscriber depth_sub;
    ros::Subscriber cam_info_sub;
    ros::Publisher map_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // 新增：OctoMap发布器
    ros::Publisher octomap_pub;
    ros::Publisher octomap_binary_pub;

    // 车辆相关发布器和定时器
    ros::Publisher vehicle_pose_pub;
    ros::Publisher vehicle_pose_cov_pub;
    ros::Publisher vehicle_odom_pub;
    ros::Publisher vehicle_marker_pub;
    ros::Timer vehicle_timer;
    ros::Timer octomap_timer;

    float fx, fy, cx, cy;
    bool cam_info_received = false;
    bool map_initialized = false;

    nav_msgs::OccupancyGrid map;
    float resolution;
    int width, height;
    float origin_x, origin_y;

    // OctoMap相关
    std::shared_ptr<octomap::OcTree> octree;

    // 车辆状态跟踪
    geometry_msgs::TransformStamped last_vehicle_transform;
    ros::Time last_vehicle_time;
    bool first_vehicle_update = true;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info) {
        fx = info->K[0];
        fy = info->K[4];
        cx = info->K[2];
        cy = info->K[5];
        cam_info_received = true;
        ROS_INFO_ONCE("Camera info received.");
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!cam_info_received) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        geometry_msgs::TransformStamped tf_cam_to_map;
        try {
            tf_cam_to_map = tf_buffer.lookupTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
            return;
        }

        // 初始化地图原点（仅针对2D地图）
        if (!map_initialized) {
            float car_x = tf_cam_to_map.transform.translation.x;
            float car_y = tf_cam_to_map.transform.translation.y;
            origin_x = car_x - (width * resolution) / 2.0;
            origin_y = car_y - (height * resolution) / 2.0;
            map.info.origin.position.x = origin_x;
            map.info.origin.position.y = origin_y;
            map_initialized = true;
            ROS_INFO("Initialized map origin at (%.2f, %.2f)", origin_x, origin_y);
        }

        // 处理深度数据并更新OctoMap和2D地图
        processDepthData(cv_ptr, tf_cam_to_map);

        // 发布更新的2D地图
        map.header.stamp = ros::Time::now();
        map_pub.publish(map);
    }

    void processDepthData(const cv_bridge::CvImagePtr& cv_ptr, 
                         const geometry_msgs::TransformStamped& tf_cam_to_map) {
        
        // 创建OctoMap点云和传感器原点
        octomap::Pointcloud octo_cloud;
        octomap::point3d sensor_origin(
            tf_cam_to_map.transform.translation.x,
            tf_cam_to_map.transform.translation.y,
            tf_cam_to_map.transform.translation.z
        );

        // 处理深度图像
        int filled_2d = 0;
        std::vector<std::pair<int, int>> occupied_cells; // 存储2D地图中的占用格子

        for (int v = 0; v < cv_ptr->image.rows; ++v) {
            for (int u = 0; u < cv_ptr->image.cols; ++u) {
                uint16_t d = cv_ptr->image.at<uint16_t>(v, u);
                if (d == 0) continue;
                
                float z = d * 0.001f;
                if (z < 0.1 || z > 20.0) continue;

                float x = (u - cx) * z / fx;
                float y = (v - cy) * z / fy;

                // 转换到世界坐标系
                geometry_msgs::PointStamped pt_cam, pt_map;
                pt_cam.header = tf_cam_to_map.header;
                pt_cam.point.x = x;
                pt_cam.point.y = y;
                pt_cam.point.z = z;

                tf2::doTransform(pt_cam, pt_map, tf_cam_to_map);

                // 添加到OctoMap点云
                octo_cloud.push_back(pt_map.point.x, pt_map.point.y, pt_map.point.z);

                // 更新2D地图（只考虑一定高度以下的点）
                if (pt_map.point.z <= 2.0) {
                    // 确保地图能容纳新点
                    expandMapToInclude(pt_map.point.x, pt_map.point.y);

                    int mx = (pt_map.point.x - origin_x) / resolution;
                    int my = (pt_map.point.y - origin_y) / resolution;

                    if (mx >= 0 && mx < width && my >= 0 && my < height) {
                        int idx = my * width + mx;
                        if (map.data[idx] != 100) {
                            map.data[idx] = 100;
                            ++filled_2d;
                        }
                    }
                }
            }
        }

        // 更新OctoMap
        if (octo_cloud.size() > 0) {
            octree->insertPointCloud(octo_cloud, sensor_origin);
            
            // 可选：定期裁剪OctoMap以节省内存
            static int update_count = 0;
            update_count++;
            if (update_count % 10 == 0) {
                octree->prune();
            }
        }

        ROS_INFO_THROTTLE(2.0, "Map updated. 2D cells: %d, OctoMap points: %zu", 
                         filled_2d, octo_cloud.size());
    }

    void publishOctomapCallback(const ros::TimerEvent&) {
        if (!octree) return;

        // 发布完整的OctoMap
        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.frame_id = "map";
        octomap_msg.header.stamp = ros::Time::now();
        
        if (octomap_msgs::fullMapToMsg(*octree, octomap_msg)) {
            octomap_pub.publish(octomap_msg);
        }

        // 发布二进制OctoMap（更紧凑）
        octomap_msgs::Octomap octomap_binary_msg;
        octomap_binary_msg.header.frame_id = "map";
        octomap_binary_msg.header.stamp = ros::Time::now();
        
        if (octomap_msgs::binaryMapToMsg(*octree, octomap_binary_msg)) {
            octomap_binary_pub.publish(octomap_binary_msg);
        }

        ROS_INFO_THROTTLE(5.0, "Published OctoMap with %zu nodes", octree->size());
    }

    void vehicleTimerCallback(const ros::TimerEvent&) {
        geometry_msgs::TransformStamped vehicle_transform;
        
        try {
            vehicle_transform = tf_buffer.lookupTransform("map", "OurCar/INS", ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(2.0, "Failed to get vehicle transform: %s", ex.what());
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        
        // 发布车辆位置信息
        publishVehiclePose(vehicle_transform, current_time);
        publishVehiclePoseWithCovariance(vehicle_transform, current_time);
        publishVehicleOdometry(vehicle_transform, current_time);
        publishVehicleMarkers(vehicle_transform, current_time);
        
        // 更新历史状态
        last_vehicle_transform = vehicle_transform;
        last_vehicle_time = current_time;
        first_vehicle_update = false;
    }

    void publishVehiclePose(const geometry_msgs::TransformStamped& transform, const ros::Time& stamp) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.position.x = transform.transform.translation.x;
        pose_msg.pose.position.y = transform.transform.translation.y;
        pose_msg.pose.position.z = transform.transform.translation.z;
        pose_msg.pose.orientation = transform.transform.rotation;
        
        vehicle_pose_pub.publish(pose_msg);
    }
    
    void publishVehiclePoseWithCovariance(const geometry_msgs::TransformStamped& transform, const ros::Time& stamp) {
        geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header.stamp = stamp;
        pose_cov_msg.header.frame_id = "map";
        
        pose_cov_msg.pose.pose.position.x = transform.transform.translation.x;
        pose_cov_msg.pose.pose.position.y = transform.transform.translation.y;
        pose_cov_msg.pose.pose.position.z = transform.transform.translation.z;
        pose_cov_msg.pose.pose.orientation = transform.transform.rotation;
        
        // 设置协方差矩阵
        for (int i = 0; i < 36; i++) {
            pose_cov_msg.pose.covariance[i] = 0.0;
        }
        pose_cov_msg.pose.covariance[0] = 0.01;  // x
        pose_cov_msg.pose.covariance[7] = 0.01;  // y
        pose_cov_msg.pose.covariance[14] = 0.01; // z
        pose_cov_msg.pose.covariance[21] = 0.01; // roll
        pose_cov_msg.pose.covariance[28] = 0.01; // pitch
        pose_cov_msg.pose.covariance[35] = 0.01; // yaw
        
        vehicle_pose_cov_pub.publish(pose_cov_msg);
    }
    
    void publishVehicleOdometry(const geometry_msgs::TransformStamped& transform, const ros::Time& stamp) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "OurCar/INS";
        
        // 位置和姿态
        odom_msg.pose.pose.position.x = transform.transform.translation.x;
        odom_msg.pose.pose.position.y = transform.transform.translation.y;
        odom_msg.pose.pose.position.z = transform.transform.translation.z;
        odom_msg.pose.pose.orientation = transform.transform.rotation;
        
        // 计算速度（简单的数值微分）
        if (!first_vehicle_update) {
            double dt = (stamp - last_vehicle_time).toSec();
            if (dt > 0) {
                odom_msg.twist.twist.linear.x = (transform.transform.translation.x - last_vehicle_transform.transform.translation.x) / dt;
                odom_msg.twist.twist.linear.y = (transform.transform.translation.y - last_vehicle_transform.transform.translation.y) / dt;
                odom_msg.twist.twist.linear.z = (transform.transform.translation.z - last_vehicle_transform.transform.translation.z) / dt;
                
                // 角速度计算（简化）
                tf2::Quaternion q_current, q_last;
                tf2::fromMsg(transform.transform.rotation, q_current);
                tf2::fromMsg(last_vehicle_transform.transform.rotation, q_last);
                
                tf2::Quaternion q_diff = q_current * q_last.inverse();
                tf2::Matrix3x3 m(q_diff);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                odom_msg.twist.twist.angular.x = roll / dt;
                odom_msg.twist.twist.angular.y = pitch / dt;
                odom_msg.twist.twist.angular.z = yaw / dt;
            }
        }
        
        // 设置协方差
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.01;  // yaw
        odom_msg.twist.covariance[0] = 0.1;   // vx
        odom_msg.twist.covariance[7] = 0.1;   // vy
        odom_msg.twist.covariance[35] = 0.1;  // vyaw
        
        vehicle_odom_pub.publish(odom_msg);
    }
    
    void publishVehicleMarkers(const geometry_msgs::TransformStamped& transform, const ros::Time& stamp) {
        visualization_msgs::MarkerArray marker_array;
        
        // 车身主体
        visualization_msgs::Marker vehicle_body;
        vehicle_body.header.frame_id = "map";
        vehicle_body.header.stamp = stamp;
        vehicle_body.ns = "vehicle";
        vehicle_body.id = 0;
        vehicle_body.type = visualization_msgs::Marker::CUBE;
        vehicle_body.action = visualization_msgs::Marker::ADD;
        
        vehicle_body.pose.position.x = transform.transform.translation.x;
        vehicle_body.pose.position.y = transform.transform.translation.y;
        vehicle_body.pose.position.z = transform.transform.translation.z + 0.635;
        vehicle_body.pose.orientation = transform.transform.rotation;
        
        vehicle_body.scale.x = 4.40; // 长度
        vehicle_body.scale.y = 2.00; // 宽度
        vehicle_body.scale.z = 1.27; // 高度
        
        vehicle_body.color.r = 0.2;
        vehicle_body.color.g = 0.5;
        vehicle_body.color.b = 0.8;
        vehicle_body.color.a = 0.8;
        
        marker_array.markers.push_back(vehicle_body);
        
        // 车辆方向箭头
        visualization_msgs::Marker direction_arrow;
        direction_arrow.header.frame_id = "map";
        direction_arrow.header.stamp = stamp;
        direction_arrow.ns = "vehicle";
        direction_arrow.id = 1;
        direction_arrow.type = visualization_msgs::Marker::ARROW;
        direction_arrow.action = visualization_msgs::Marker::ADD;
        
        direction_arrow.pose.position.x = transform.transform.translation.x;
        direction_arrow.pose.position.y = transform.transform.translation.y;
        direction_arrow.pose.position.z = transform.transform.translation.z + 1.5;
        direction_arrow.pose.orientation = transform.transform.rotation;
        
        direction_arrow.scale.x = 3.0; // 箭头长度
        direction_arrow.scale.y = 0.5; // 箭头宽度
        direction_arrow.scale.z = 0.5; // 箭头高度
        
        direction_arrow.color.r = 1.0;
        direction_arrow.color.g = 0.0;
        direction_arrow.color.b = 0.0;
        direction_arrow.color.a = 0.9;
        
        marker_array.markers.push_back(direction_arrow);
        
        // 车轮标记
        for (int i = 0; i < 4; i++) {
            visualization_msgs::Marker wheel;
            wheel.header.frame_id = "map";
            wheel.header.stamp = stamp;
            wheel.ns = "vehicle";
            wheel.id = 10 + i;
            wheel.type = visualization_msgs::Marker::CYLINDER;
            wheel.action = visualization_msgs::Marker::ADD;
            
            // 车轮相对位置
            double wheel_x_offset = (i < 2) ? 1.35 : -1.28;
            double wheel_y_offset = (i % 2 == 0) ? 1.0 : -1.0;
            
            // 转换到世界坐标系
            tf2::Quaternion vehicle_quat;
            tf2::fromMsg(transform.transform.rotation, vehicle_quat);
            tf2::Vector3 wheel_local(wheel_x_offset, wheel_y_offset, 0.0);
            tf2::Vector3 wheel_global = tf2::quatRotate(vehicle_quat, wheel_local);
            
            wheel.pose.position.x = transform.transform.translation.x + wheel_global.x();
            wheel.pose.position.y = transform.transform.translation.y + wheel_global.y();
            wheel.pose.position.z = transform.transform.translation.z + 0.35;
            
            tf2::Quaternion wheel_rotation;
            wheel_rotation.setRPY(1.5708, 0, 0);
            tf2::Quaternion final_wheel_quat = vehicle_quat * wheel_rotation;
            wheel.pose.orientation = tf2::toMsg(final_wheel_quat);
            
            wheel.scale.x = 0.7;
            wheel.scale.y = 0.7;
            wheel.scale.z = 0.25;
            
            wheel.color.r = 0.1;
            wheel.color.g = 0.1;
            wheel.color.b = 0.1;
            wheel.color.a = 1.0;
            
            marker_array.markers.push_back(wheel);
        }
        
        // 添加车辆ID文本
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = stamp;
        text_marker.ns = "vehicle";
        text_marker.id = 20;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = transform.transform.translation.x;
        text_marker.pose.position.y = transform.transform.translation.y;
        text_marker.pose.position.z = transform.transform.translation.z + 2.5;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 0.8;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = "EGO VEHICLE";
        
        marker_array.markers.push_back(text_marker);
        
        vehicle_marker_pub.publish(marker_array);
    }

    // 扩展2D地图函数（保持原有逻辑）
    void expandMapToInclude(double x, double y) {
        double map_right = origin_x + width * resolution;
        double map_top = origin_y + height * resolution;

        bool expand = false;
        double new_origin_x = origin_x;
        double new_origin_y = origin_y;
        int new_width = width;
        int new_height = height;

        double buffer = 5.0;  // 米

        if (x < origin_x + buffer) {
            int expand_cells = std::ceil((origin_x + buffer - x) / resolution);
            new_origin_x -= expand_cells * resolution;
            new_width += expand_cells;
            expand = true;
        } else if (x > map_right - buffer) {
            int expand_cells = std::ceil((x - (map_right - buffer)) / resolution);
            new_width += expand_cells;
            expand = true;
        }

        if (y < origin_y + buffer) {
            int expand_cells = std::ceil((origin_y + buffer - y) / resolution);
            new_origin_y -= expand_cells * resolution;
            new_height += expand_cells;
            expand = true;
        } else if (y > map_top - buffer) {
            int expand_cells = std::ceil((y - (map_top - buffer)) / resolution);
            new_height += expand_cells;
            expand = true;
        }

        if (!expand) return;

        nav_msgs::OccupancyGrid new_map;
        new_map.header.frame_id = "map";
        new_map.info.resolution = resolution;
        new_map.info.width = new_width;
        new_map.info.height = new_height;
        new_map.info.origin.position.x = new_origin_x;
        new_map.info.origin.position.y = new_origin_y;
        new_map.info.origin.orientation.w = 1.0;
        new_map.data.assign(new_width * new_height, -1);

        int x_offset = (origin_x - new_origin_x) / resolution;
        int y_offset = (origin_y - new_origin_y) / resolution;

        for (int y0 = 0; y0 < height; ++y0) {
            for (int x0 = 0; x0 < width; ++x0) {
                int old_idx = y0 * width + x0;
                int new_idx = (y0 + y_offset) * new_width + (x0 + x_offset);
                if (map.data[old_idx] >= 0) {
                    new_map.data[new_idx] = map.data[old_idx];
                }
            }
        }

        width = new_width;
        height = new_height;
        origin_x = new_origin_x;
        origin_y = new_origin_y;
        map = new_map;

        ROS_WARN("Map auto-expanded to %dx%d (%.1fx%.1f meters)", width, height, 
                width * resolution, height * resolution);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_map_builder_node");
    ros::NodeHandle nh;
    EnhancedMapBuilder builder(nh);
    ros::spin();
    return 0;
}