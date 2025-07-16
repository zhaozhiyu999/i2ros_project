#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class MapBuilder {
public:
    MapBuilder(ros::NodeHandle& nh) : tf_listener(tf_buffer) {
        depth_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1, &MapBuilder::depthCallback, this);
        cam_info_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1, &MapBuilder::cameraInfoCallback, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/perception/occupancy_grid", 1, true);

        resolution = 0.2;
        width = 500;
        height = 500;
        map_initialized = false;

        octree = std::make_shared<octomap::OcTree>(resolution);

        map.header.frame_id = "map";
        map.info.resolution = resolution;
        map.info.width = width;
        map.info.height = height;
        map.info.origin.position.z = 0;
        map.info.origin.orientation.w = 1.0;
        map.data.assign(width * height, -1);
    }

private:
    ros::Subscriber depth_sub;
    ros::Subscriber cam_info_sub;
    ros::Publisher map_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    float fx, fy, cx, cy;
    bool cam_info_received = false;
    bool map_initialized = false;

    nav_msgs::OccupancyGrid map;
    float resolution;
    int width, height;
    float origin_x, origin_y;

    std::shared_ptr<octomap::OcTree> octree;

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

        double car_x = tf_cam_to_map.transform.translation.x;
        double car_y = tf_cam_to_map.transform.translation.y;
        expandMapToInclude(car_x, car_y);

        octomap::Pointcloud cloud;
        octomap::point3d sensor_origin(tf_cam_to_map.transform.translation.x,
                                       tf_cam_to_map.transform.translation.y,
                                       tf_cam_to_map.transform.translation.z);

        for (int v = 0; v < cv_ptr->image.rows; ++v) {
            for (int u = 0; u < cv_ptr->image.cols; ++u) {
                uint16_t d = cv_ptr->image.at<uint16_t>(v, u);
                if (d == 0) continue;
                float z = d * 0.001f;
                if (z < 0.1 || z > 20.0) continue;

                float x = (u - cx) * z / fx;
                float y = (v - cy) * z / fy;

                geometry_msgs::PointStamped pt_cam, pt_map;
                pt_cam.header = msg->header;
                pt_cam.point.x = x;
                pt_cam.point.y = y;
                pt_cam.point.z = z;

                tf2::doTransform(pt_cam, pt_map, tf_cam_to_map);
                if (pt_map.point.z > 2.0) continue;

                cloud.push_back(pt_map.point.x, pt_map.point.y, pt_map.point.z);
            }
        }

        octree->insertPointCloud(cloud, sensor_origin);

        map.data.assign(width * height, -1);
        int filled = 0;

        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
            if (octree->isNodeOccupied(*it)) {
                float x = it.getX();
                float y = it.getY();
                float z = it.getZ();
                if (z > 2.0) continue;

                int mx = (x - origin_x) / resolution;
                int my = (y - origin_y) / resolution;
                if (mx >= 0 && mx < width && my >= 0 && my < height) {
                    int idx = my * width + mx;
                    if (map.data[idx] != 100) {
                        map.data[idx] = 100;
                        ++filled;
                    }
                }
            }
        }

        map.header.stamp = ros::Time::now();
        map_pub.publish(map);
        ROS_INFO_THROTTLE(2.0, "OccupancyGrid published. Cells updated: %d", filled);
    }

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

        ROS_WARN("Map auto-expanded to %dx%d (%.1fx%.1f meters)", width, height, width * resolution, height * resolution);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_builder_node");
    ros::NodeHandle nh;
    MapBuilder builder(nh);
    ros::spin();
    return 0;
}

