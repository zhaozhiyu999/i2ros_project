#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class DepthToCloudNode {
public:
    DepthToCloudNode(ros::NodeHandle& nh) : tf_listener(tf_buffer) {
        depth_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", 1, &DepthToCloudNode::depthCallback, this);
        cam_info_sub = nh.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info", 1, &DepthToCloudNode::cameraInfoCallback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth/points", 1);
    }

private:
    ros::Subscriber depth_sub;
    ros::Subscriber cam_info_sub;
    ros::Publisher cloud_pub;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    float fx, fy, cx, cy;
    bool cam_info_received = false;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info) {
        fx = info->K[0];
        fy = info->K[4];
        cx = info->K[2];
        cy = info->K[5];
        cam_info_received = true;
        ROS_INFO("Camera info received: fx=%f, fy=%f, cx=%f, cy=%f", fx, fy, cx, cy);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!cam_info_received) return;

        ROS_INFO_STREAM_ONCE("Got depth image with encoding: " << msg->encoding);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = "map";
        cloud.is_dense = false;

        geometry_msgs::TransformStamped tf_cam_to_map;
        try {
            tf_cam_to_map = tf_buffer.lookupTransform("map", msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
            return;
        }

        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            msg->encoding == sensor_msgs::image_encodings::MONO16) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
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
                    cloud.points.emplace_back(pt_map.point.x, pt_map.point.y, pt_map.point.z);
                }
            }

        } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            for (int v = 0; v < cv_ptr->image.rows; ++v) {
                for (int u = 0; u < cv_ptr->image.cols; ++u) {
                    float z = cv_ptr->image.at<float>(v, u);
                    if (std::isnan(z) || z < 0.1 || z > 20.0) continue;

                    float x = (u - cx) * z / fx;
                    float y = (v - cy) * z / fy;

                    geometry_msgs::PointStamped pt_cam, pt_map;
                    pt_cam.header = msg->header;
                    pt_cam.point.x = x;
                    pt_cam.point.y = y;
                    pt_cam.point.z = z;

                    tf2::doTransform(pt_cam, pt_map, tf_cam_to_map);
                    cloud.points.emplace_back(pt_map.point.x, pt_map.point.y, pt_map.point.z);
                }
            }

        } else {
            ROS_WARN_STREAM("Unsupported depth encoding: " << msg->encoding);
            return;
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();  

        cloud_msg.header.frame_id = "map";
        cloud_pub.publish(cloud_msg);
        ROS_INFO("Published point cloud with %lu points", cloud.points.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_to_cloud_node");
    ros::NodeHandle nh;
    DepthToCloudNode node(nh);
    ros::spin();
    return 0;
}


