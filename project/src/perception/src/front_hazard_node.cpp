#include <ros/ros.h> 
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class FrontHazardDetector {
public:
    FrontHazardDetector()
        : it_(nh_) {
        sem_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1, &FrontHazardDetector::semCallback, this);
        hazard_pub_ = nh_.advertise<std_msgs::Bool>("/perception/front_hazard", 1);
        angle_pub_ = nh_.advertise<std_msgs::Float32>("/perception/front_hazard_angle", 1);
        debug_pub_ = it_.advertise("/debug/front_hazard_roi", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sem_sub_;
    ros::Publisher hazard_pub_, angle_pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher debug_pub_;

    void semCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat sem_img;
        try {
            sem_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        int w = sem_img.cols;
        int h = sem_img.rows;
        cv::Rect roi(w / 3, h / 2, w / 3, h / 2);  // 中间下半部分
        cv::Mat roi_img = sem_img(roi);

        // 红色遮罩
        cv::Mat hsv_img, red_mask1, red_mask2, red_mask;
        cv::cvtColor(roi_img, hsv_img, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(hsv_img, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;

        int red_pixels = cv::countNonZero(red_mask);
        bool hazard = (red_pixels > 200);

        float angle_deg = 0.0;
        cv::Point center_point(-1, -1);

        if (red_pixels > 0) {
            cv::Moments m = cv::moments(red_mask, true);
            if (m.m00 > 0) {
                float cx = m.m10 / m.m00;
                float cx_img = cx + roi.x;

                float fov_deg = 90.0;  // 相机视角
                float angle_ratio = (cx_img - w / 2.0) / (w / 2.0);
                angle_deg = angle_ratio * (fov_deg / 2.0);

                center_point = cv::Point(static_cast<int>(cx_img), roi.y + roi.height / 2);
            }
        }

        // 发布布尔值
        std_msgs::Bool msg_out;
        msg_out.data = hazard;
        hazard_pub_.publish(msg_out);

        // 发布角度
        std_msgs::Float32 angle_msg;
        angle_msg.data = angle_deg;
        angle_pub_.publish(angle_msg);

        // 可视化
        cv::Mat debug_img = sem_img.clone();
        cv::rectangle(debug_img, roi, hazard ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255), 2);
        if (center_point.x > 0)
            cv::circle(debug_img, center_point, 5, cv::Scalar(255, 0, 0), -1);  // 蓝色点

        cv::putText(debug_img, "Angle: " + std::to_string(angle_deg) + " deg",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 255), 2);

        debug_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img).toImageMsg());

        ROS_INFO("Hazard: %s | Pixels: %d | Angle: %.2f deg",
                 hazard ? "true" : "false", red_pixels, angle_deg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "front_hazard_node");
    FrontHazardDetector node;
    ros::spin();
    return 0;
}


