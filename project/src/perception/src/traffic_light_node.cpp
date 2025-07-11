#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

class TrafficLightDetector
{
public:
    TrafficLightDetector()
        : it_(nh_)
    {
        rgb_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraRight/image_raw", 1, &TrafficLightDetector::rgbCallback, this);
        sem_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1, &TrafficLightDetector::semCallback, this);
        rgb_info_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraRight/camera_info", 1, &TrafficLightDetector::rgbInfoCallback, this);
        sem_info_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/camera_info", 1, &TrafficLightDetector::semInfoCallback, this);
        pub_ = nh_.advertise<std_msgs::String>("/perception/traffic_light_status", 1);
        debug_pub_ = it_.advertise("/debug/traffic_light_roi", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber rgb_sub_, sem_sub_, rgb_info_sub_, sem_info_sub_;
    ros::Publisher pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher debug_pub_;

    sensor_msgs::CameraInfo rgb_info_, sem_info_;
    image_geometry::PinholeCameraModel model_rgb_, model_sem_;

    cv::Mat latest_rgb_, latest_sem_;

    void rgbInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        rgb_info_ = *msg;
        model_rgb_.fromCameraInfo(msg);
    }

    void semInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        sem_info_ = *msg;
        model_sem_.fromCameraInfo(msg);
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            latest_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }
        process();
    }

    void semCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            latest_sem_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }
    }

    void process()
    {
        if (latest_rgb_.empty() || latest_sem_.empty()) return;
        if (!model_rgb_.initialized() || !model_sem_.initialized()) return;

        cv::Mat hsv_sem;
        cv::cvtColor(latest_sem_, hsv_sem, cv::COLOR_BGR2HSV);
        cv::Mat yellow_mask;
        cv::inRange(hsv_sem, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255), yellow_mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) return;

        cv::Rect best_box;
        double min_dist = 1e9;
        cv::Point image_center(latest_sem_.cols / 2, latest_sem_.rows / 2);

        for (const auto& contour : contours)
        {
            cv::Rect box = cv::boundingRect(contour);
            if (box.area() < 10) continue;
            cv::Point center = (box.tl() + box.br()) * 0.5;
            double dist = cv::norm(center - image_center);
            if (dist < min_dist) {
                min_dist = dist;
                best_box = box;
            }
        }

        // 等比例缩放 best_box 到 RGB 图像中
    
        double scale_x = latest_rgb_.cols / static_cast<double>(latest_sem_.cols);
        double scale_y = latest_rgb_.rows / static_cast<double>(latest_sem_.rows);

        int rgb_x = static_cast<int>(best_box.x * scale_x)-3;
        int rgb_y = static_cast<int>(best_box.y * scale_y);
        int rgb_w = static_cast<int>(best_box.width * scale_x)+5;
        int rgb_h = static_cast<int>(best_box.height * scale_y);

        rgb_x = std::max(0, rgb_x);
        rgb_y = std::max(0, rgb_y);
        rgb_w = std::min(rgb_w, latest_rgb_.cols - rgb_x);
        rgb_h = std::min(rgb_h, latest_rgb_.rows - rgb_y);
        if (rgb_w <= 0 || rgb_h <= 0) return;

        cv::Rect roi(rgb_x, rgb_y, rgb_w, rgb_h);
        cv::Mat hsv_rgb;
        cv::cvtColor(latest_rgb_, hsv_rgb, cv::COLOR_BGR2HSV);
        cv::Mat region = hsv_rgb(roi);

        // Red capture
        cv::Mat red1, red2, red_mask;
        cv::inRange(region, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red1);
        cv::inRange(region, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), red2);
        red_mask = red1 | red2;
        //Green capture
        cv::Mat green_mask;
        cv::inRange(region, cv::Scalar(35, 100, 100), cv::Scalar(95, 255, 255), green_mask);

        int red_count = cv::countNonZero(red_mask);
        int green_count = cv::countNonZero(green_mask);

        cv::Scalar mean_hsv = cv::mean(region);
        ROS_INFO("Red: %d, Green: %d | ROI HSV avg: H=%.1f S=%.1f V=%.1f",
                 red_count, green_count, mean_hsv[0], mean_hsv[1], mean_hsv[2]);

        std_msgs::String msg;
        if (red_count > 10) msg.data = "red";
        else if (green_count > 10) msg.data = "green";
        else msg.data = "unknown";

        pub_.publish(msg);

        // Debug 显示
        cv::Mat dbg = latest_rgb_.clone();
        cv::rectangle(dbg, roi, cv::Scalar(255, 0, 0), 2);  // 仅用于可视化
        debug_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", dbg).toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_node");
    TrafficLightDetector node;
    ros::spin();
    return 0;
}

