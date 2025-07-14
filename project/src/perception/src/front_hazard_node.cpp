#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class FrontHazardDetector {
public:
    FrontHazardDetector()
        : it_(nh_) {
        sem_sub_ = nh_.subscribe("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", 1, &FrontHazardDetector::semCallback, this);
        hazard_pub_ = nh_.advertise<std_msgs::Bool>("/perception/front_hazard", 1);//rostopic pub
        debug_pub_ = it_.advertise("/debug/front_hazard_roi", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sem_sub_;
    ros::Publisher hazard_pub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher debug_pub_;

    void semCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat sem_img;
        try {
            sem_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        // middle of semantic
        int w = sem_img.cols;
        int h = sem_img.rows;
        cv::Rect roi(w / 3, h / 2, w / 3, h / 2);  // down part
        cv::Mat roi_img = sem_img(roi);

        // count red pixle
        cv::Mat hsv_img, red_mask1, red_mask2, red_mask;
        cv::cvtColor(roi_img, hsv_img, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(hsv_img, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;

        int red_pixels = cv::countNonZero(red_mask);
        bool hazard = (red_pixels > 200);  // 

        // output the result
        std_msgs::Bool msg_out;
        msg_out.data = hazard;
        hazard_pub_.publish(msg_out);

        // Debug
        cv::Mat debug_img = sem_img.clone();
        cv::rectangle(debug_img, roi, hazard ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255), 2);
        debug_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img).toImageMsg());

        ROS_INFO("Hazard pixels: %d → hazard: %s", red_pixels, hazard ? "true" : "false");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "front_hazard_node");
    FrontHazardDetector node;
    ros::spin();
    return 0;
}

