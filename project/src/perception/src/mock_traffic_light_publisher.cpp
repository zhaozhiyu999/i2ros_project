// perception/src/mock_traffic_light_publisher.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_traffic_light_publisher");
    ros::NodeHandle nh;

    ros::Publisher light_pub = nh.advertise<std_msgs::String>("/perception/traffic_light_status", 10);

    ros::Rate rate(1.0);  // 1 Hz

    bool is_green = true;

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = is_green ? "green" : "red";
        light_pub.publish(msg);

        ROS_INFO_STREAM("[MOCK] Published traffic light: " << msg.data);

        is_green = !is_green;  // 每秒切换一次状态
        rate.sleep();
    }

    return 0;
}
