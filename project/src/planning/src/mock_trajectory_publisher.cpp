// planning/src/mock_trajectory_publisher.cpp

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <msg_interfaces/Trajectory.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_trajectory_publisher");
    ros::NodeHandle nh;

    ros::Publisher traj_pub = nh.advertise<msg_interfaces::Trajectory>("/planning/trajectory", 10);
    ros::Rate rate(1.0);  // 1 Hz

    while (ros::ok())
    {
        msg_interfaces::Trajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();

        // 构造 3 个轨迹点
        for (int i = 0; i < 3; ++i)
        {
            geometry_msgs::Pose pose;
            pose.position.x = i * 1.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;

            traj_msg.poses.push_back(pose);
            traj_msg.velocities.push_back(2.0);         // m/s
            traj_msg.timestamps.push_back(i * 1.0);      // sec
        }

        traj_pub.publish(traj_msg);
        ROS_INFO("[MOCK] Published trajectory with %lu points", traj_msg.poses.size());

        rate.sleep();
    }

    return 0;
}
