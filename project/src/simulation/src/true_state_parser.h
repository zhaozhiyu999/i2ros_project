#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "unity_stream_parser.h"

class TrueStateParser : public UnityStreamParser {
public:
  TrueStateParser() : nh_("~") { };

  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float rx, ry, rz;

    px = stream_reader.ReadFloat();
    py = stream_reader.ReadFloat();
    pz = stream_reader.ReadFloat();
    
    qx = stream_reader.ReadFloat();
    qy = stream_reader.ReadFloat();
    qz = stream_reader.ReadFloat();
    qw = stream_reader.ReadFloat();

    vx = stream_reader.ReadFloat();
    vy = stream_reader.ReadFloat();
    vz = stream_reader.ReadFloat();

    rx = stream_reader.ReadFloat();
    ry = stream_reader.ReadFloat();
    rz = stream_reader.ReadFloat();

    if(pose_publishers_.find(header.name) == pose_publishers_.end()) {
      pose_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::PoseStamped>(header.name + "/pose", 10)));
      twist_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::TwistStamped>(header.name + "/twist", 10)));
    }    

    if(last_time_stamp == header.timestamp + time_offset)
    // Avoid duplicated time stamps
    {
      return false;
    }
    else
    {
      last_time_stamp = header.timestamp + time_offset;
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world"; //"odom_nav";
    pose_msg.header.stamp = ros::Time(last_time_stamp);
    
    pose_msg.pose.position.x = pz;
    pose_msg.pose.position.y = -px;
    pose_msg.pose.position.z = py;

    pose_msg.pose.orientation.x = -qz;
    pose_msg.pose.orientation.y = qx;
    pose_msg.pose.orientation.z = -qy;
    pose_msg.pose.orientation.w = qw;

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "world"; //"odom_nav";
    twist_msg.header.stamp = pose_msg.header.stamp;
    
    twist_msg.twist.linear.x = vz;    
    twist_msg.twist.linear.y = -vx;
    twist_msg.twist.linear.z = vy;

    twist_msg.twist.angular.x = rz;
    twist_msg.twist.angular.y = -rx;
    twist_msg.twist.angular.z = -ry;

    pose_publishers_[header.name].publish(pose_msg);
    twist_publishers_[header.name].publish(twist_msg);

    geometry_msgs::TransformStamped::Ptr tf(new geometry_msgs::TransformStamped);
    tf->header.stamp = pose_msg.header.stamp;
    tf->header.frame_id = "world";
    tf->child_frame_id = "OurCar/INS";

    tf->transform.translation.x = pose_msg.pose.position.x;
    tf->transform.translation.y = pose_msg.pose.position.y;
    tf->transform.translation.z = pose_msg.pose.position.z;

    tf->transform.rotation.x = pose_msg.pose.orientation.x;
    tf->transform.rotation.y = pose_msg.pose.orientation.y;
    tf->transform.rotation.z = pose_msg.pose.orientation.z;
    tf->transform.rotation.w = pose_msg.pose.orientation.w;

    tf_broadcaster.sendTransform(*tf);
    return true;
  }

private:
  std::unordered_map<std::string, ros::Publisher> pose_publishers_;
  std::unordered_map<std::string, ros::Publisher> twist_publishers_;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  ros::NodeHandle nh_;
  double last_time_stamp = 0.0;
};
