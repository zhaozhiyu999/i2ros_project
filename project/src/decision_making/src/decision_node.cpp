#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>

class SimplifiedDecisionNode
{
public:
    SimplifiedDecisionNode()
    {
        // ========== 订阅感知模块话题 ==========
        traffic_light_sub_ = nh_.subscribe("/perception/traffic_light_status", 1,
                                          &SimplifiedDecisionNode::trafficLightCallback, this);
        front_hazard_sub_ = nh_.subscribe("/perception/front_hazard", 1,
                                         &SimplifiedDecisionNode::frontHazardCallback, this);

        // ========== 发布决策结果 ==========
        behavior_command_pub_ = nh_.advertise<std_msgs::String>("/decision/behavior_command", 1);
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/decision/emergency_stop", 1);
        
        // ========== 可视化发布器 ==========
        decision_status_pub_ = nh_.advertise<visualization_msgs::Marker>("/decision/status_display", 1);

        // ========== 状态初始化 ==========
        traffic_light_red_ = false;
        front_hazard_detected_ = false;
        current_behavior_ = STRAIGHT;
        behavior_change_time_ = ros::Time::now();

        ROS_INFO("Simplified Decision Node started");
        ROS_INFO("Behavior states: STRAIGHT, AVOIDANCE, EMERGENCY_STOP");
    }

    void spin()
    {
        ros::Rate rate(10);  // 10 Hz
        
        while (ros::ok())
        {
            // ========== 主决策逻辑 ==========
            BehaviorState new_behavior = decideBehavior();
            
            // ========== 检查行为变化 ==========
            if (new_behavior != current_behavior_) {
                logBehaviorChange(current_behavior_, new_behavior);
                current_behavior_ = new_behavior;
                behavior_change_time_ = ros::Time::now();
            }
            
            // ========== 发布决策结果 ==========
            publishBehaviorCommand();
            publishEmergencyStop();
            publishStatusVisualization();

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // ========== 行为状态枚举 ==========
    enum BehaviorState {
        STRAIGHT = 0,        // 直行
        AVOIDANCE = 1,       // 避让
        EMERGENCY_STOP = 2   // 急停
    };

    // ========== ROS相关 ==========
    ros::NodeHandle nh_;
    ros::Subscriber traffic_light_sub_;
    ros::Subscriber front_hazard_sub_;
    ros::Publisher behavior_command_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher decision_status_pub_;

    // ========== 状态变量 ==========
    bool traffic_light_red_;
    bool front_hazard_detected_;
    BehaviorState current_behavior_;
    ros::Time behavior_change_time_;

    // ========== 回调函数 ==========
    void trafficLightCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string light_status = msg->data;
        std::transform(light_status.begin(), light_status.end(), 
                      light_status.begin(), ::tolower);
        
        bool prev_red = traffic_light_red_;
        traffic_light_red_ = (light_status == "red");
        
        if (prev_red != traffic_light_red_) {
            if (traffic_light_red_) {
                ROS_WARN("🔴 RED LIGHT DETECTED - Emergency stop required");
            } else {
                ROS_INFO("🟢 GREEN LIGHT - Normal driving resumed");
            }
        }
    }

    void frontHazardCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        bool prev_hazard = front_hazard_detected_;
        front_hazard_detected_ = msg->data;
        
        if (prev_hazard != front_hazard_detected_) {
            if (front_hazard_detected_) {
                ROS_WARN("⚠️  FRONT HAZARD DETECTED - Avoidance maneuver required");
            } else {
                ROS_INFO("✅ Front hazard cleared - Normal driving resumed");
            }
        }
    }

    // ========== 核心决策逻辑 ==========
    BehaviorState decideBehavior()
    {
        // 决策优先级：
        // 1. 红绿灯（最高优先级）- 交通规则必须遵守
        // 2. 前方障碍物 - 安全避让
        // 3. 正常直行
        
        if (traffic_light_red_) {
            return EMERGENCY_STOP;  // 红灯必须停车
        }
        
        if (front_hazard_detected_) {
            return AVOIDANCE;       // 有障碍物需要避让
        }
        
        return STRAIGHT;            // 正常直行
    }

    // ========== 发布决策结果 ==========
    void publishBehaviorCommand()
    {
        std_msgs::String command_msg;
        command_msg.data = behaviorToString(current_behavior_);
        behavior_command_pub_.publish(command_msg);
    }

    void publishEmergencyStop()
    {
        std_msgs::Bool emergency_msg;
        emergency_msg.data = (current_behavior_ == EMERGENCY_STOP);
        emergency_stop_pub_.publish(emergency_msg);
    }

    // ========== 状态可视化 ==========
    void publishStatusVisualization()
    {
        visualization_msgs::Marker status_marker;
        status_marker.header.frame_id = "base_link";
        status_marker.header.stamp = ros::Time::now();
        status_marker.ns = "decision_status";
        status_marker.id = 0;
        status_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        status_marker.action = visualization_msgs::Marker::ADD;

        // 设置显示位置（车辆上方）
        status_marker.pose.position.x = 0.0;
        status_marker.pose.position.y = 0.0;
        status_marker.pose.position.z = 3.0;
        status_marker.pose.orientation.w = 1.0;

        // 设置文字大小
        status_marker.scale.z = 1.2;

        // 根据当前行为设置颜色和文字
        std::string status_text = "DECISION: " + behaviorToString(current_behavior_);
        
        switch (current_behavior_) {
            case EMERGENCY_STOP:
                status_marker.color.r = 1.0; status_marker.color.g = 0.0; status_marker.color.b = 0.0;
                status_text += "\n🔴 EMERGENCY STOP";
                status_text += "\nReason: " + std::string(traffic_light_red_ ? "Red Light" : "Unknown");
                break;
                
            case AVOIDANCE:
                status_marker.color.r = 1.0; status_marker.color.g = 0.6; status_marker.color.b = 0.0;
                status_text += "\n⚠️  AVOIDANCE";
                status_text += "\nReason: Front Hazard";
                break;
                
            case STRAIGHT:
                status_marker.color.r = 0.0; status_marker.color.g = 1.0; status_marker.color.b = 0.0;
                status_text += "\n✅ STRAIGHT";
                status_text += "\nStatus: Normal Driving";
                break;
        }
        
        status_marker.color.a = 1.0;

        // 添加传感器状态信息
        status_text += "\n" + std::string(20, '-');
        status_text += "\n🚦 Traffic Light: " + std::string(traffic_light_red_ ? "RED" : "GREEN");
        status_text += "\n🚗 Front Hazard: " + std::string(front_hazard_detected_ ? "YES" : "NO");
        
        // 添加行为持续时间
        double duration = (ros::Time::now() - behavior_change_time_).toSec();
        status_text += "\n⏱️  Duration: " + std::to_string((int)duration) + "s";

        status_marker.text = status_text;
        decision_status_pub_.publish(status_marker);
    }

    // ========== 辅助函数 ==========
    std::string behaviorToString(BehaviorState behavior)
    {
        switch (behavior) {
            case STRAIGHT:      return "STRAIGHT";
            case AVOIDANCE:     return "AVOIDANCE";
            case EMERGENCY_STOP: return "EMERGENCY_STOP";
            default:            return "UNKNOWN";
        }
    }

    void logBehaviorChange(BehaviorState from, BehaviorState to)
    {
        std::string from_str = behaviorToString(from);
        std::string to_str = behaviorToString(to);
        
        ROS_INFO("🔄 BEHAVIOR CHANGE: %s → %s", from_str.c_str(), to_str.c_str());
        
        // 详细日志
        switch (to) {
            case EMERGENCY_STOP:
                ROS_WARN("🛑 Executing emergency stop procedure");
                break;
            case AVOIDANCE:
                ROS_WARN("🔀 Initiating avoidance maneuver");
                break;
            case STRAIGHT:
                ROS_INFO("➡️  Resuming straight driving");
                break;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simplified_decision_node");
    
    SimplifiedDecisionNode decision_node;
    
    ROS_INFO("===== Simplified Decision Node Ready =====");
    ROS_INFO("Input Topics:");
    ROS_INFO("  - /perception/traffic_light_status");
    ROS_INFO("  - /perception/front_hazard");
    ROS_INFO("Output Topics:");
    ROS_INFO("  - /decision/behavior_command");
    ROS_INFO("  - /decision/emergency_stop");
    ROS_INFO("  - /decision/status_display");
    ROS_INFO("==========================================");
    
    decision_node.spin();
    
    return 0;
}