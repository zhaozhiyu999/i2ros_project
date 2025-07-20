#include <ros/ros.h>
#include <simulation/VehicleControl.h>
#include <nav_msgs/Path.h>
#include <msg_interfaces/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

class AngleDebugExecutor {
public:
    AngleDebugExecutor() {
        // 发布控制指令
        control_pub_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
        
        // 订阅DWA输出
        dwa_trajectory_sub_ = nh_.subscribe<msg_interfaces::Trajectory>("/planning/trajectory", 1, 
            &AngleDebugExecutor::dwaTrajectoryCallback, this);
        dwa_path_sub_ = nh_.subscribe<nav_msgs::Path>("/planning/path", 1, 
            &AngleDebugExecutor::dwaPathCallback, this);
        
        // 订阅车辆状态
        vehicle_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vehicle/odometry", 1, 
            &AngleDebugExecutor::odomCallback, this);
        
        // 订阅紧急停车
        emergency_stop_sub_ = nh_.subscribe<std_msgs::Bool>("/decision/emergency_stop", 1, 
            &AngleDebugExecutor::emergencyStopCallback, this);
        
        // 参数
        nh_.param("control_gain", control_gain_, 2.0);     // 基础转向增益
        nh_.param("speed_gain", speed_gain_, 0.8);
        nh_.param("max_steering", max_steering_, 0.5);
        nh_.param("steering_sign", steering_sign_, -1.0);  // 反转转向符号
        nh_.param("use_dwa_heading", use_dwa_heading_, true);  // 是否使用DWA的朝向
        
        // 新增：更激进的二元转向控制参数
        nh_.param("steering_deadzone", steering_deadzone_, 1.5);      // 死区：1.5度（更大）
        nh_.param("small_angle_gain", small_angle_gain_, 0.0);        // 小角度增益（设为0，不转向）
        nh_.param("large_angle_gain", large_angle_gain_, 15.0);        // 大角度增益（很大）
        nh_.param("angle_transition", angle_transition_, 1.5);        // 过渡角度：1.5度（与死区相同）
        
        // 状态
        has_dwa_trajectory_ = false;
        has_dwa_path_ = false;
        emergency_stop_ = false;
        current_speed_ = 0.0;
        current_yaw_ = 0.0;
        
        ROS_INFO("=== Angle Debug DWA Executor Started ===");
        ROS_INFO("Binary steering control: deadzone=%.1f°, large_gain=%.1f", 
                 steering_deadzone_, large_angle_gain_);
        ROS_INFO("Strategy: NO steering <%.1f°, BIG steering >%.1f°", 
                 steering_deadzone_, angle_transition_);
        ROS_INFO("Steering sign: %.1f (CORRECTED for proper direction)", steering_sign_);
    }
    
    void spin() {
        ros::Rate rate(20);
        while (ros::ok()) {
            executeDWACommand();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_;
    ros::Subscriber dwa_trajectory_sub_;
    ros::Subscriber dwa_path_sub_;
    ros::Subscriber vehicle_odom_sub_;
    ros::Subscriber emergency_stop_sub_;
    
    // 数据
    msg_interfaces::Trajectory dwa_trajectory_;
    nav_msgs::Path dwa_path_;
    bool has_dwa_trajectory_;
    bool has_dwa_path_;
    double current_speed_;
    double current_yaw_;
    bool emergency_stop_;
    bool use_dwa_heading_;
    
    // 参数
    double control_gain_;
    double speed_gain_;
    double max_steering_;
    double steering_sign_;
    
    // 非线性转向控制参数
    double steering_deadzone_;     // 死区角度（度）
    double small_angle_gain_;      // 小角度增益
    double large_angle_gain_;      // 大角度增益
    double angle_transition_;      // 过渡角度（度）
    
    // ==================== 回调函数 ====================
    void dwaTrajectoryCallback(const msg_interfaces::Trajectory::ConstPtr& msg) {
        dwa_trajectory_ = *msg;
        has_dwa_trajectory_ = !msg->poses.empty();
        
        // 简化输出
        ROS_DEBUG_THROTTLE(1.0, "DWA trajectory: %lu points", msg->poses.size());
    }
    
    void dwaPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        dwa_path_ = *msg;
        has_dwa_path_ = !msg->poses.empty();
        
        // 简化输出
        ROS_DEBUG_THROTTLE(1.0, "DWA path: %lu points", msg->poses.size());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + 
                             pow(msg->twist.twist.linear.y, 2));
        
        // 获取当前朝向
        current_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        
        ROS_INFO_THROTTLE(2.0, "Vehicle current: pos(%.1f, %.1f) yaw=%.1f° speed=%.1f", 
                         msg->pose.pose.position.x, msg->pose.pose.position.y,
                         current_yaw_ * 180.0 / M_PI, current_speed_);
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        emergency_stop_ = msg->data;
    }
    
    // ==================== 核心执行逻辑 ====================
    void executeDWACommand() {
        simulation::VehicleControl cmd;
        
        if (emergency_stop_) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 1.0;
            cmd.Reserved = 0.0;
            
            ROS_INFO_THROTTLE(2.0, "🚨 Emergency stop");
        } else if (has_dwa_trajectory_ && !dwa_trajectory_.poses.empty()) {
            executeDebugTrajectoryCommand(cmd);
        } else if (has_dwa_path_ && !dwa_path_.poses.empty()) {
            executeDebugPathCommand(cmd);
        } else {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            
            ROS_WARN_THROTTLE(3.0, "No DWA command available");
        }
        
        control_pub_.publish(cmd);
    }
    
    // ==================== 调试轨迹执行 ====================
    void executeDebugTrajectoryCommand(simulation::VehicleControl& cmd) {
        if (dwa_trajectory_.poses.size() < 2) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            return;
        }
        
        // 选择目标点
        size_t target_idx = std::min((size_t)2, dwa_trajectory_.poses.size() - 1);
        
        const auto& current_pose = dwa_trajectory_.poses[0];
        const auto& target_pose = dwa_trajectory_.poses[target_idx];
        
        // 详细的角度分析
        analyzeAngles(current_pose, target_pose, cmd);
    }
    
    void executeDebugPathCommand(simulation::VehicleControl& cmd) {
        if (dwa_path_.poses.size() < 2) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            return;
        }
        
        // 选择目标点
        size_t target_idx = std::min((size_t)2, dwa_path_.poses.size() - 1);
        
        const auto& current_pose = dwa_path_.poses[0];
        const auto& target_pose = dwa_path_.poses[target_idx];
        
        // 详细的角度分析
        analyzeAngles(current_pose.pose, target_pose.pose, cmd);
    }
    
    // ==================== 详细角度分析 ====================
    void analyzeAngles(const geometry_msgs::Pose& current_pose, 
                      const geometry_msgs::Pose& target_pose, 
                      simulation::VehicleControl& cmd) {
        
        // 使用DWA给出的朝向
        double dwa_target_yaw = getYawFromQuaternion(target_pose.orientation);
        
        // 计算角度误差
        double yaw_error = normalizeAngle(dwa_target_yaw - current_yaw_);
        double yaw_error_deg = fabs(yaw_error * 180.0 / M_PI);
        
        // 非线性转向控制
        double steering = calculateNonlinearSteering(yaw_error, yaw_error_deg);
        
        // 获取目标速度
        double target_speed = 2.0;
        if (has_dwa_trajectory_ && dwa_trajectory_.velocities.size() > 0) {
            double dwa_speed = static_cast<double>(dwa_trajectory_.velocities[0]);
            target_speed = std::max(1.0, dwa_speed);
        }
        
        // 根据转向模式调整速度
        if (yaw_error_deg >= steering_deadzone_) {
            // 只要开始转向就减速
            target_speed *= 0.85;  // 转向时减速15%
        }
        
        double speed_error = target_speed - current_speed_;
        double throttle = speed_gain_ * speed_error;
        throttle = std::max(-1.0, std::min(1.0, throttle));
        
        cmd.Throttle = throttle;
        cmd.Steering = steering * steering_sign_;
        cmd.Brake = (throttle < -0.1) ? -throttle : 0.0;
        cmd.Reserved = 0.0;
        
        // 详细输出控制逻辑
        std::string control_mode = getControlMode(yaw_error_deg);
        ROS_INFO_THROTTLE(1.0, "Control: T=%.2f S=%.2f | Yaw: %.1f°→%.1f° (err=%.1f°) [%s]", 
                         throttle, cmd.Steering, 
                         current_yaw_ * 180.0 / M_PI, dwa_target_yaw * 180.0 / M_PI,
                         yaw_error * 180.0 / M_PI, control_mode.c_str());
    }
    
    // ==================== 优化的二元转向控制 ====================
    double calculateNonlinearSteering(double yaw_error, double yaw_error_deg) {
        // 优化的二元控制：小角度不转向，大角度快速响应
        if (yaw_error_deg < steering_deadzone_) {
            // 小于0.8度：完全不转向
            return 0.0;
        } else {
            // 大于0.8度：立即转向
            double steering = large_angle_gain_ * yaw_error;
            
            // 根据角度大小分级放大
            if (yaw_error_deg > 2.0) {
                steering *= 1.8;  // >2度：放大80%
            } else if (yaw_error_deg > 1.5) {
                steering *= 1.4;  // >1.5度：放大40%
            } else if (yaw_error_deg > 1.0) {
                steering *= 1.2;  // >1度：放大20%
            }
            // 0.8-1度：使用基础增益
            
            // 限制转向范围
            steering = std::max(-max_steering_, std::min(max_steering_, steering));
            
            return steering;
        }
    }
    
    // 获取当前控制模式描述（多级模式）
    std::string getControlMode(double yaw_error_deg) {
        if (yaw_error_deg < steering_deadzone_) {
            return "STRAIGHT";      // <0.8度：直行
        } else if (yaw_error_deg <= 1.0) {
            return "MILD";          // 0.8-1度：轻微转向
        } else if (yaw_error_deg <= 1.5) {
            return "NORMAL";        // 1-1.5度：正常转向
        } else if (yaw_error_deg <= 2.0) {
            return "STRONG";        // 1.5-2度：强转向
        } else {
            return "MAX";           // >2度：最大转向
        }
    }
    
    // ==================== 辅助函数 ====================
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "angle_debug_executor");
    
    AngleDebugExecutor executor;
    executor.spin();
    
    return 0;
}