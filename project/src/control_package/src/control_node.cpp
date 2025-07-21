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
#include <deque>

class StableDWAExecutor {
public:
    StableDWAExecutor() {
        // 发布控制指令
        control_pub_ = nh_.advertise<simulation::VehicleControl>("car_command", 1);
        
        // 订阅DWA输出
        dwa_trajectory_sub_ = nh_.subscribe<msg_interfaces::Trajectory>("/planning/trajectory", 1, 
            &StableDWAExecutor::dwaTrajectoryCallback, this);
        dwa_path_sub_ = nh_.subscribe<nav_msgs::Path>("/planning/path", 1, 
            &StableDWAExecutor::dwaPathCallback, this);
        
        // 订阅车辆状态
        vehicle_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vehicle/odometry", 1, 
            &StableDWAExecutor::odomCallback, this);
        
        // 订阅紧急停车
        emergency_stop_sub_ = nh_.subscribe<std_msgs::Bool>("/decision/emergency_stop", 1, 
            &StableDWAExecutor::emergencyStopCallback, this);
        
        // ==================== 用户指定参数配置 ====================
        // 基础控制参数
        nh_.param("base_gain", base_gain_, 25.0);                      // 基础增益
        nh_.param("speed_gain", speed_gain_, 0.8);
        nh_.param("max_steering", max_steering_, 0.7);                 // 最大转向
        nh_.param("steering_sign", steering_sign_, -1.0);
        
        // 角度阈值
        nh_.param("micro_angle_threshold", micro_angle_threshold_, 0.6);    // 噪声死区
        nh_.param("small_angle_threshold", small_angle_threshold_, 1.0);    
        nh_.param("medium_angle_threshold", medium_angle_threshold_, 1.7);  
        
        // 分级增益
        nh_.param("micro_gain", micro_gain_, 20.0);                    // 微小角度增益
        nh_.param("small_gain", small_gain_, 35.0);                    // 小角度增益  
        nh_.param("medium_gain", medium_gain_, 50.0);                  // 中角度增益
        nh_.param("large_gain", large_gain_, 65.0);                    // 大角度增益
        
        // ==================== 新增：稳定性控制参数 ====================
        nh_.param("stability_mode", stability_mode_, true);                    // 启用稳定模式
        nh_.param("oscillation_detection", oscillation_detection_, true);     // 振荡检测
        nh_.param("steering_smoothing_window", smoothing_window_size_, 5);     // 平滑窗口
        nh_.param("max_steering_change_rate", max_steering_change_rate_, 1.5); // 最大转向变化率
        nh_.param("damping_factor", damping_factor_, 0.3);                     // 阻尼系数
        
        // PID控制参数（增强响应）
        nh_.param("kp", kp_, 25.0);                                    // 增大比例增益
        nh_.param("ki", ki_, 1.0);                                     // 适中积分增益  
        nh_.param("kd", kd_, 3.0);                                     // 增强微分增益
        nh_.param("integral_limit", integral_limit_, 0.3);             // 增大积分限制
        
        // 路径预测参数
        nh_.param("path_prediction_enabled", path_prediction_, true);   // 路径预测
        nh_.param("lookahead_distance", lookahead_distance_, 3.0);     // 前瞻距离
        nh_.param("path_smoothing", path_smoothing_, true);            // 路径平滑
        
        // 高级平滑参数（减少过度平滑）
        nh_.param("steering_filter_alpha", steering_filter_alpha_, 0.6);    // 减少平滑强度
        nh_.param("acceleration_limit", acceleration_limit_, 2.5);          // 放宽限制
        nh_.param("min_steering_output", min_steering_output_, 0.08);       // 提高最小输出
        
        // 速度自适应（更积极响应）
        nh_.param("low_speed_boost", low_speed_boost_, 1.4);           // 增强低速响应
        nh_.param("high_speed_reduce", high_speed_reduce_, 0.8);       // 减少高速减弱
        nh_.param("speed_threshold", speed_threshold_, 6.0);           // 保持速度阈值
        
        // 状态初始化
        has_dwa_trajectory_ = false;
        has_dwa_path_ = false;
        emergency_stop_ = false;
        current_speed_ = 0.0;
        current_yaw_ = 0.0;
        previous_steering_ = 0.0;
        last_control_time_ = ros::Time::now();
        
        // PID状态
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        
        // 振荡检测
        oscillation_counter_ = 0;
        last_steering_direction_ = 0;
        
        // 历史记录初始化
        steering_history_.clear();
        error_history_.clear();
        
        ROS_INFO("=== Stable DWA Executor - Anti-Oscillation ===");
        ROS_INFO("Stability mode: %s, Oscillation detection: %s", 
                 stability_mode_ ? "ON" : "OFF", oscillation_detection_ ? "ON" : "OFF");
        ROS_INFO("PID gains: Kp=%.1f Ki=%.1f Kd=%.1f", kp_, ki_, kd_);
        ROS_INFO("Smoothing: window=%d, max_change=%.1f, damping=%.1f", 
                 smoothing_window_size_, max_steering_change_rate_, damping_factor_);
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
    
    // DWA数据
    msg_interfaces::Trajectory dwa_trajectory_;
    nav_msgs::Path dwa_path_;
    bool has_dwa_trajectory_;
    bool has_dwa_path_;
    
    // 车辆状态
    double current_speed_;
    double current_yaw_;
    bool emergency_stop_;
    
    // 控制状态
    double previous_steering_;
    ros::Time last_control_time_;
    
    // 基础参数
    double base_gain_, speed_gain_, max_steering_, steering_sign_;
    double micro_angle_threshold_, small_angle_threshold_, medium_angle_threshold_;
    double micro_gain_, small_gain_, medium_gain_, large_gain_;
    double steering_filter_alpha_, acceleration_limit_, min_steering_output_;
    double low_speed_boost_, high_speed_reduce_, speed_threshold_;
    
    // ==================== 新增：稳定性控制变量 ====================
    bool stability_mode_;
    bool oscillation_detection_;
    bool path_prediction_;
    bool path_smoothing_;
    
    // PID控制
    double kp_, ki_, kd_;
    double integral_error_;
    double previous_error_;
    double integral_limit_;
    
    // 振荡检测
    int oscillation_counter_;
    int last_steering_direction_;
    int smoothing_window_size_;
    double max_steering_change_rate_;
    double damping_factor_;
    double lookahead_distance_;
    
    // 历史数据
    std::deque<double> steering_history_;
    std::deque<double> error_history_;
    
    // ==================== 回调函数 ====================
    void dwaTrajectoryCallback(const msg_interfaces::Trajectory::ConstPtr& msg) {
        dwa_trajectory_ = *msg;
        has_dwa_trajectory_ = !msg->poses.empty();
        ROS_DEBUG_THROTTLE(2.0, "DWA trajectory: %lu points", msg->poses.size());
    }
    
    void dwaPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        dwa_path_ = *msg;
        has_dwa_path_ = !msg->poses.empty();
        ROS_DEBUG_THROTTLE(2.0, "DWA path: %lu points", msg->poses.size());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_speed_ = sqrt(pow(msg->twist.twist.linear.x, 2) + 
                             pow(msg->twist.twist.linear.y, 2));
        current_yaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
        
        ROS_INFO_THROTTLE(3.0, "Vehicle: pos(%.1f,%.1f) yaw=%.1f° speed=%.1fm/s", 
                         msg->pose.pose.position.x, msg->pose.pose.position.y,
                         current_yaw_ * 180.0 / M_PI, current_speed_);
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        emergency_stop_ = msg->data;
        if (emergency_stop_) {
            ROS_WARN("🚨 Emergency stop activated!");
            resetControlStates();  // 重置控制状态
        }
    }
    
    // ==================== 主要执行逻辑 ====================
    void executeDWACommand() {
        simulation::VehicleControl cmd;
        
        if (emergency_stop_) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 1.0;
            cmd.Reserved = 0.0;
            ROS_INFO_THROTTLE(2.0, "🚨 Emergency stop active");
        } else if (has_dwa_trajectory_ && !dwa_trajectory_.poses.empty()) {
            executeStableTrajectoryCommand(cmd);
        } else if (has_dwa_path_ && !dwa_path_.poses.empty()) {
            executeStablePathCommand(cmd);
        } else {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            ROS_WARN_THROTTLE(3.0, "No DWA command available - gentle stop");
            resetControlStates();
        }
        
        control_pub_.publish(cmd);
    }
    
    // ==================== 稳定轨迹执行 ====================
    void executeStableTrajectoryCommand(simulation::VehicleControl& cmd) {
        if (dwa_trajectory_.poses.size() < 2) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            return;
        }
        
        // 计算角度误差
        double yaw_error, yaw_error_deg;
        calculateAngleError(yaw_error, yaw_error_deg);
        
        // 稳定转向控制
        double steering = calculateStableSteering(yaw_error, yaw_error_deg);
        
        // 速度控制
        double target_speed = getTargetSpeed();
        double speed_error = target_speed - current_speed_;
        double throttle = speed_gain_ * speed_error;
        
        // 根据稳定性调整速度（减少过度减速）
        if (detectInstability()) {
            throttle *= 0.85;  // 减少减速幅度
            ROS_WARN_THROTTLE(1.0, "⚠️  Instability detected - slight speed reduction");
        } else if (yaw_error_deg > medium_angle_threshold_) {
            throttle *= 0.9;   // 减少大角度减速
        }
        
        throttle = std::max(-1.0, std::min(1.0, throttle));
        
        cmd.Throttle = throttle;
        cmd.Steering = steering * steering_sign_;
        cmd.Brake = (throttle < -0.1) ? -throttle : 0.0;
        cmd.Reserved = 0.0;
        
        // 更新历史记录
        updateControlHistory(steering, yaw_error);
        
        // 调试输出
        printStabilityDebug(yaw_error, yaw_error_deg, steering, target_speed);
    }
    
    void executeStablePathCommand(simulation::VehicleControl& cmd) {
        if (dwa_path_.poses.size() < 2) {
            cmd.Throttle = 0.0;
            cmd.Steering = 0.0;
            cmd.Brake = 0.3;
            cmd.Reserved = 0.0;
            return;
        }
        
        // 使用路径计算角度误差
        double yaw_error, yaw_error_deg;
        calculatePathAngleError(yaw_error, yaw_error_deg);
        
        // 稳定转向控制
        double steering = calculateStableSteering(yaw_error, yaw_error_deg);
        
        // 速度控制
        double target_speed = getTargetSpeed();
        double speed_error = target_speed - current_speed_;
        double throttle = speed_gain_ * speed_error;
        throttle = std::max(-1.0, std::min(1.0, throttle));
        
        cmd.Throttle = throttle;
        cmd.Steering = steering * steering_sign_;
        cmd.Brake = (throttle < -0.1) ? -throttle : 0.0;
        cmd.Reserved = 0.0;
        
        updateControlHistory(steering, yaw_error);
        printStabilityDebug(yaw_error, yaw_error_deg, steering, target_speed);
    }
    
    // ==================== 核心：稳定转向算法 ====================
    double calculateStableSteering(double yaw_error, double yaw_error_deg) {
        double steering = 0.0;
        
        if (stability_mode_) {
            // 使用PID控制替代简单比例控制
            steering = calculatePIDSteering(yaw_error, yaw_error_deg);
        } else {
            // 传统分级控制（但增益更保守）
            steering = calculateConservativeSteering(yaw_error, yaw_error_deg);
        }
        
        // 振荡检测和抑制
        if (oscillation_detection_ && detectOscillation()) {
            steering = applyOscillationSuppression(steering);
        }
        
        // 路径预测修正
        if (path_prediction_) {
            steering = applyPathPredictionCorrection(steering);
        }
        
        // 多层平滑处理
        steering = applyAdvancedSmoothing(steering);
        
        // 限制转向范围
        steering = std::max(-max_steering_, std::min(max_steering_, steering));
        
        return steering;
    }
    
    // ==================== PID转向控制 ====================
    double calculatePIDSteering(double yaw_error, double yaw_error_deg) {
        if (yaw_error_deg < micro_angle_threshold_) {
            return 0.0;  // 噪声死区
        }
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time_).toSec();
        dt = std::max(dt, 0.001);
        
        // 比例项
        double proportional = kp_ * yaw_error;
        
        // 积分项（限制积分饱和）
        integral_error_ += yaw_error * dt;
        integral_error_ = std::max(-integral_limit_, std::min(integral_limit_, integral_error_));
        double integral = ki_ * integral_error_;
        
        // 微分项（增强阻尼）
        double derivative = kd_ * (yaw_error - previous_error_) / dt;
        
        // 组合PID输出
        double pid_output = proportional + integral + derivative;
        
        // 根据角度大小调整增益
        double angle_factor = getAngleAdaptiveFactor(yaw_error_deg);
        pid_output *= angle_factor;
        
        // 更新状态
        previous_error_ = yaw_error;
        
        return pid_output;
    }
    
    // ==================== 保守分级控制 ====================
    double calculateConservativeSteering(double yaw_error, double yaw_error_deg) {
        if (yaw_error_deg < micro_angle_threshold_) {
            return 0.0;
        } else if (yaw_error_deg < small_angle_threshold_) {
            return micro_gain_ * yaw_error;
        } else if (yaw_error_deg < medium_angle_threshold_) {
            return small_gain_ * yaw_error;
        } else if (yaw_error_deg < 2.0) {
            return medium_gain_ * yaw_error;
        } else {
            return large_gain_ * yaw_error;
        }
    }
    
    // ==================== 振荡检测和抑制 ====================
    bool detectOscillation() {
        if (steering_history_.size() < 4) return false;
        
        // 检测连续的方向变化
        int direction_changes = 0;
        for (size_t i = 1; i < steering_history_.size(); ++i) {
            if ((steering_history_[i] > 0) != (steering_history_[i-1] > 0)) {
                direction_changes++;
            }
        }
        
        // 如果方向变化过于频繁，认为是振荡
        return direction_changes >= 3;
    }
    
    double applyOscillationSuppression(double steering) {
        oscillation_counter_++;
        
        if (oscillation_counter_ > 5) {  // 提高阈值，减少误触发
            // 应用适中阻尼，保持响应能力
            steering *= (1.0 - damping_factor_ * 0.5);  // 减少阻尼强度
            ROS_WARN_THROTTLE(1.0, "🔧 Mild oscillation suppression: damping=%.1f", damping_factor_ * 0.5);
            
            // 轻微重置积分项
            integral_error_ *= 0.8;
            
            if (oscillation_counter_ > 15) {  // 增加重置阈值
                oscillation_counter_ = 0;
            }
        }
        
        return steering;
    }
    
    // ==================== 路径预测修正 ====================
    double applyPathPredictionCorrection(double steering) {
        if (!has_dwa_trajectory_ || dwa_trajectory_.poses.size() < 3) {
            return steering;
        }
        
        // 计算前瞻角度误差
        double lookahead_error = calculateLookaheadError();
        
        // 如果前瞻显示需要相反方向的转向，减小当前转向
        if ((steering > 0 && lookahead_error < -0.5) || 
            (steering < 0 && lookahead_error > 0.5)) {
            steering *= 0.7;  // 减小转向强度
            ROS_DEBUG_THROTTLE(1.0, "🔮 Path prediction correction applied");
        }
        
        return steering;
    }
    
    // ==================== 高级平滑处理 ====================
    double applyAdvancedSmoothing(double target_steering) {
        // 滑动窗口平均
        steering_history_.push_back(target_steering);
        if (steering_history_.size() > smoothing_window_size_) {
            steering_history_.pop_front();
        }
        
        double windowed_average = 0.0;
        for (double s : steering_history_) {
            windowed_average += s;
        }
        windowed_average /= steering_history_.size();
        
        // 变化率限制
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time_).toSec();
        dt = std::max(dt, 0.001);
        
        double max_change = max_steering_change_rate_ * dt;
        double steering_change = windowed_average - previous_steering_;
        
        if (fabs(steering_change) > max_change) {
            steering_change = (steering_change > 0) ? max_change : -max_change;
        }
        
        // 组合平滑
        double smooth_steering = previous_steering_ + steering_change;
        smooth_steering = steering_filter_alpha_ * smooth_steering + 
                         (1.0 - steering_filter_alpha_) * previous_steering_;
        
        // 更新状态
        previous_steering_ = smooth_steering;
        last_control_time_ = current_time;
        
        return smooth_steering;
    }
    
    // ==================== 辅助函数 ====================
    
    void calculateAngleError(double& yaw_error, double& yaw_error_deg) {
        size_t target_idx = selectOptimalTargetIndex(dwa_trajectory_.poses.size());
        double target_yaw = getYawFromQuaternion(dwa_trajectory_.poses[target_idx].orientation);
        yaw_error = normalizeAngle(target_yaw - current_yaw_);
        yaw_error_deg = fabs(yaw_error * 180.0 / M_PI);
    }
    
    void calculatePathAngleError(double& yaw_error, double& yaw_error_deg) {
        size_t target_idx = selectOptimalTargetIndex(dwa_path_.poses.size());
        double target_yaw = getYawFromQuaternion(dwa_path_.poses[target_idx].pose.orientation);
        yaw_error = normalizeAngle(target_yaw - current_yaw_);
        yaw_error_deg = fabs(yaw_error * 180.0 / M_PI);
    }
    
    size_t selectOptimalTargetIndex(size_t total_points) {
        if (total_points <= 2) return std::min((size_t)1, total_points - 1);
        
        // 更积极的目标点选择
        size_t lookahead_points;
        if (current_speed_ < 3.0) {
            lookahead_points = 2;  // 低速：看稍远一点
        } else if (current_speed_ < 6.0) {
            lookahead_points = 3;  // 中速：正常距离
        } else {
            lookahead_points = 4;  // 高速：看得更远
        }
        
        return std::min(lookahead_points, total_points - 1);
    }
    
    double getTargetSpeed() {
        double target_speed = 6.0;  // 降低默认速度，提高稳定性
        
        if (has_dwa_trajectory_ && !dwa_trajectory_.velocities.empty()) {
            double dwa_speed = static_cast<double>(dwa_trajectory_.velocities[0]);
            target_speed = std::max(1.5, std::min(8.0, dwa_speed));  // 限制速度范围
        }
        
        return target_speed;
    }
    
    double getAngleAdaptiveFactor(double yaw_error_deg) {
        if (yaw_error_deg < small_angle_threshold_) {
            return 1.0;  // 小角度：正常增益
        } else if (yaw_error_deg < medium_angle_threshold_) {
            return 1.3;  // 中角度：增强增益
        } else {
            return 1.6;  // 大角度：显著增强
        }
    }
    
    double calculateLookaheadError() {
        if (dwa_trajectory_.poses.size() < 5) return 0.0;
        
        double lookahead_yaw = getYawFromQuaternion(dwa_trajectory_.poses[4].orientation);
        return normalizeAngle(lookahead_yaw - current_yaw_);
    }
    
    bool detectInstability() {
        if (error_history_.size() < 3) return false;
        
        // 检测误差是否持续增大
        bool increasing_error = true;
        for (size_t i = 1; i < error_history_.size(); ++i) {
            if (fabs(error_history_[i]) <= fabs(error_history_[i-1])) {
                increasing_error = false;
                break;
            }
        }
        
        return increasing_error && fabs(error_history_.back()) > 1.0;
    }
    
    void updateControlHistory(double steering, double yaw_error) {
        // 更新误差历史
        error_history_.push_back(yaw_error);
        if (error_history_.size() > 5) {
            error_history_.pop_front();
        }
        
        // 检测转向方向变化
        int current_direction = (steering > 0.05) ? 1 : ((steering < -0.05) ? -1 : 0);
        if (current_direction != 0 && current_direction != last_steering_direction_) {
            last_steering_direction_ = current_direction;
        }
    }
    
    void resetControlStates() {
        integral_error_ = 0.0;
        previous_error_ = 0.0;
        previous_steering_ = 0.0;
        oscillation_counter_ = 0;
        steering_history_.clear();
        error_history_.clear();
    }
    
    void printStabilityDebug(double yaw_error, double yaw_error_deg, double steering, double target_speed) {
        static int debug_counter = 0;
        debug_counter++;
        
        if (debug_counter % 15 == 0) {
            bool is_oscillating = detectOscillation();
            bool is_unstable = detectInstability();
            
            ROS_INFO("🎯 Stable: %.2f°→S=%.3f | speed=%.1f | osc=%s | unstable=%s | integral=%.3f", 
                     yaw_error * 180.0 / M_PI, steering, current_speed_,
                     is_oscillating ? "YES" : "NO", is_unstable ? "YES" : "NO", integral_error_);
        }
    }
    
    // 工具函数
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
    ros::init(argc, argv, "stable_dwa_executor");
    
    StableDWAExecutor executor;
    executor.spin();
    
    return 0;
}