#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <simulation/VehicleControl.h>
#include <msg_interfaces/Trajectory.h>  // 路径规划模块的轨迹消息
#include <algorithm>
#include <cmath>
#include <vector>
#include <deque>

class VehicleController {
public:
    VehicleController(ros::NodeHandle& nh) : tf_listener_(tf_buffer_) {
        // ========== 订阅者 ==========
        // 订阅路径规划模块的轨迹 (主要输入)
        trajectory_sub_ = nh.subscribe("/planning/trajectory", 1, 
                                      &VehicleController::trajectoryCallback, this);
        
        // 订阅感知层的前方危险信号 - 用于判断何时恢复正常
        front_hazard_sub_ = nh.subscribe("/perception/front_hazard", 1,
                                        &VehicleController::frontHazardCallback, this);
        
        // 可选：订阅紧急制动指令作为备用安全机制
        emergency_stop_sub_ = nh.subscribe("/decision/emergency_stop", 1, 
                                         &VehicleController::emergencyStopCallback, this);
        
        // ========== 发布者 ==========
        cmd_pub_ = nh.advertise<simulation::VehicleControl>("car_command", 1);
        
        ROS_INFO("Publishing vehicle control commands to: %s", cmd_pub_.getTopic().c_str());
        
        // ========== 参数初始化 ==========
        nh.param("max_linear_velocity", max_linear_vel_, 2.0);  // 降低最大速度从5.0到2.0
        nh.param("max_angular_velocity", max_angular_vel_, 1.5);
        nh.param("wheelbase", wheelbase_, 2.7);
        nh.param("lookahead_distance", lookahead_distance_, 2.5);
        nh.param("control_frequency", control_frequency_, 50.0);  // 提高到50Hz
        
        // 转弯控制参数
        nh.param("turn_detection_distance", turn_detection_distance_, 3.0);  // 转弯检测距离
        nh.param("max_turn_speed", max_turn_speed_, 0.5);                    // 转弯最大速度
        nh.param("turn_angle_threshold", turn_angle_threshold_, 0.3);        // 转弯角度阈值(约17度)
        nh.param("straight_speed", straight_speed_, 1.0);                    // 直行速度
        
        // PID控制器参数
        nh.param("kp_linear", kp_linear_, 1.2);
        nh.param("ki_linear", ki_linear_, 0.05);
        nh.param("kd_linear", kd_linear_, 0.1);
        
        nh.param("kp_angular", kp_angular_, 2.5);
        nh.param("ki_angular", ki_angular_, 0.0);
        nh.param("kd_angular", kd_angular_, 0.15);
        
        // 横向控制参数（Pure Pursuit）
        nh.param("pure_pursuit_lookahead_ratio", pp_lookahead_ratio_, 0.3);
        nh.param("min_lookahead_distance", min_lookahead_, 1.5);
        nh.param("max_lookahead_distance", max_lookahead_, 5.0);
        
        // ========== 状态初始化 ==========
        emergency_stop_ = false;
        front_hazard_ = false;  // 前方危险状态
        current_trajectory_index_ = 0;
        has_trajectory_ = false;
        trajectory_updated_ = false;
        is_emergency_trajectory_ = false;
        max_safe_speed_ = 1.0;    // 硬编码安全速度
        emergency_speed_ = 0.0;   // 紧急时目标速度
        
        // 转弯控制参数初始化
        turn_detection_distance_ = 3.0;
        max_turn_speed_ = 0.5;
        turn_angle_threshold_ = 0.3;  // 约17度
        straight_speed_ = 0.8;
        
        // PID状态初始化
        prev_linear_error_ = 0.0;
        prev_angular_error_ = 0.0;
        integral_linear_error_ = 0.0;
        integral_angular_error_ = 0.0;
        last_control_time_ = ros::Time::now();
        
        // 轨迹缓存
        trajectory_buffer_size_ = 50;
        
        // ========== 控制循环定时器 ==========
        control_timer_ = nh.createTimer(ros::Duration(1.0 / control_frequency_), 
                                      &VehicleController::controlLoop, this);
        
        ROS_INFO("Enhanced Vehicle Controller initialized");
        ROS_INFO("Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("Pure Pursuit lookahead: %.2f - %.2f m", min_lookahead_, max_lookahead_);
    }

private:
    // ========== ROS相关 ==========
    ros::Subscriber trajectory_sub_, emergency_stop_sub_, front_hazard_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ========== 控制参数 ==========
    double max_linear_vel_;
    double max_angular_vel_;
    double wheelbase_;
    double lookahead_distance_;
    double control_frequency_;
    
    // 转弯控制参数
    double turn_detection_distance_;  // 转弯检测提前距离
    double max_turn_speed_;           // 转弯时的最大速度
    double turn_angle_threshold_;     // 判定为转弯的角度阈值
    double straight_speed_;           // 直行时的速度
    
    // 安全速度参数
    double max_safe_speed_;   // 新增：强制的最大安全速度
    double emergency_speed_;  // 紧急情况下的目标速度
    
    // PID参数
    double kp_linear_, ki_linear_, kd_linear_;
    double kp_angular_, ki_angular_, kd_angular_;
    
    // Pure Pursuit参数
    double pp_lookahead_ratio_;
    double min_lookahead_, max_lookahead_;
    
    // ========== 状态变量 ==========
    bool emergency_stop_;  // 仅作为备用安全机制
    bool front_hazard_;    // 感知层的前方危险信号
    bool has_trajectory_;
    bool trajectory_updated_;
    bool is_emergency_trajectory_;  // 新增：标识是否为紧急轨迹
    
    // 轨迹相关
    msg_interfaces::Trajectory current_trajectory_;
    size_t current_trajectory_index_;
    size_t trajectory_buffer_size_;
    std::deque<msg_interfaces::Trajectory> trajectory_history_;
    
    // PID控制器状态
    double prev_linear_error_;
    double prev_angular_error_;
    double integral_linear_error_;
    double integral_angular_error_;
    ros::Time last_control_time_;
    
    // 车辆状态缓存
    struct VehicleState {
        double x, y, yaw;
        double linear_vel, angular_vel;
        ros::Time timestamp;
    };
    VehicleState current_state_;

    // ========== 轨迹回调函数 ==========
    void trajectoryCallback(const msg_interfaces::Trajectory::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty trajectory");
            return;
        }
        
        current_trajectory_ = *msg;
        has_trajectory_ = true;
        trajectory_updated_ = true;
        current_trajectory_index_ = 0;  // 重置轨迹索引
        
        // 检测是否为紧急轨迹（基于轨迹长度和速度）
        is_emergency_trajectory_ = isEmergencyTrajectory(*msg);
        
        // 清除积分误差以避免跳跃
        integral_linear_error_ = 0.0;
        integral_angular_error_ = 0.0;
        
        // 添加到历史缓存
        trajectory_history_.push_back(*msg);
        if (trajectory_history_.size() > trajectory_buffer_size_) {
            trajectory_history_.pop_front();
        }
        
        if (is_emergency_trajectory_) {
            ROS_ERROR("Received EMERGENCY trajectory with %lu poses - executing immediate stop!", 
                      msg->poses.size());
        } else {
            ROS_INFO("Received normal trajectory with %lu poses, target speed: %.2f m/s", 
                     msg->poses.size(), 
                     msg->velocities.empty() ? 0.0 : msg->velocities[0]);
        }
    }

    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        bool prev_emergency = emergency_stop_;
        emergency_stop_ = msg->data;
        
        if (prev_emergency != emergency_stop_) {
            if (emergency_stop_) {
                ROS_WARN("Backup emergency stop activated (decision layer)!");
                // 清除积分项避免急停后的控制跳跃
                integral_linear_error_ = 0.0;
                integral_angular_error_ = 0.0;
            } else {
                ROS_INFO("Backup emergency stop deactivated");
            }
        }
    }

    // 新增：前方危险信号回调
    void frontHazardCallback(const std_msgs::Bool::ConstPtr& msg) {
        bool prev_hazard = front_hazard_;
        front_hazard_ = msg->data;
        
        if (prev_hazard != front_hazard_) {
            if (front_hazard_) {
                ROS_WARN("Front hazard detected by perception - expecting avoidance trajectory");
            } else {
                ROS_INFO("Front hazard cleared - should resume normal trajectory soon");
                // 危险清除时，重置紧急轨迹标志，准备接受正常轨迹
                if (is_emergency_trajectory_) {
                    ROS_INFO("Ready to resume normal path following");
                }
            }
        }
    }

    // 改进的紧急轨迹检测函数
    bool isEmergencyTrajectory(const msg_interfaces::Trajectory& trajectory) {
        // 新的判断逻辑：结合感知信号和轨迹特征
        
        // 1. 如果感知层没有检测到危险，很可能不是紧急轨迹
        if (!front_hazard_) {
            // 但如果轨迹突然变得很短且速度很低，可能是紧急制动
            if (trajectory.poses.size() <= 2) {
                if (!trajectory.velocities.empty()) {
                    double avg_speed = 0.0;
                    for (double v : trajectory.velocities) {
                        avg_speed += v;
                    }
                    avg_speed /= trajectory.velocities.size();
                    if (avg_speed < 0.1) {  // 几乎停止
                        return true;
                    }
                }
            }
            return false;  // 感知层没有危险信号，认为是正常轨迹
        }
        
        // 2. 感知层检测到危险时，分析轨迹特征
        if (trajectory.poses.size() <= 3) {
            // 检查速度
            if (!trajectory.velocities.empty()) {
                double avg_speed = 0.0;
                for (double v : trajectory.velocities) {
                    avg_speed += v;
                }
                avg_speed /= trajectory.velocities.size();
                
                if (avg_speed < 0.3) {  // 低速或停止
                    return true;
                }
            }
            
            // 检查轨迹总长度
            if (trajectory.poses.size() >= 2) {
                double total_distance = 0.0;
                for (size_t i = 1; i < trajectory.poses.size(); ++i) {
                    double dx = trajectory.poses[i].position.x - trajectory.poses[i-1].position.x;
                    double dy = trajectory.poses[i].position.y - trajectory.poses[i-1].position.y;
                    total_distance += sqrt(dx*dx + dy*dy);
                }
                
                if (total_distance < 1.5) {  // 总长度很短
                    return true;
                }
            }
        }
        
        return false;
    }

    // ========== 主控制循环 ==========
    void controlLoop(const ros::TimerEvent& event) {
        simulation::VehicleControl cmd;
        
        // 0. 检查是否为紧急模式 - 最高优先级
        if (is_emergency_trajectory_ && has_trajectory_) {
            publishEmergencyTrajectoryControl(cmd);
            return;
        }
        
        // 1. 备用紧急制动（决策层）
        if (emergency_stop_) {
            publishBackupEmergencyStop(cmd);
            return;
        }
        
        // 2. 获取当前车辆状态
        if (!updateVehicleState()) {
            ROS_WARN_THROTTLE(1.0, "Could not get vehicle state, using fallback control");
            publishFallbackControl(cmd);
            return;
        }
        
        // 3. 检查是否有轨迹
        if (!has_trajectory_ || current_trajectory_.poses.empty()) {
            ROS_WARN_THROTTLE(2.0, "No trajectory available, using standby control");
            publishStandbyControl(cmd);
            return;
        }
        
        // 4. 执行轨迹跟踪控制
        if (!computeTrajectoryFollowingControl(cmd)) {
            ROS_WARN("Trajectory following failed, using fallback control");
            publishFallbackControl(cmd);
            return;
        }
        
        // 5. 发布控制指令
        cmd_pub_.publish(cmd);
        
        // 6. 调试输出：显示实际发布的控制指令
        ROS_DEBUG("Published control: T=%.3f B=%.3f S=%.3f", 
                 cmd.Throttle, cmd.Brake, cmd.Steering);
        
        // 7. 重置轨迹更新标志
        trajectory_updated_ = false;
    }

    // ========== 车辆状态更新 ==========
    bool updateVehicleState() {
        geometry_msgs::TransformStamped transform;
        
        // 尝试多个可能的坐标系
        std::vector<std::string> possible_frames = {
            "base_link", "OurCar/Sensors/INS", "vehicle"
        };
        
        bool transform_found = false;
        for (const auto& frame : possible_frames) {
            try {
                transform = tf_buffer_.lookupTransform("map", frame, 
                                                     ros::Time(0), ros::Duration(0.1));
                transform_found = true;
                break;
            } catch (tf2::TransformException& ex) {
                continue;
            }
        }
        
        if (!transform_found) {
            return false;
        }
        
        // 更新位置和朝向
        current_state_.x = transform.transform.translation.x;
        current_state_.y = transform.transform.translation.y;
        
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_state_.yaw);
        
        current_state_.timestamp = ros::Time::now();
        
        return true;
    }

    // ========== 轨迹跟踪控制 ==========
    bool computeTrajectoryFollowingControl(simulation::VehicleControl& cmd) {
        // 1. 找到最佳跟踪点
        size_t target_index = findBestTrajectoryPoint();
        
        if (target_index >= current_trajectory_.poses.size()) {
            // 到达轨迹终点
            ROS_INFO_THROTTLE(1.0, "Reached end of trajectory");
            publishTrajectoryEndControl(cmd);
            return true;
        }
        
        // 2. 获取目标状态
        const auto& target_pose = current_trajectory_.poses[target_index];
        double target_speed = 0.0;
        
        if (target_index < current_trajectory_.velocities.size()) {
            target_speed = current_trajectory_.velocities[target_index];
        }
        
        // 3. 智能速度调节：根据轨迹几何特征调整速度
        double adjusted_speed = calculateAdaptiveSpeed(target_index, target_speed);
        
        // 4. 强制限制速度！无论路径规划要求什么速度
        if (is_emergency_trajectory_) {
            adjusted_speed = emergency_speed_;  // 紧急情况：0速度
            ROS_ERROR_THROTTLE(0.5, "EMERGENCY trajectory - forced speed: %.2f m/s", adjusted_speed);
        } else {
            // 正常情况：限制在安全速度范围内
            adjusted_speed = std::min(adjusted_speed, max_safe_speed_);
            adjusted_speed = std::max(adjusted_speed, 0.0);  // 不允许负速度
            
            // 检测转弯并进一步限制速度
            bool is_turning = detectUpcomingTurn(target_index);
            if (is_turning) {
                adjusted_speed = std::min(adjusted_speed, max_turn_speed_);
                ROS_WARN_THROTTLE(0.5, "TURN detected - reduced speed: %.2f m/s", adjusted_speed);
            }
            
            ROS_INFO_THROTTLE(1.0, "Speed control - original: %.2f, adjusted: %.2f, turning: %s", 
                             target_speed, adjusted_speed, is_turning ? "YES" : "NO");
        }
        
        // 5. 计算控制指令
        return computePurePursuitControl(target_pose, adjusted_speed, cmd);
    }
    
    // ========== 新增：自适应速度计算 ==========
    double calculateAdaptiveSpeed(size_t current_index, double original_speed) {
        if (current_trajectory_.poses.size() < 3) {
            return original_speed;
        }
        
        // 分析前方路径的曲率
        double max_curvature = 0.0;
        size_t look_ahead_points = std::min(size_t(5), current_trajectory_.poses.size() - current_index);
        
        for (size_t i = current_index; i < current_index + look_ahead_points - 2; ++i) {
            double curvature = calculatePathCurvature(i);
            max_curvature = std::max(max_curvature, std::abs(curvature));
        }
        
        // 根据曲率调整速度
        if (max_curvature > 0.5) {
            return max_turn_speed_;  // 急转弯
        } else if (max_curvature > 0.2) {
            return (max_turn_speed_ + straight_speed_) * 0.5;  // 中等转弯
        } else {
            return straight_speed_;  // 直行
        }
    }
    
    // ========== 新增：转弯检测 ==========
    bool detectUpcomingTurn(size_t current_index) {
        if (current_trajectory_.poses.size() < 3 || 
            current_index + 3 >= current_trajectory_.poses.size()) {
            return false;
        }
        
        // 检查前方几个点的角度变化
        for (size_t i = current_index; i < current_index + 3 && i + 1 < current_trajectory_.poses.size(); ++i) {
            double angle_change = calculateAngleChange(i);
            if (std::abs(angle_change) > turn_angle_threshold_) {
                return true;
            }
        }
        
        return false;
    }
    
    // ========== 新增：计算路径曲率 ==========
    double calculatePathCurvature(size_t index) {
        if (index + 2 >= current_trajectory_.poses.size()) {
            return 0.0;
        }
        
        // 使用三点计算曲率
        const auto& p1 = current_trajectory_.poses[index].position;
        const auto& p2 = current_trajectory_.poses[index + 1].position;
        const auto& p3 = current_trajectory_.poses[index + 2].position;
        
        // 计算向量
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;
        
        // 计算角度变化
        double angle1 = atan2(dy1, dx1);
        double angle2 = atan2(dy2, dx2);
        double angle_diff = normalizeAngle(angle2 - angle1);
        
        // 计算距离
        double dist = sqrt(dx1*dx1 + dy1*dy1) + sqrt(dx2*dx2 + dy2*dy2);
        
        if (dist < 0.01) return 0.0;
        
        // 曲率 = 角度变化 / 弧长
        return std::abs(angle_diff) / dist;
    }
    
    // ========== 新增：计算角度变化 ==========
    double calculateAngleChange(size_t index) {
        if (index + 1 >= current_trajectory_.poses.size()) {
            return 0.0;
        }
        
        const auto& p1 = current_trajectory_.poses[index].position;
        const auto& p2 = current_trajectory_.poses[index + 1].position;
        
        // 当前车辆朝向
        double current_yaw = current_state_.yaw;
        
        // 目标朝向
        double target_yaw = atan2(p2.y - p1.y, p2.x - p1.x);
        
        return normalizeAngle(target_yaw - current_yaw);
    }

    // ========== Pure Pursuit控制算法 ==========
    bool computePurePursuitControl(const geometry_msgs::Pose& target_pose, 
                                   double target_speed,
                                   simulation::VehicleControl& cmd) {
        
        // 计算到目标点的距离和角度
        double dx = target_pose.position.x - current_state_.x;
        double dy = target_pose.position.y - current_state_.y;
        double distance_to_target = sqrt(dx*dx + dy*dy);
        
        // 计算目标角度
        double target_angle = atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - current_state_.yaw);
        
        // 动态调整前瞻距离
        double dynamic_lookahead = std::max(min_lookahead_, 
                                           std::min(max_lookahead_, 
                                                   target_speed * pp_lookahead_ratio_));
        
        // ========== 纵向控制（速度） ==========
        double speed_error = target_speed - current_state_.linear_vel;
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time_).toSec();
        if (dt <= 0.0 || dt > 0.2) dt = 1.0 / control_frequency_;
        
        // PID速度控制
        integral_linear_error_ += speed_error * dt;
        // 限制积分饱和
        integral_linear_error_ = std::max(-2.0, std::min(2.0, integral_linear_error_));
        
        double derivative_linear = (speed_error - prev_linear_error_) / dt;
        double speed_output = kp_linear_ * speed_error + 
                             ki_linear_ * integral_linear_error_ + 
                             kd_linear_ * derivative_linear;
        
        // 油门/制动分配 - 智能速度控制
        if (is_emergency_trajectory_ || target_speed <= 0.1) {
            // 紧急情况：强制制动
            cmd.Throttle = 0.0f;
            cmd.Brake = 1.0f;
            cmd.Steering = 0.0f;
            
            ROS_ERROR_THROTTLE(0.3, "EMERGENCY STOP: T=%.2f B=%.2f", cmd.Throttle, cmd.Brake);
        } else {
            // 正常行驶：基于速度误差和转弯状态的精细控制
            double current_speed = current_state_.linear_vel;
            double speed_error = target_speed - current_speed;
            
            // 检测当前是否在转弯
            bool currently_turning = std::abs(angle_error) > turn_angle_threshold_;
            
            if (currently_turning) {
                // 转弯中：优先减速，谨慎加速
                if (current_speed > max_turn_speed_) {
                    // 当前速度过快，强制减速
                    cmd.Throttle = 0.0f;
                    cmd.Brake = 0.8f;
                    ROS_WARN_THROTTLE(0.5, "TURNING - Speed too high: %.2f > %.2f, braking", 
                                     current_speed, max_turn_speed_);
                } else if (speed_error > 0.1) {
                    // 需要轻微加速
                    cmd.Throttle = std::min(0.2f, static_cast<float>(speed_error * 0.3));
                    cmd.Brake = 0.0f;
                } else {
                    // 维持转弯速度
                    cmd.Throttle = 0.05f;  // 很小的维持油门
                    cmd.Brake = 0.0f;
                }
            } else {
                // 直行中：可以相对积极地控制速度
                if (speed_error > 0.2) {
                    // 需要加速
                    cmd.Throttle = std::min(0.4f, static_cast<float>(speed_error * 0.6));
                    cmd.Brake = 0.0f;
                } else if (speed_error < -0.2) {
                    // 需要减速
                    cmd.Throttle = 0.0f;
                    cmd.Brake = std::min(0.6f, static_cast<float>(-speed_error * 0.8));
                } else {
                    // 速度合适，温和维持
                    cmd.Throttle = 0.15f;  // 维持油门
                    cmd.Brake = 0.0f;
                }
            }
            
            ROS_DEBUG_THROTTLE(1.0, "Speed control: target=%.2f, current=%.2f, turning=%s, T=%.2f, B=%.2f", 
                              target_speed, current_speed, currently_turning ? "YES" : "NO", 
                              cmd.Throttle, cmd.Brake);
        }
        
        // 平滑油门控制
        cmd.Throttle = std::max(0.0f, std::min(1.0f, cmd.Throttle));
        cmd.Brake = std::max(0.0f, std::min(1.0f, cmd.Brake));
        
        // ========== 横向控制（转向） - 精确路径跟踪 ==========
        double steering_angle = 0.0;
        
        if (distance_to_target > 0.1) {  // 避免除零
            // 改进的Pure Pursuit算法 - 减少过度转向
            double curvature = 2.0 * sin(angle_error) / distance_to_target;
            
            // 根据距离调整转向强度 - 距离越近，转向越温和
            double distance_factor = std::min(1.0, distance_to_target / 2.0);
            curvature *= distance_factor;
            
            steering_angle = atan2(wheelbase_ * curvature, 1.0);
            
            // 根据速度和轨迹类型调整转向灵敏度
            double speed_factor = 1.0;
            if (current_state_.linear_vel > max_turn_speed_) {
                // 高速时大幅减少转向幅度
                speed_factor = max_turn_speed_ / (current_state_.linear_vel + 0.1);
                speed_factor = std::min(speed_factor, 0.5);  // 最多减少50%
            }
            
            // 如果是紧急轨迹，进一步减少转向（专注于制动）
            if (is_emergency_trajectory_) {
                speed_factor *= 0.3;  // 紧急情况下转向很小
            }
            
            steering_angle *= speed_factor;
            
            // 更保守的转向角度限制
            double max_steering = M_PI / 8;  // 22.5度，比之前更保守
            if (std::abs(angle_error) > turn_angle_threshold_) {
                max_steering = M_PI / 12;  // 转弯时限制到15度
            }
            
            // 对于紧急轨迹，几乎不转向
            if (is_emergency_trajectory_) {
                max_steering = M_PI / 24;  // 7.5度，主要靠制动
            }
            
            steering_angle = std::max(-max_steering, std::min(max_steering, steering_angle));
            
            // 反向转向输出（修复转向方向）
            cmd.Steering = -static_cast<float>(steering_angle / (M_PI / 8));
            
            // 调试输出
            ROS_DEBUG_THROTTLE(1.0, "Steering: angle_err=%.1f°, dist=%.2f, emergency=%s, cmd=%.3f", 
                              angle_error * 180.0 / M_PI, distance_to_target,
                              is_emergency_trajectory_ ? "YES" : "NO", cmd.Steering);
            
        } else {
            cmd.Steering = 0.0f;
        }
        
        // 转向平滑限制
        cmd.Steering = std::max(-1.0f, std::min(1.0f, cmd.Steering));
        
        // 转向过大的警告
        if (std::abs(cmd.Steering) > 0.7f && !is_emergency_trajectory_) {
            ROS_WARN_THROTTLE(1.0, "Large steering: %.2f, consider path replanning", cmd.Steering);
        }
        
        cmd.Reserved = 0.0f;
        
        // 更新控制器状态
        prev_linear_error_ = speed_error;
        prev_angular_error_ = angle_error;
        last_control_time_ = current_time;
        current_trajectory_index_ = findBestTrajectoryPoint();
        
        ROS_DEBUG("Control: T=%.2f B=%.2f S=%.2f | Speed: %.2f->%.2f | Dist: %.2f | Angle: %.2f°", 
                  cmd.Throttle, cmd.Brake, cmd.Steering, 
                  current_state_.linear_vel, target_speed,
                  distance_to_target, angle_error * 180.0 / M_PI);
        
        return true;
    }

    // ========== 寻找最佳轨迹跟踪点 ==========
    size_t findBestTrajectoryPoint() {
        if (current_trajectory_.poses.empty()) {
            return 0;
        }
        
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = current_trajectory_index_;
        
        // 从当前索引开始搜索，避免倒退
        size_t search_start = current_trajectory_index_;
        size_t search_end = std::min(search_start + 10, current_trajectory_.poses.size());
        
        for (size_t i = search_start; i < search_end; ++i) {
            double dx = current_trajectory_.poses[i].position.x - current_state_.x;
            double dy = current_trajectory_.poses[i].position.y - current_state_.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        // 前瞻控制：选择前方合适距离的点
        double dynamic_lookahead = std::max(min_lookahead_, 
                                           std::min(max_lookahead_, 
                                                   current_state_.linear_vel * pp_lookahead_ratio_ + 1.0));
        
        for (size_t i = closest_index; i < current_trajectory_.poses.size(); ++i) {
            double dx = current_trajectory_.poses[i].position.x - current_state_.x;
            double dy = current_trajectory_.poses[i].position.y - current_state_.y;
            double distance = sqrt(dx*dx + dy*dy);
            
            if (distance >= dynamic_lookahead) {
                return i;
            }
        }
        
        // 如果没有找到合适的前瞻点，使用最后一个点
        return std::min(closest_index + 1, current_trajectory_.poses.size() - 1);
    }

    // ========== 特殊控制模式 ==========
    void publishEmergencyTrajectoryControl(simulation::VehicleControl& cmd) {
        // 专用的紧急轨迹控制 - 简单但有效的制动
        cmd.Throttle = 0.0f;   // 完全停止油门
        cmd.Brake = 1.0f;      // 最大制动
        cmd.Steering = 0.0f;   // 直行制动
        cmd.Reserved = 0.0f;
        
        cmd_pub_.publish(cmd);
        
        ROS_ERROR_THROTTLE(0.1, "EMERGENCY: Pure Braking - T=%.3f B=%.3f", 
                          cmd.Throttle, cmd.Brake);
    }
    
    void publishBackupEmergencyStop(simulation::VehicleControl& cmd) {
        cmd.Throttle = 0.0f;   // 停止油门
        cmd.Brake = 1.0f;      // 最大制动力
        cmd.Steering = 0.0f;
        cmd.Reserved = 0.0f;
        cmd_pub_.publish(cmd);
        
        ROS_WARN_THROTTLE(1.0, "BACKUP EMERGENCY: Pure Braking");
    }
    
    void publishStandbyControl(simulation::VehicleControl& cmd) {
        cmd.Throttle = 0.0f;
        cmd.Brake = 0.3f;  // 轻微制动保持停止
        cmd.Steering = 0.0f;
        cmd.Reserved = 0.0f;
        cmd_pub_.publish(cmd);
    }
    
    void publishTrajectoryEndControl(simulation::VehicleControl& cmd) {
        cmd.Throttle = 0.0f;
        cmd.Brake = 0.5f;  // 到达终点后制动
        cmd.Steering = 0.0f;
        cmd.Reserved = 0.0f;
        cmd_pub_.publish(cmd);
    }
    
    void publishFallbackControl(simulation::VehicleControl& cmd) {
        // 简单的前进控制作为备用
        cmd.Throttle = 0.2f;
        cmd.Brake = 0.0f;
        cmd.Steering = 0.0f;
        cmd.Reserved = 0.0f;
        cmd_pub_.publish(cmd);
        
        ROS_WARN_THROTTLE(2.0, "Using fallback control");
    }

    // ========== 工具函数 ==========
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_vehicle_controller");
    ros::NodeHandle nh;
    
    VehicleController controller(nh);
    
    ROS_INFO("Enhanced Vehicle Controller with trajectory following started!");
    ros::spin();
    
    return 0;
}