#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gantry {

/**
 * @brief 将 DR16 左摇杆的实际横向通道转换为飞镖镖架 yaw 的速度目标。
 *
 * 数据流：
 *
 *   RemoteControl -> /remote/joystick/left.y
 *                 -> DraftYawController
 *                 -> /draft/yaw/motor/control_velocity
 *                 -> 通用 PidController
 *                 -> /draft/yaw/motor/control_torque
 *
 * 该组件只产生速度目标，不直接发送 CAN 命令。最终的双下、遥控器超时和 CAN
 * 反馈超时互锁仍由 Draft 硬件层在发送前执行，因此本组件失效或 PID 残留输出时
 * 也不会绕过硬件安全闸门。
 */
class DraftYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DraftYawController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        get_parameter_or("manual_velocity_scale", manual_velocity_scale_, 0.5);
        get_parameter_or("max_velocity", max_velocity_, 0.5);
        get_parameter_or("fixed_velocity_enabled", fixed_velocity_enabled_, false);
        get_parameter_or("fixed_velocity", fixed_velocity_, 0.0);

        // RemoteControl 由 Draft 创建并更新。实测本遥控器的左摇杆横向量
        // 被解析到 Vector2d 的 y 分量；不要在这里擅自改用 x 分量。
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/draft/control_locked", control_locked_);
        register_output("/draft/yaw/motor/control_velocity", control_velocity_, 0.0);

        if (manual_velocity_scale_ < 0.0 || max_velocity_ <= 0.0
            || !std::isfinite(fixed_velocity_))
            valid_parameters_ = false;
    }

    void update() override {
        if (!valid_parameters_ || !switch_left_.ready() || !switch_right_.ready()
            || !control_locked_.ready() || (!fixed_velocity_enabled_ && !joystick_left_.ready())
            || !switches_enable_motion() || *control_locked_) {
            *control_velocity_ = 0.0;
            return;
        }

        // 固定模式用于速度环辨识：曲线上的 control_velocity 与 PID 实际 setpoint
        // 是同一个信号。关闭固定模式后，正 x -> 正 yaw；已实测正 yaw 是面向
        // 发射方向向左。
        const double requested_velocity = fixed_velocity_enabled_
            ? fixed_velocity_
            // 保留实测的正负号。若机械运动方向与操作者习惯相反，
            // 应在完成该方向实测后再仅在这一行前添加负号。
            : joystick_left_->y() * manual_velocity_scale_;
        *control_velocity_ = std::clamp(
            requested_velocity, -max_velocity_, max_velocity_);
    }

private:
    bool switches_enable_motion() const {
        using Switch = rmcs_msgs::Switch;
        const auto left = *switch_left_;
        const auto right = *switch_right_;
        return left != Switch::UNKNOWN && right != Switch::UNKNOWN
            && !(left == Switch::DOWN && right == Switch::DOWN);
    }

    double manual_velocity_scale_ = 0.5;
    double max_velocity_ = 0.5;
    bool fixed_velocity_enabled_ = false;
    double fixed_velocity_ = 0.0;
    bool valid_parameters_ = true;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<bool> control_locked_;
    OutputInterface<double> control_velocity_;
};

} // namespace rmcs_core::controller::gantry

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gantry::DraftYawController, rmcs_executor::Component)
