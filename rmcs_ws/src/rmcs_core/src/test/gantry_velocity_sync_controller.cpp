#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Core>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::testing {

/**
 * @brief 龙门架的“共同速度 + 位置差速度修正”控制器。
 *
 * 不建立 Pitch 的绝对目标角度。右摇杆直接给两侧共同速度，左右编码器的相对
 * 位置差只产生等量反向的速度修正，再由两个通用速度 PID 输出电机力矩。
 */
class GantryVelocitySyncController
    : public rmcs_executor::Component
    , public rclcpp::Node {
private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    enum class State { kHoming, kRunning, kFault };

public:
    GantryVelocitySyncController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        get_parameter_or("auto_home", auto_home_, false);
        get_parameter_or(
            "lock_manual_during_homing", lock_manual_during_homing_, true);
        get_parameter_or("homing_velocity", homing_velocity_, 0.0);
        get_parameter_or("homing_stall_time_s", homing_stall_time_s_, 0.3);
        get_parameter_or("homing_timeout_s", homing_timeout_s_, 5.0);
        get_parameter_or("stall_velocity_threshold", stall_velocity_threshold_, 0.05);
        get_parameter_or("manual_velocity_scale", manual_velocity_scale_, 15.0);
        get_parameter_or("max_velocity", max_velocity_, 15.0);
        double sync_kp = 2.0;
        double sync_velocity_limit = 5.0;
        get_parameter_or("sync_kp", sync_kp, sync_kp);
        get_parameter_or("sync_velocity_limit", sync_velocity_limit, sync_velocity_limit);
        sync_kp_.store(sync_kp, std::memory_order_relaxed);
        sync_velocity_limit_.store(sync_velocity_limit, std::memory_order_relaxed);

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/draft/gantry/left_motor/angle", left_angle_);
        register_input("/draft/gantry/right_motor/angle", right_angle_);
        register_input("/draft/gantry/left_motor/velocity", left_velocity_);
        register_input("/draft/gantry/right_motor/velocity", right_velocity_);
        register_input("/draft/gantry/left_motor/feedback_ready", left_feedback_ready_);
        register_input("/draft/gantry/right_motor/feedback_ready", right_feedback_ready_);

        register_output(
            "/draft/gantry/left_motor/control_velocity", left_control_velocity_, 0.0);
        register_output(
            "/draft/gantry/right_motor/control_velocity", right_control_velocity_, 0.0);
        register_output("/draft/gantry/position_difference", position_difference_, nan());
        register_output("/draft/gantry/average_position", average_position_, nan());
        register_output("/draft/gantry/common_velocity", common_velocity_output_, 0.0);
        register_output("/draft/gantry/sync_velocity_correction", sync_correction_output_, 0.0);
        register_output("/draft/gantry/homing", homing_, false);
        // 回零期间锁住其他人工通道，避免遥控器与自动回零同时写控制目标。
        register_output("/draft/control_locked", control_locked_, false);
        register_output("/draft/gantry/fault", fault_, false);

        if (homing_stall_time_s_ < 0.0 || homing_timeout_s_ <= 0.0
            || stall_velocity_threshold_ < 0.0 || manual_velocity_scale_ < 0.0
            || max_velocity_ <= 0.0 || sync_kp < 0.0 || sync_velocity_limit < 0.0) {
            enter_fault("invalid gantry velocity-sync parameters");
        }
        if (auto_home_ && std::abs(homing_velocity_) <= std::numeric_limits<double>::epsilon())
            enter_fault("auto_home is enabled but homing_velocity is zero");
        if (state_ != State::kFault && auto_home_)
            state_ = State::kHoming;

        // 同步增益用于现场辨识，允许在节点运行时修改；不会重建相对零点。
        parameter_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                double new_sync_kp = sync_kp_.load(std::memory_order_relaxed);
                double new_sync_velocity_limit =
                    sync_velocity_limit_.load(std::memory_order_relaxed);
                bool changed = false;
                for (const auto& parameter : parameters) {
                    if (parameter.get_name() != "sync_kp"
                        && parameter.get_name() != "sync_velocity_limit")
                        continue;
                    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE
                        || !std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0) {
                        result.successful = false;
                        result.reason = "sync_kp and sync_velocity_limit must be finite and >= 0";
                        return result;
                    }
                    if (parameter.get_name() == "sync_kp")
                        new_sync_kp = parameter.as_double();
                    else
                        new_sync_velocity_limit = parameter.as_double();
                    changed = true;
                }
                if (changed) {
                    sync_kp_.store(new_sync_kp, std::memory_order_relaxed);
                    sync_velocity_limit_.store(new_sync_velocity_limit, std::memory_order_relaxed);
                    RCLCPP_INFO(
                        get_logger(), "Updated Pitch sync: kp=%.3f s^-1, correction_limit=%.3f rad/s",
                        new_sync_kp, new_sync_velocity_limit);
                }
                return result;
            });
    }

    void update() override {
        if (!inputs_ready() || !switches_enable_motion()) {
            stop();
            return;
        }
        if (state_ == State::kFault) {
            stop();
            return;
        }
        if (state_ == State::kHoming) {
            update_homing(Clock::now());
            return;
        }
        update_running();
    }

private:
    static double nan() { return std::numeric_limits<double>::quiet_NaN(); }

    bool inputs_ready() const {
        return joystick_right_.ready() && switch_left_.ready() && switch_right_.ready()
            && left_angle_.ready() && right_angle_.ready() && left_velocity_.ready()
            && right_velocity_.ready() && left_feedback_ready_.ready()
            && right_feedback_ready_.ready() && *left_feedback_ready_ && *right_feedback_ready_
            && std::isfinite(*left_angle_) && std::isfinite(*right_angle_)
            && std::isfinite(*left_velocity_) && std::isfinite(*right_velocity_);
    }

    bool switches_enable_motion() const {
        using Switch = rmcs_msgs::Switch;
        return *switch_left_ != Switch::UNKNOWN && *switch_right_ != Switch::UNKNOWN
            && !(*switch_left_ == Switch::DOWN && *switch_right_ == Switch::DOWN);
    }

    void initialize_reference() {
        if (reference_initialized_)
            return;
        left_zero_ = *left_angle_;
        right_zero_ = *right_angle_;
        reference_initialized_ = true;
    }

    void update_position_observation() {
        const double left_position = *left_angle_ - left_zero_;
        const double right_position = *right_angle_ - right_zero_;
        *position_difference_ = left_position - right_position;
        *average_position_ = 0.5 * (left_position + right_position);
    }

    void update_running() {
        initialize_reference();
        update_position_observation();

        // 与 yaw 的 left.y 约定一致：DR16 物理横向通道是 right.y。
        const double common_velocity = std::clamp(
            joystick_right_->y() * manual_velocity_scale_, -max_velocity_, max_velocity_);
        // e = q_left - q_right。e > 0 时，左侧领先；令左减速、右加速，
        // 则 e_dot = -2 * sync_kp * e，误差会回到零。
        const double sync_kp = sync_kp_.load(std::memory_order_relaxed);
        const double sync_velocity_limit =
            sync_velocity_limit_.load(std::memory_order_relaxed);
        const double correction = std::clamp(
            sync_kp * *position_difference_, -sync_velocity_limit, sync_velocity_limit);

        *left_control_velocity_ = std::clamp(
            common_velocity - correction, -max_velocity_, max_velocity_);
        *right_control_velocity_ = std::clamp(
            common_velocity + correction, -max_velocity_, max_velocity_);
        *common_velocity_output_ = common_velocity;
        *sync_correction_output_ = correction;
        *homing_ = false;
        *control_locked_ = false;
        *fault_ = false;
    }

    void update_homing(const TimePoint now) {
        if (!homing_started_at_.has_value())
            homing_started_at_ = now;
        if (std::chrono::duration<double>(now - *homing_started_at_).count()
            > homing_timeout_s_) {
            enter_fault("homing timeout");
            return;
        }

        update_stall_timer(*left_velocity_, left_stall_started_at_, now, left_homed_);
        update_stall_timer(*right_velocity_, right_stall_started_at_, now, right_homed_);
        *left_control_velocity_ = left_homed_ ? 0.0 : homing_velocity_;
        *right_control_velocity_ = right_homed_ ? 0.0 : homing_velocity_;
        *common_velocity_output_ = homing_velocity_;
        *sync_correction_output_ = 0.0;
        *homing_ = true;
        *control_locked_ = lock_manual_during_homing_;
        *fault_ = false;

        if (!left_homed_ || !right_homed_)
            return;

        left_zero_ = *left_angle_;
        right_zero_ = *right_angle_;
        reference_initialized_ = true;
        state_ = State::kRunning;
        homing_started_at_.reset();
        update_position_observation();
        stop();
        RCLCPP_INFO(get_logger(), "Gantry homing completed");
    }

    void update_stall_timer(
        double velocity, std::optional<TimePoint>& started_at, const TimePoint now,
        bool& homed) const {
        if (homed)
            return;
        if (std::abs(velocity) > stall_velocity_threshold_) {
            started_at.reset();
            return;
        }
        if (!started_at.has_value())
            started_at = now;
        if (std::chrono::duration<double>(now - *started_at).count() >= homing_stall_time_s_)
            homed = true;
    }

    void enter_fault(const char* reason) {
        state_ = State::kFault;
        *fault_ = true;
        RCLCPP_ERROR(get_logger(), "Gantry velocity-sync fault: %s", reason);
    }

    void stop() {
        *left_control_velocity_ = 0.0;
        *right_control_velocity_ = 0.0;
        *common_velocity_output_ = 0.0;
        *sync_correction_output_ = 0.0;
        *homing_ = state_ == State::kHoming;
        *control_locked_ = state_ == State::kHoming && lock_manual_during_homing_;
        *fault_ = state_ == State::kFault;
    }

    bool auto_home_ = false;
    bool lock_manual_during_homing_ = true;
    double homing_velocity_ = 0.0;
    double homing_stall_time_s_ = 0.3;
    double homing_timeout_s_ = 5.0;
    double stall_velocity_threshold_ = 0.05;
    double manual_velocity_scale_ = 15.0;
    double max_velocity_ = 15.0;
    std::atomic<double> sync_kp_{2.0};
    std::atomic<double> sync_velocity_limit_{5.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    State state_ = State::kRunning;
    bool reference_initialized_ = false;
    bool left_homed_ = false;
    bool right_homed_ = false;
    double left_zero_ = 0.0;
    double right_zero_ = 0.0;
    std::optional<TimePoint> homing_started_at_;
    std::optional<TimePoint> left_stall_started_at_;
    std::optional<TimePoint> right_stall_started_at_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<double> left_velocity_;
    InputInterface<double> right_velocity_;
    InputInterface<bool> left_feedback_ready_;
    InputInterface<bool> right_feedback_ready_;

    OutputInterface<double> left_control_velocity_;
    OutputInterface<double> right_control_velocity_;
    OutputInterface<double> position_difference_;
    OutputInterface<double> average_position_;
    OutputInterface<double> common_velocity_output_;
    OutputInterface<double> sync_correction_output_;
    OutputInterface<bool> homing_;
    OutputInterface<bool> control_locked_;
    OutputInterface<bool> fault_;
};

/** @brief 在同步速度模式下直接把纠偏量和两侧 PID 状态打印到终端。 */
class GantryVelocitySyncLogger
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryVelocitySyncLogger()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        get_parameter_or("log_period_ms", log_period_ms_, 100);
        if (log_period_ms_ <= 0)
            log_period_ms_ = 100;
        register_input("/draft/safety_enabled", safety_enabled_);
        register_input("/draft/control_locked", control_locked_);
        register_input("/draft/gantry/common_velocity", common_velocity_);
        register_input("/draft/gantry/sync_velocity_correction", correction_);
        register_input("/draft/gantry/position_difference", position_difference_);
        register_input("/draft/gantry/left_motor/control_velocity", left_control_velocity_);
        register_input("/draft/gantry/right_motor/control_velocity", right_control_velocity_);
        register_input("/draft/gantry/left_motor/velocity", left_velocity_);
        register_input("/draft/gantry/right_motor/velocity", right_velocity_);
        register_input("/draft/gantry/left_motor/control_torque", left_control_torque_);
        register_input("/draft/gantry/right_motor/control_torque", right_control_torque_);
    }

    void update() override {
        if (!ready())
            return;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), log_period_ms_,
            "Pitch sync safety=%d locked=%d common_v=%.3f correction=%.3f e(L-R)=%.3f | "
            "L(v_ref=%.3f v=%.3f tau_cmd=%.3f) | "
            "R(v_ref=%.3f v=%.3f tau_cmd=%.3f)",
            static_cast<int>(*safety_enabled_), static_cast<int>(*control_locked_),
            *common_velocity_, *correction_,
            *position_difference_, *left_control_velocity_, *left_velocity_,
            *left_control_torque_, *right_control_velocity_, *right_velocity_,
            *right_control_torque_);
    }

private:
    bool ready() const {
        return safety_enabled_.ready() && control_locked_.ready() && common_velocity_.ready()
            && correction_.ready()
            && position_difference_.ready() && left_control_velocity_.ready()
            && right_control_velocity_.ready() && left_velocity_.ready() && right_velocity_.ready()
            && left_control_torque_.ready() && right_control_torque_.ready();
    }

    InputInterface<bool> safety_enabled_;
    InputInterface<bool> control_locked_;
    InputInterface<double> common_velocity_;
    InputInterface<double> correction_;
    InputInterface<double> position_difference_;
    InputInterface<double> left_control_velocity_;
    InputInterface<double> right_control_velocity_;
    InputInterface<double> left_velocity_;
    InputInterface<double> right_velocity_;
    InputInterface<double> left_control_torque_;
    InputInterface<double> right_control_torque_;
    int log_period_ms_ = 100;
};

} // namespace rmcs_core::controller::testing

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::testing::GantryVelocitySyncController, rmcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::testing::GantryVelocitySyncLogger, rmcs_executor::Component)
