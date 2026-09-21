#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include "librmcs/board/c_board.hpp"

namespace rmcs_core::hardware {

/**
 * @brief 飞镖镖架的硬件通信与独立安全互锁。
 *
 * CAN1 上的三个 M2006（C610 电调）固定使用唯一 ID：yaw=1、pitch_right=2、pitch_left=3。
 * Pitch 的回零、左右同步和三轴的闭环控制属于控制层；本组件只发布电机反馈、
 * 接收 DR16，并在以下任一情况向全部三个电机发送零力矩：
 *
 * - DR16 从未收到或超过 remote_timeout_ms 未更新；
 * - DR16 两个拨杆均在 DOWN；
 * - 任一电机从未收到反馈或反馈超过 feedback_timeout_ms 未更新；
 * - 电机 ID 配置无效或重复。
 */
class Draft
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
private:
    // =====================================================================
    // 硬件拓扑与命令组件
    // =====================================================================

    using Clock = std::chrono::steady_clock;

    enum class MotorIndex : std::size_t {
        kYaw = 0,
        kPitchRight = 1,
        kPitchLeft = 2,
    };

    class MotorCommand final : public rmcs_executor::Component {
    public:
        explicit MotorCommand(Draft& draft)
            : draft_(draft) {}

        void update() override { draft_.send_motor_command(); }

    private:
        Draft& draft_;
    };

    // command partner 由 Draft 自动创建，不需要放入 YAML。状态组件先执行 update()
    // 解析反馈；命令组件随后执行 send_motor_command()，从控制器读取最新力矩。
    std::shared_ptr<MotorCommand> command_component_{
        create_partner_component<MotorCommand>(get_component_name() + "_command", *this)};

    device::DjiMotor yaw_motor_{*this, *command_component_, "/draft/yaw/motor"};
    device::DjiMotor pitch_right_motor_{
        *this, *command_component_, "/draft/gantry/right_motor"};
    device::DjiMotor pitch_left_motor_{
        *this, *command_component_, "/draft/gantry/left_motor"};

    device::Dr16 dr16_;
    std::unique_ptr<device::RemoteControl> remote_control_;
    std::unique_ptr<librmcs::board::CBoard> board_;

    // 回调线程写入、组件 update/command 线程读取。不能仅记录“曾收到反馈”：
    // 电机掉线后旧反馈仍会留在内存中，必须按时间戳判定为失效。
    std::array<std::atomic_int64_t, 3> motor_feedback_received_at_ns_{};
    std::atomic_int64_t dr16_received_at_ns_{0};
    std::array<bool, 3> motor_configuration_valid_{};

    std::chrono::milliseconds remote_timeout_{500};
    std::chrono::milliseconds feedback_timeout_{100};

    OutputInterface<bool> yaw_feedback_ready_output_;
    OutputInterface<bool> pitch_right_feedback_ready_output_;
    OutputInterface<bool> pitch_left_feedback_ready_output_;
    OutputInterface<bool> safety_enabled_output_;

public:
    // =====================================================================
    // 构造与初始化
    // =====================================================================

    Draft()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        register_output("/draft/yaw/motor/feedback_ready", yaw_feedback_ready_output_, false);
        register_output(
            "/draft/gantry/right_motor/feedback_ready", pitch_right_feedback_ready_output_, false);
        register_output(
            "/draft/gantry/left_motor/feedback_ready", pitch_left_feedback_ready_output_, false);
        register_output("/draft/safety_enabled", safety_enabled_output_, false);

        // 超时是硬件层的独立互锁，不依赖 RemoteControl 的仲裁/超时策略。
        // 因此上层即使保留旧的 PID 力矩，本组件仍会在超时后发送全零包。
        const auto remote_timeout_ms = get_parameter_or("remote_timeout_ms", 500);
        const auto feedback_timeout_ms = get_parameter_or("feedback_timeout_ms", 100);
        if (remote_timeout_ms <= 0 || feedback_timeout_ms <= 0) {
            RCLCPP_ERROR(get_logger(), "Timeouts must be positive; hardware output remains disabled");
        } else {
            remote_timeout_ = std::chrono::milliseconds{remote_timeout_ms};
            feedback_timeout_ = std::chrono::milliseconds{feedback_timeout_ms};
        }

        configure_motors();

        std::string board_serial;
        get_parameter_or("board_serial", board_serial, std::string{});
        board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
    }

    void update() override {
        // 与 my_rob_test 的 update 结构一致：这里只做状态解析、接口发布和日志，
        // 不在硬件层计算 PID、同步补偿或回零轨迹。
        update_dr16_status();
        update_motor_statuses();
        update_feedback_outputs();
        log_hardware_status();
    }

private:
    // =====================================================================
    // 周期状态更新
    // =====================================================================

    static std::int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
            .count();
    }

    void configure_motors() {
        const auto yaw_id = get_parameter_or("yaw_motor_id", 1);
        const auto pitch_right_id = get_parameter_or("pitch_right_motor_id", 2);
        const auto pitch_left_id = get_parameter_or("pitch_left_motor_id", 3);
        const auto yaw_reversed = get_parameter_or("yaw_motor_reversed", false);
        const auto pitch_right_reversed = get_parameter_or("pitch_right_motor_reversed", false);
        const auto pitch_left_reversed = get_parameter_or("pitch_left_motor_reversed", false);

        // M2006/C610 在同一 CAN 总线上的 ID 必须唯一；ID 1..4 共用 CAN ID 0x200，
        // 分别占据数据帧中的第 1..4 个 16-bit 槽位。
        const std::array ids{yaw_id, pitch_right_id, pitch_left_id};
        for (std::size_t index = 0; index < ids.size(); ++index) {
            motor_configuration_valid_[index] = ids[index] >= 1 && ids[index] <= 4;
            for (std::size_t other = 0; other < index; ++other)
                motor_configuration_valid_[index] = motor_configuration_valid_[index]
                    && ids[index] != ids[other];
        }
        if (!motor_configuration_valid_[0] || !motor_configuration_valid_[1]
            || !motor_configuration_valid_[2]) {
            RCLCPP_ERROR(
                get_logger(), "Motor IDs must be unique values in [1, 4]; hardware output disabled");
        }

        configure_motor(yaw_motor_, yaw_id, yaw_reversed);
        configure_motor(pitch_right_motor_, pitch_right_id, pitch_right_reversed);
        configure_motor(pitch_left_motor_, pitch_left_id, pitch_left_reversed);
    }

    static void configure_motor(device::DjiMotor& motor, int id, bool reversed) {
        auto config = device::DjiMotor::Config{
            device::DjiMotor::Type::kM2006, static_cast<std::uint8_t>(id)};
        // 回零后需要连续的位置量来计算左右高度差，故所有电机启用多圈角度。
        // reversed 在实测正方向后配置；首次接线测试都应保持 false。
        config.enable_multi_turn_angle();
        if (reversed)
            config.set_reversed();
        motor.configure(config);
    }

    void update_dr16_status() {
        dr16_.update_status();
        // RemoteControl 将解析结果发布为 /remote/joystick/* 和 /remote/switch/*，
        // 供同步控制器、后续 yaw 输入组件等控制层使用。
        remote_control_->update();
    }

    void update_motor_statuses() {
        const auto now = now_ns();
        if (feedback_fresh(MotorIndex::kYaw, now))
            yaw_motor_.update_status();
        if (feedback_fresh(MotorIndex::kPitchRight, now))
            pitch_right_motor_.update_status();
        if (feedback_fresh(MotorIndex::kPitchLeft, now))
            pitch_left_motor_.update_status();
    }

    void update_feedback_outputs() {
        const auto now = now_ns();
        *yaw_feedback_ready_output_ = feedback_fresh(MotorIndex::kYaw, now);
        *pitch_right_feedback_ready_output_ = feedback_fresh(MotorIndex::kPitchRight, now);
        *pitch_left_feedback_ready_output_ = feedback_fresh(MotorIndex::kPitchLeft, now);
        *safety_enabled_output_ = motion_enabled(now);
    }

    bool feedback_fresh(MotorIndex index, std::int64_t now) const {
        const auto received_at = motor_feedback_received_at_ns_[static_cast<std::size_t>(index)]
                                     .load(std::memory_order_relaxed);
        return received_at > 0 && now - received_at <= feedback_timeout_.count() * 1'000'000;
    }

    bool configuration_valid() const {
        return motor_configuration_valid_[0] && motor_configuration_valid_[1]
            && motor_configuration_valid_[2];
    }

    bool remote_fresh(std::int64_t now) const {
        const auto received_at = dr16_received_at_ns_.load(std::memory_order_relaxed);
        return received_at > 0 && now - received_at <= remote_timeout_.count() * 1'000'000;
    }

    bool dual_down() const {
        using Switch = rmcs_msgs::Switch;
        return dr16_.switch_left() == Switch::DOWN && dr16_.switch_right() == Switch::DOWN;
    }

    bool remote_switches_valid() const {
        using Switch = rmcs_msgs::Switch;
        return dr16_.switch_left() != Switch::UNKNOWN && dr16_.switch_right() != Switch::UNKNOWN;
    }

    bool motion_enabled(std::int64_t now) const {
        // 双下是操作者主动的硬件级失能动作。只要求“两个都 DOWN”，
        // 单个拨杆 DOWN 不应意外打断正常操作。
        // UNKNOWN 不是可操作状态，也必须失能；这可覆盖遥控器原始开关值异常。
        return configuration_valid() && remote_fresh(now) && dr16_.valid() && remote_switches_valid()
            && !dual_down()
            && feedback_fresh(MotorIndex::kYaw, now)
            && feedback_fresh(MotorIndex::kPitchRight, now)
            && feedback_fresh(MotorIndex::kPitchLeft, now);
    }

    // =====================================================================
    // 状态日志
    // =====================================================================

    void log_hardware_status() {
        const auto now = now_ns();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Draft safety=%d remote=(valid=%d,fresh=%d,switches_valid=%d,dual_down=%d) "
            "feedback=(yaw=%d,right=%d,left=%d)",
            static_cast<int>(motion_enabled(now)), static_cast<int>(dr16_.valid()),
            static_cast<int>(remote_fresh(now)), static_cast<int>(remote_switches_valid()),
            static_cast<int>(dual_down()),
            static_cast<int>(feedback_fresh(MotorIndex::kYaw, now)),
            static_cast<int>(feedback_fresh(MotorIndex::kPitchRight, now)),
            static_cast<int>(feedback_fresh(MotorIndex::kPitchLeft, now)));

        // 仅用于确认 DR16 物理摇杆方向与软件 x/y 轴的对应关系。当前约定是
        // 物理横向通道映射到 y：yaw 使用 left.y，pitch 使用 right.y。
        const auto joystick_left = dr16_.joystick_left();
        const auto joystick_right = dr16_.joystick_right();
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Draft DR16 joystick: left(x=%.3f,y=%.3f) right(x=%.3f,y=%.3f)",
            joystick_left.x(), joystick_left.y(), joystick_right.x(), joystick_right.y());

        // 初次接线/辨向时无需打开 Foxglove：直接手动转动电机即可在终端确认
        // CAN ID 是否对应、角度是否变化、速度正负号是否符合预期。只有反馈新鲜
        // 才打印数值，避免掉线时把上一次的旧状态误认为实时反馈。
        if (!feedback_fresh(MotorIndex::kYaw, now)
            || !feedback_fresh(MotorIndex::kPitchRight, now)
            || !feedback_fresh(MotorIndex::kPitchLeft, now)) {
            return;
        }
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Draft motors: yaw[a=%.3f rad,v=%.3f rad/s,t=%.3f Nm] "
            "pitch_right[a=%.3f rad,v=%.3f rad/s,t=%.3f Nm] "
            "pitch_left[a=%.3f rad,v=%.3f rad/s,t=%.3f Nm]",
            yaw_motor_.angle(), yaw_motor_.velocity(), yaw_motor_.torque(),
            pitch_right_motor_.angle(), pitch_right_motor_.velocity(), pitch_right_motor_.torque(),
            pitch_left_motor_.angle(), pitch_left_motor_.velocity(), pitch_left_motor_.torque());
    }

    // =====================================================================
    // 电机命令发送
    // =====================================================================

    void send_motor_command() {
        // 这里是最后一道安全闸门：不安全时不读取/使用上游 PID 的非零输出，
        // 而是对 CAN1 的三个电机槽位明确发送零电流。
        const bool enabled = motion_enabled(now_ns());
        const auto command_for = [enabled](const device::DjiMotor& motor) {
            return enabled ? motor.generate_command() : device::CanPacket8::Quarter{0};
        };
        std::array commands{
            device::CanPacket8::Quarter{0}, device::CanPacket8::Quarter{0},
            device::CanPacket8::Quarter{0}, device::CanPacket8::Quarter{0}};
        if (configuration_valid()) {
            commands[yaw_motor_.id() - 1] = command_for(yaw_motor_);
            commands[pitch_right_motor_.id() - 1] = command_for(pitch_right_motor_);
            commands[pitch_left_motor_.id() - 1] = command_for(pitch_left_motor_);
        }

        // DJI 0x200 的四个 quarter 按电机 ID 选择，不能按 C++ 成员声明顺序排。
        // 当前拓扑展开后是 [yaw(ID1), pitch_right(ID2), pitch_left(ID3), 空(ID4)]；
        // 此处仍根据运行参数的 ID 填槽，以便接线诊断时改 ID 不会误发给其他电机。
        auto packet = device::CanPacket8{
            commands[0], commands[1], commands[2], commands[3]};
        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1,
            {.can_id = 0x200, .can_data = packet.as_bytes()});
    }

protected:
    // =====================================================================
    // 板卡接收回调：只缓存原始设备状态，不在回调中运行控制算法
    // =====================================================================

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus)
            return;

        // 仅完整的 18 字节 DBUS 帧才刷新互锁时间戳；格式错误帧不可维持使能。
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        if (data.uart_data.size() == 18)
            dr16_received_at_ns_.store(now_ns(), std::memory_order_relaxed);
    }

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan1 || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;

        // 三轴任意一个反馈丢失都会切断全部输出，避免龙门架单侧继续施力。
        const auto received_at = now_ns();
        if (yaw_motor_.match_then_store_status(data.can_id, data.can_data))
            motor_feedback_received_at_ns_[static_cast<std::size_t>(MotorIndex::kYaw)]
                .store(received_at, std::memory_order_relaxed);
        if (pitch_right_motor_.match_then_store_status(data.can_id, data.can_data))
            motor_feedback_received_at_ns_[static_cast<std::size_t>(MotorIndex::kPitchRight)]
                .store(received_at, std::memory_order_relaxed);
        if (pitch_left_motor_.match_then_store_status(data.can_id, data.can_data))
            motor_feedback_received_at_ns_[static_cast<std::size_t>(MotorIndex::kPitchLeft)]
                .store(received_at, std::memory_order_relaxed);
    }
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Draft, rmcs_executor::Component)
