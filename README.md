# 飞镖发射架 Pitch / Yaw 控制（RMCS `feat/gantry-control`）

本分支在 RMCS（RoboMaster Control System，ROS2）上实现龙门式飞镖发射架的 Pitch 升降与 Yaw 转向控制，面向使用 C 板、3 个 M2006 电机与 C610 电调、DR16 遥控器的配置。控制配置见 [`gantry_draft.yaml`](rmcs_ws/src/rmcs_bringup/config/gantry_draft.yaml)。

## 控制与测试情况

- DR16 分别控制 Pitch 和 Yaw；Pitch 使用左右两侧电机的速度 PID，并根据两侧编码器位置差进行同步修正，减少升降时的歪斜。
- 已完成实操验证：在统一高度进行 Pitch 升降时，两侧运动保持同步；单独控制 Yaw 时，Pitch 角度不发生改变。以上为项目组实操结果，未在此独立复测。
- 回零控制逻辑已实现，但龙门架到达上端归零点的实操验证尚未完成。因此，本分支目前不声称上端回零已通过测试。

## 同步误差说明

肉眼下看不出 Pitch 两侧明显的高度差，但在实际的飞镖发射过程中，这一点点差距也是不可忽视的一环。本项目已注意到该影响，但未做进一步深入研究。

根据实机测试：在两个 Pitch 控制电机给定相同速度、发射架从上端运动到最下端的情况下，两侧的最终误差只有 2-3 rad（电机侧反馈读数）；由于电机减速比，该误差传递到螺杆上时，左右两边几乎看不出高度差。

## 主要实现位置

- 控制配置：`rmcs_ws/src/rmcs_bringup/config/gantry_draft.yaml`
- 硬件组件（发射架、M2006 电机、DR16 遥控器）：`rmcs_ws/src/rmcs_core/src/hardware/drafts.cpp`
- Pitch 同步、回零控制与日志：`rmcs_ws/src/rmcs_core/src/test/gantry_velocity_sync_controller.cpp`
- Yaw 控制器：`rmcs_ws/src/rmcs_core/src/controller/gantry/draft_yaw_controller.cpp`

## 实操录像

Pitch 同步升降与 Yaw 解耦实操录像：[VID_20260921_211349.mp4](docs/gantry-control/VID_20260921_211349.mp4)。
