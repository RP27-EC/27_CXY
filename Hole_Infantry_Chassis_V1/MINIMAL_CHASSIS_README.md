# H723 最小遥控麦轮底盘

已直接修改工程：`D:\C++\RoboMaster\27_CXY_Hole_Infantry\Hole_Infantry_Chassis\Hole_Infantry_Chassis_V1`。

最终使用原 Keil ARMCC 5.06、目标 `DM-MC02` 编译通过：**0 Error(s), 0 Warning(s)**。未执行烧录或实车运行；编译通过不代表电机物理方向已验证。

## 使用方法

1. 左摇杆回中，左开关 **s1 拨下**，等待遥控器和四个电机反馈正常。
2. 左摇杆保持回中，将两个开关都拨上：**s1=UP、s2=UP**，启用底盘。允许 s1 从下拨上时经过中档。
3. 左摇杆上下 `ch3` 控制前后，左右 `ch2` 控制横移。右摇杆、滚轮、键鼠均不参与控制。
4. 任一开关离开上档立即停止输出；遥控器掉线、非法 DBUS 数据、任一电机反馈超时也停止输出。故障恢复后重新执行第 1、2 步。

上电不会自动启用；直接双上档上电、带杆切入均不会启动。停止时发送 `0x200` 的八字节全零帧，并清除四个 PID 的积分、误差历史和输出，不执行零目标速度闭环制动。全零电流属于卸力，车会受惯性滑行。

## 代码入口与参数

相对于上述工程目录：

- `Application/TaskLayer/control_task.c`：`StartCtrlTask()`，每 1 ms 调用一次底盘。
- `Application/ModuleLayer/Chassis.c`：`Chassis_Init()`、`Chassis_Step()`；唯一的底盘运算和电流发送入口。
- **`Application/ConfigLayer/chassis_minimal_config.h`**：集中修改默认转速、死区、PID、四轮 ESC ID、方向符号。新增头文件由代码直接包含，不需要修改工程分组。
- `Application/ProtocolLayer/rc_protocol.c`：沿用 UART5 DBUS 位布局，只解析摇杆和开关，增加有效性与时间戳检查。
- `Application/ProtocolLayer/can_protocol.c`：仅分发四个 3508 反馈，复用原 `RM_motor.c` 解析。

主链路：`ch3/ch2 → 死区20/归一化 → front/left → 麦轮逆解 → 四轮目标 → 原 single_pid_ctrl → FDCAN1 0x200`。`cycle_speed` 恒为 0，无旋转控制输入。

原工程 `encoder_speed` 是 **电机转子 rpm**，`speed` 是减速后输出轴 rad/s，使用原减速比 19：

`输出轴 rad/s = 转子 rpm × 2π ÷ (60 × 19)`。

默认每轮目标最多 **1500 转子 rpm**（约 8.27 rad/s）；斜向输入会整体等比例缩放，也不会超过这个上限。队内硬上限 **5000 转子 rpm**（约 27.56 rad/s），配置超过它会编译报错；实测反馈超过 5000 时也触发停机。软件限制不能保证机械瞬态绝无超调。

复用原速度 PID 的单位与 `Kp=1、Ki=0、Kd=0`，初始扭矩输出限幅降至 **1.5 N·m**，按原驱动换算约 **5 A / 原始电流值4096**。遥控超时 60 ms，电机反馈超时 100 ms。

## 已核对的四轮映射

所有电机均在 **FDCAN1**，统一发送标准帧 **0x200、8字节**。表格来自原工程 `motor.h`、`motor.c` 和 `Chassis.c`，仍需核对实车接线。

| 数组下标 | 原轮位名称 | ESC ID / 反馈 ID | 0x200 字节 | 原逆解式，旋转为零 |
|---|---|---|---|---|
| 0 / WHEEL_LF | 左前 | 1 / 0x201 | 0、1 | -front + left |
| 1 / WHEEL_LB | 左后 | 2 / 0x202 | 2、3 | -front - left |
| 2 / WHEEL_RF | 右前 | 3 / 0x203 | 4、5 | +front + left |
| 3 / WHEEL_RB | 右后 | 4 / 0x204 | 6、7 | +front - left |

保留原始通道符号：`front=-ch3`，`left=+ch2`。整车前后反向改 `CHASSIS_FRONT_RC_SIGN`；左右反向改 `CHASSIS_LEFT_RC_SIGN`；单轮安装反向改该轮 `CHASSIS_*_SIGN`。轮位与 ESC ID 对不上则集中改 `CHASSIS_*_ESC_ID`，接收 ID 和发送槽位会一起跟随。不要只反转 PID 反馈符号。

## 保留与禁用范围

保留 H723 启动、时钟、HAL/Drivers、FreeRTOS 内核、GPIO/UART/FDCAN/DMA/中断配置、UART5 原有 DMA 接收、RM 电机基础驱动与反馈换算、通用 PID。`uvprojx`、启动文件、链接脚本、第三方库均未改；没有删除工程文件。

清空复杂底盘实现、整车状态机、云台、发射、视觉、UI、应用服务/解算等实现，保留文件名和兼容类型。仅创建控制任务；旧监控、命令、观测、UI、板间通信任务不再创建。关掉超电、功率分配、打滑等功能开关；IMU、裁判系统、超电等设备/协议底层文件保留但不初始化、不用于底盘。此版不具备原比赛功率管理功能。

必要的小改动：驱动初始化只启动 UART5 接收与 FDCAN1；FDCAN 接收检查标准数据帧和八字节长度；发送函数返回真实错误状态，底盘停机时取消待发旧电流，发送失败则保持停机并继续尝试零电流。物理 CAN 断线时不能保证帧送达。

## 第一次上车前检查（四项）

1. 确认烧录的是这份 H723 工程生成的新 HEX/AXF，四个 3508/C620 都接 FDCAN1，反馈 ID 与上表一致。
2. 四轮离地，一次检查四轮：上电双上档不会动；按“回中、s1下、再双上”启用。只做小幅前后/横移，核对轮位、麦轮安装和方向。
3. 检查停机：任一开关离开上档、关遥控器，应使四路 `current_raw` 为零；掉线恢复不能自行继续运动。
4. 确认默认 1500 rpm、无旋转输入，四轮反馈正常后再低速落地。方向或开关实物位置不符时先停机，改集中参数后重编译。

可在调试器观察 `chassis_debug`：`armed`、`ready`、`target_rpm[4]`、`feedback_rpm[4]`、`current_raw[4]`、`fault`、`tx_errors`。`fault`：1=遥控异常，2=电机超时，3=超速，4=发送失败。

## 恢复原版

关闭该工程，用你已有的完整备份恢复原工程目录，再重新编译，避免使用旧 HEX。开始时也已完成一份完整原版 ZIP（在你说已备份之前完成），与本说明同目录，名称为 `Hole_Infantry_Chassis_V1_original_20260921_225827.zip`。恢复时可先解压到新的目录，打开其中的 `MDK-ARM/DM-MC02.uvprojx`，不必覆盖现在的版本。

验证范围：原工程实际编译、源文件差异核对及控制路径静态检查。尚未完成实车方向、遥控开关实物位置、闭环响应与 CAN 送达验证。
