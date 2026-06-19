#! /usr/bin/env python3
"""
Kinova Gen3 7轴机械臂关节力矩控制程序 
================================================================

本程序实现了Kinova Gen3机械臂的7轴独立力矩控制，支持：
- 每个关节的独立参数化控制器配置
- 实时轨迹跟踪控制
- 角度连续性处理和安全限制
- 完整的数据记录和可视化
- 控制结束后直接停止（不切换模式）
- 独立的7个关节跟踪误差图，每个图包含±0.2度放大视图
- 综合分析图表：包含角度误差、角速度误差、u1、u2、K1、K2参数变化

作者: [Wen Yan]
日期: [Date]
版本: 3.1 - Individual Joint Analysis with Real-time K1/K2 Monitoring
"""

# ===========================
# 模块导入部分
# ===========================
import sys                    # 系统相关功能
import os                     # 操作系统接口
import json
import time                   # 时间相关功能
import threading              # 多线程支持
import math                   # 数学计算
import select                 # I/O多路复用
import numpy as np            # 数值计算库
import matplotlib.pyplot as plt  # 绘图库
from mpl_toolkits.axes_grid1.inset_locator import inset_axes  # 用于图中图
# 注意：图表标注使用英文，无需中文字体支持

# 导入自定义控制器模块
from control_main import control_main

# 导入Kortex API相关模块 - Kinova机械臂通信接口
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

# ===========================
# 工具函数部分
# ===========================

def load_runtime_config(config_path):
    if not config_path:
        return {}

    config_path = os.path.abspath(config_path)
    with open(config_path, "r", encoding="utf-8") as config_file:
        return json.load(config_file)


def find_kortex_examples_dir(start_dir):
    current = os.path.abspath(start_dir)
    while True:
        if os.path.exists(os.path.join(current, "utilities.py")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            raise FileNotFoundError(
                "Cannot find Kinova examples/utilities.py. Put this package under "
                "api_python/examples or add that directory to PYTHONPATH."
            )
        current = parent


def normalize_angle_to_180(angle_deg):
    """
    角度归一化函数
    
    将任意角度值归一化到[-180, 180]度范围内，确保角度表示的一致性
    
    Args:
        angle_deg (float): 输入角度值（度）
        
    Returns:
        float: 归一化后的角度值（度）
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0  # 使用模运算实现角度归一化

# ===========================
# 角度连续性处理类
# ===========================

class AngleUnwrapper:
    """
    角度连续性处理类
    
    用于处理角度传感器读数中的360度跳跃问题，确保角度数据的连续性。
    当角度从179度跳到-179度时，能够正确识别并保持数据连续性。
    """
    
    def __init__(self, soft_limit=170.0):
        """
        初始化角度连续性处理器
        
        Args:
            soft_limit (float): 软限制角度，超过此角度将触发安全警告
        """
        self.previous_angle = None      # 上一次的角度值，用于检测跳跃
        self.cumulative_offset = 0.0    # 累积偏移量，用于保持连续性
        self.jump_threshold = 180.0     # 跳跃检测阈值
        self.soft_limit = soft_limit    # 软限制边界
        
    def unwrap_angle(self, current_angle_deg):
        """
        处理角度连续性，消除360度跳跃
        
        Args:
            current_angle_deg (float): 当前原始角度读数
            
        Returns:
            tuple: (连续角度, 归一化角度, 是否在安全范围内)
        """
        # 首先将当前角度归一化到[-180, 180]范围
        normalized_current = normalize_angle_to_180(current_angle_deg)
        
        # 如果是第一次调用，初始化参考角度
        if self.previous_angle is None:
            self.previous_angle = normalized_current
            is_within_limits = abs(normalized_current) <= self.soft_limit
            return normalized_current, normalized_current, is_within_limits
        
        # 计算与上次角度的差值，检测是否发生跳跃
        angle_diff = normalized_current - self.previous_angle
        
        # 检测负向跳跃（从179到-179）
        if angle_diff < -self.jump_threshold:
            self.cumulative_offset += 360.0    # 增加360度偏移
        # 检测正向跳跃（从-179到179）
        elif angle_diff > self.jump_threshold:
            self.cumulative_offset -= 360.0    # 减少360度偏移
        
        # 更新上次角度记录
        self.previous_angle = normalized_current
        
        # 计算连续角度和安全状态
        continuous_angle = normalized_current + self.cumulative_offset
        is_within_limits = abs(normalized_current) <= self.soft_limit
        
        return continuous_angle, normalized_current, is_within_limits
    
    def reset(self):
        """重置角度连续性处理器状态"""
        self.previous_angle = None
        self.cumulative_offset = 0.0

# ===========================
# 主控制类
# ===========================

class TorqueControlSevenAxis:
    """
    Kinova Gen3 7轴机械臂力矩控制主类
    
    实现完整的7轴机械臂力矩控制功能，包括：
    - 初始化和配置管理
    - 实时控制循环
    - 安全监控和数据记录
    - 独立关节误差可视化
    - 综合控制参数分析
    """

    def __init__(self, router, router_real_time, config=None):
        """
        初始化7轴力矩控制器
        
        Args:
            router: TCP通信路由器，用于配置和命令
            router_real_time: UDP通信路由器，用于实时控制
        """
        # ===========================
        # 基本控制参数配置
        # ===========================
        self.config = config or {}
        self.run_duration = self.config.get("run_duration", 20)          # 控制运行持续时间（秒）
        self.control_active = False     # 控制激活状态标志
        
        # ===========================
        # 控制限制参数配置
        # ===========================
        self.u1_limit = self.config.get("u1_limit", 40.0)           # u1输出限制（±40）
        self.u2_max_torque = self.config.get("u2_max_torque", 40.0)       # 正常控制时最大力矩限制
        self.u2_safety_torque = self.config.get("u2_safety_torque", 5.0)     # 安全模式时力矩限制

        # ===========================
        # Kortex API客户端初始化
        # ===========================
        # 设备管理客户端 - 用于读取设备信息
        self.device_manager = DeviceManagerClient(router)
        # 执行器配置客户端 - 用于设置控制模式
        self.actuator_config = ActuatorConfigClient(router)
        # 基础控制客户端 - 用于高级命令
        self.base = BaseClient(router)
        # 实时循环控制客户端 - 用于低级实时控制
        self.base_cyclic = BaseCyclicClient(router_real_time)

        # ===========================
        # 通信消息结构初始化
        # ===========================
        # 命令消息 - 发送给机械臂的控制指令
        self.base_command = BaseCyclic_pb2.Command()
        # 反馈消息 - 从机械臂接收的状态信息
        self.base_feedback = BaseCyclic_pb2.Feedback()

        # ===========================
        # 设备验证和配置
        # ===========================
        # 读取所有连接的设备信息
        device_handles = self.device_manager.ReadAllDevices()
        # 获取执行器（关节）数量
        self.actuator_count = self.base.GetActuatorCount().count

        # 验证是否为7轴机械臂
        if self.actuator_count != 7:
            raise ValueError(f"此程序需要7轴机械臂，当前机械臂有 {self.actuator_count} 个轴")

        # ===========================
        # 通信结构体配置
        # ===========================
        # 为每个执行器添加命令和反馈结构
        for handle in device_handles.device_handle:
            if handle.device_type == Common_pb2.BIG_ACTUATOR or handle.device_type == Common_pb2.SMALL_ACTUATOR:
                self.base_command.actuators.add()      # 添加执行器命令结构
                self.base_feedback.actuators.add()     # 添加执行器反馈结构

        # ===========================
        # 通信选项配置
        # ===========================
        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False       # 等待确认，确保命令送达
        self.sendOption.delay_ms = 0            # 无延迟发送
        self.sendOption.timeout_ms = 3          # 3ms超时设置

        # ===========================
        # 线程控制变量
        # ===========================
        self.kill_thread = False        # 线程终止标志
        self.thread = None              # 控制线程对象
        self.control_started = False    # 控制开始标志
        
        # ===========================
        # 7轴控制器参数配置
        # ===========================
        # 每个轴的独立控制参数 - 根据各轴特性优化
        self.axis_control_params = [
            # 轴1参数：基座旋转轴，需要较强的控制能力
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.5, "AF22_0": 0.2},
            
            # 轴2参数：肩部俯仰轴，承载较大负载
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.11,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.2, "AF22_0": 0.1},
            
            # 轴3参数：肩部滚转轴，中等负载
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.3, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.6, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.6},
            
            # 轴4参数：肘部弯曲轴，需要快速响应
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.0, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.3, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.5},
            
            # 轴5参数：前臂旋转轴，轻负载高精度
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.3, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            
            # 轴6参数：腕部俯仰轴，精细控制
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.2, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            
            # 轴7参数：腕部旋转轴，最轻负载最高精度
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.01, "w2": 0.001, "dag_deg": 1.8, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 0.1, "AF22_0": 0.05}
        ]
        
        # ===========================
        # 控制器和处理器初始化
        # ===========================
        self.controllers = []           # 各轴控制器列表
        self.angle_unwrappers = []      # 各轴角度处理器列表
        self.torque_values = [0.0] * 7  # 当前力矩输出值
        self.previous_u1_values = [0.0] * 7  # 上一次的u1值，用于计算修正后的速度误差
        self.safety_stop_flag = False   # 安全停止标志
        self.axis_limits = self.config.get("axis_limits_deg", [170.0] * 7)  # 各轴角度软限制
        if len(self.axis_limits) != 7:
            raise ValueError("axis_limits_deg must contain 7 values")
        if "axis_control_params" in self.config:
            if len(self.config["axis_control_params"]) != 7:
                raise ValueError("axis_control_params must contain 7 joint parameter sets")
            self.axis_control_params = self.config["axis_control_params"]
        self.control_completed = False  # 控制完成标志，避免重复清理
        
        # 为每个轴创建独立的控制器和角度处理器
        for i in range(7):
            params = self.axis_control_params[i]
            # 创建该轴的控制器实例
            controller = control_main(
                u1_init=params["u1_init"], u2_init=params["u2_init"],
                K1_init=params["K1_init"], K2_init=params["K2_init"],
                w1=params["w1"], w2=params["w2"], dag_deg=params["dag_deg"],
                b1=params["b1"], b2=params["b2"], AF11_0=params["AF11_0"],
                AF12_0=params["AF12_0"], AF21_0=params["AF21_0"], AF22_0=params["AF22_0"]
            )
            self.controllers.append(controller)
            # 创建该轴的角度连续性处理器
            self.angle_unwrappers.append(AngleUnwrapper(soft_limit=self.axis_limits[i]))
        
        # ===========================
        # 数据记录初始化
        # ===========================
        # 注意：这些列表在每次启动控制时会被清空，确保数据的独立性
        self.time_history = []                              # 时间历史记录
        self.position_history = [[] for _ in range(7)]      # 位置历史记录
        self.velocity_history = [[] for _ in range(7)]      # 速度历史记录
        self.torque_history = [[] for _ in range(7)]        # 力矩历史记录
        self.u1_history = [[] for _ in range(7)]            # u1历史记录
        self.desired_history = [[] for _ in range(7)]       # 期望位置历史记录
        self.desired_velocity_history = [[] for _ in range(7)]  # 期望速度历史记录
        self.error_history = [[] for _ in range(7)]         # 位置误差历史记录
        self.velocity_error_history = [[] for _ in range(7)]  # 速度误差历史记录
        
        # 新增：控制参数历史记录
        self.K1_history = [[] for _ in range(7)]            # K1参数历史记录
        self.K2_history = [[] for _ in range(7)]            # K2参数历史记录

        # ===========================
        # 7轴轨迹参数配置
        # ===========================
        # 每个轴的轨迹参数：offset_deg + amplitude_deg * sin(ωt + phase_offset)
        # 不同的偏移位置和频率相位，模拟真实工作场景
        self.axis_configs = [
            {"offset_deg": 5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.0},    # 轴1：5°±10°正弦运动
            {"offset_deg": -5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.5*np.pi},  # 轴2：-5°±10°正弦运动  
            {"offset_deg": 5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.0},   # 轴3：5°±10°正弦运动
            {"offset_deg": -5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.5*np.pi},  # 轴4：-5°±10°正弦运动
            {"offset_deg": 5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.0},   # 轴5：5°±10°正弦运动
            {"offset_deg": -5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.5*np.pi},  # 轴6：-5°±10°正弦运动
            {"offset_deg": 5.0, "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.0}     # 轴7：5°±10°正弦运动
        ]
        if "axis_trajectory" in self.config:
            if len(self.config["axis_trajectory"]) != 7:
                raise ValueError("axis_trajectory must contain 7 joint trajectory parameter sets")
            self.axis_configs = self.config["axis_trajectory"]

        print(f"已配置7轴机械臂力矩控制（个体关节分析版）")
        print(f"   初始位置: 0°（所有轴）")
        print(f"   期望轨迹: θ_d(t) = offset_deg + amplitude_deg × sin(2πf×t + φ)")
        print(f"   角度范围: ±{self.axis_limits[0]}° (软限制)")
        print(f"   将生成独立关节跟踪误差图和个体关节控制参数分析图")
        print(f"   控制挑战：从0°跟踪到带偏移的正弦轨迹")

    def compute_desired_trajectory(self, axis_index, time_value):
        """
        计算指定轴的期望轨迹
        
        实现轨迹：θ_d(t) = offset_deg + amplitude_deg * sin(ωt + φ)
        这种形式更符合实际应用，在某个工作位置附近进行小幅度正弦运动。
        
        Args:
            axis_index (int): 轴索引 (0-6)
            time_value (float): 当前时间值（秒）
            
        Returns:
            tuple: (期望位置(rad), 期望速度(rad/s))
        """
        # 获取该轴的轨迹配置参数
        config = self.axis_configs[axis_index]
        offset_rad = config["offset_deg"] * math.pi / 180.0        # 偏移角度（弧度）
        amplitude_rad = config["amplitude_deg"] * math.pi / 180.0   # 振幅（弧度）
        omega = 2.0 * math.pi * config["frequency_hz"]             # 角频率（rad/s）
        phase = config["phase_offset"]                             # 相位偏移
        
        # 计算带偏移的正弦轨迹
        # 位置：θ_d(t) = offset + amplitude * sin(ωt + φ)
        desired_pos = offset_rad + amplitude_rad * np.sin(omega * time_value + phase)
        # 速度：θ̇_d(t) = amplitude * ω * cos(ωt + φ)
        desired_vel = amplitude_rad * omega * np.cos(omega * time_value + phase)
        
        # 安全限制：确保期望位置（包括偏移）不会超出软限制
        max_limit_rad = (self.axis_limits[axis_index] - 5.0) * math.pi / 180.0
        min_limit_rad = -(self.axis_limits[axis_index] - 5.0) * math.pi / 180.0
        
        # 如果计算的期望位置超出限制，进行裁剪
        if desired_pos > max_limit_rad or desired_pos < min_limit_rad:
            desired_pos = np.clip(desired_pos, min_limit_rad, max_limit_rad)
            # 如果位置被裁剪，速度设为0（避免持续推向边界）
            desired_vel = 0.0
            
        return desired_pos, desired_vel

    def example_move_to_start_position(self):
        """
        移动到起始位置（所有关节角度为0度）
        
        这是控制开始前的准备步骤，确保机械臂从已知的安全位置（0度）开始控制。
        然后机械臂将跟踪带偏移的正弦轨迹：θ_d(t) = offset_deg + amplitude_deg * sin(ωt+φ)
        
        Returns:
            bool: 移动是否成功完成
        """
        print("   正在执行移动到0度操作...")
        
        # 重置所有轴的角度连续性处理器，清除历史状态
        for unwrapper in self.angle_unwrappers:
            unwrapper.reset()
        
        # 切换到单层伺服模式 - 高级运动控制模式
        print("   - 切换到单层伺服模式")
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # 准备关节角度目标 - 所有轴设为0度（安全起始位置）
        print("   - 设置目标位置：所有关节0度")
        constrained_joint_angles = Base_pb2.ConstrainedJointAngles()
        angles = [0.0] * 7  # 目标角度数组：所有轴都是0度

        # 为每个关节设置目标角度
        for joint_id in range(7):
            joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id    # 关节ID
            joint_angle.value = angles[joint_id]        # 目标角度值（0度）

        # ===========================
        # 异步运动控制事件处理
        # ===========================
        finished_event = threading.Event()  # 运动完成事件

        def check_for_end_or_abort(notification, event=finished_event):
            """检查运动是否完成或中止的回调函数"""
            if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
                event.set()  # 设置事件，通知主线程运动完成

        # 订阅动作通知，监控运动状态
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort, Base_pb2.NotificationOptions())

        # 开始执行关节轨迹运动
        print("   - 开始执行关节运动...")
        self.base.PlayJointTrajectory(constrained_joint_angles)

        # 等待运动完成或超时
        print("   - 等待运动完成（超时时间：20秒）...")
        finished = finished_event.wait(self.run_duration)
        self.base.Unsubscribe(notification_handle)  # 取消订阅

        # 返回运动结果并提供状态反馈
        if finished:
            print("   ✅ 成功到达0度起始位置")
            return True
        else:
            print("   ❌ 运动超时，可能未完全到达目标位置")
            return False

    def InitTorqueControl(self):
        """
        初始化7轴力矩控制模式
        
        将机械臂从位置控制模式切换到力矩控制模式，这是一个关键的模式转换过程。
        包括低级伺服模式设置、执行器配置和初始命令发送。
        
        Returns:
            bool: 初始化是否成功
        """
        print("   正在执行力矩控制初始化...")
        
        try:
            # ===========================
            # 获取初始状态反馈
            # ===========================
            print("   - 获取当前机械臂状态...")
            # 刷新反馈，获取当前机械臂状态
            self.base_feedback = self.base_cyclic.RefreshFeedback()

            # ===========================
            # 配置初始命令结构
            # ===========================
            print("   - 配置初始命令结构...")
            # 为所有执行器启用伺服并设置初始位置
            for i in range(7):
                self.base_command.actuators[i].flags = 1    # 启用执行器标志
                # 设置当前位置为初始命令位置，避免突然跳跃
                self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                self.base_command.actuators[i].torque_joint = 0.0  # 初始力矩为0

            # ===========================
            # 切换到低级伺服模式
            # ===========================
            print("   - 切换到低级伺服模式...")
            # 低级伺服模式允许直接的力矩控制
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)

            # 发送首个命令帧，建立通信
            self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)

            # ===========================
            # 执行器模式切换
            # ===========================
            print("   - 将所有7个执行器切换为力矩控制模式...")
            # 将所有7个执行器从位置控制切换为力矩控制模式
            for axis_id in range(1, 8):  # Kinova API使用1-8的轴ID
                control_mode_msg = ActuatorConfig_pb2.ControlModeInformation()
                control_mode_msg.control_mode = ActuatorConfig_pb2.ControlMode.TORQUE
                self.actuator_config.SetControlMode(control_mode_msg, axis_id)

            print(f"   ✅ 所有7轴已切换到力矩控制模式")
            
            # ===========================
            # 系统稳定化过程
            # ===========================
            print("   - 保持0力矩状态0.005秒系统稳定...")
            stable_start_time = time.time()
            frame_id = 0
            
            while (time.time() - stable_start_time) < 0.005:
                # 刷新反馈，获取最新状态
                self.base_feedback = self.base_cyclic.RefreshFeedback()
                
                # 设置所有轴的0力矩命令
                for i in range(7):
                    self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                    self.base_command.actuators[i].torque_joint = 0.0
                    
                # 更新帧ID和命令ID，确保通信同步
                frame_id = (frame_id + 1) % 65536  # 16位循环计数器
                self.base_command.frame_id = frame_id
                for i in range(7):
                    self.base_command.actuators[i].command_id = frame_id
                    
                # 发送0力矩命令
                try:
                    self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                except Exception as err:
                    print(f"   ⚠️  警告: 发送0力矩命令失败: {err}")
                    
                time.sleep(0.001)  # 1ms循环周期
            
            print(f"   ✅ 力矩控制初始化完成，系统稳定")
            return True
            
        except Exception as e:
            print(f"   ❌ 力矩控制初始化失败: {e}")
            print(f"   错误详情: 请检查机械臂连接和状态")
            return False

    def StartTorqueControl(self):
        """
        启动7轴力矩控制线程
        
        创建并启动控制线程，开始实时力矩控制循环。
        控制循环运行在独立线程中，避免阻塞主程序。
        """
        # 检查是否已有控制线程在运行
        if self.thread and self.thread.is_alive():
            print("   ❌ 控制已在运行中")
            return
        
        print("   正在启动动力学控制...")
        
        # ===========================
        # 控制前重置
        # ===========================
        print("   - 重置控制器状态和数据记录...")
        # 重置所有轴的角度处理器，清除历史状态
        for unwrapper in self.angle_unwrappers:
            unwrapper.reset()
        self.safety_stop_flag = False   # 重置安全标志
        self.control_started = True     # 标记控制已开始
        self.plot_saved = False         # 重置图表保存标志
        self.control_completed = False  # 重置控制完成标志
        
        # 清空历史数据，为新的控制循环做准备
        self.time_history.clear()
        for i in range(7):
            self.position_history[i].clear()
            self.velocity_history[i].clear()
            self.torque_history[i].clear()
            self.u1_history[i].clear()
            self.desired_history[i].clear()
            self.desired_velocity_history[i].clear()
            self.error_history[i].clear()
            self.velocity_error_history[i].clear()
            self.K1_history[i].clear()  # 清空K1历史
            self.K2_history[i].clear()  # 清空K2历史
        
        # 重置u1的上一时刻值（类似matlab的memory模块）
        self.previous_u1_values = [0.0] * 7
        
        # ===========================
        # 启动控制线程
        # ===========================
        print("   - 创建实时控制线程...")
        self.control_active = True      # 设置控制活动标志
        self.kill_thread = False        # 清除线程终止标志
        # 创建并启动控制线程
        self.thread = threading.Thread(target=self.RunTorqueLoop)
        self.thread.start()
        
        print("   ✅ 动力学控制线程已启动")

    def get_continuous_positions(self):
        """
        获取所有轴的连续位置角度
        
        处理从机械臂获取的原始角度数据，通过角度连续性处理器
        消除360度跳跃，获得连续的角度值。
        
        Returns:
            tuple: (连续位置列表, 归一化位置列表, 原始位置列表, 是否全部在安全范围内)
        """
        continuous_positions = []   # 连续角度值
        normalized_positions = []   # 归一化角度值
        raw_positions = []          # 原始角度值
        all_within_limits = True    # 安全状态标志
        
        # 处理每个轴的角度数据
        for i in range(7):
            # 获取原始位置反馈
            raw_position_deg = self.base_feedback.actuators[i].position
            raw_positions.append(raw_position_deg)
            
            # 通过角度处理器获取连续角度和安全状态
            continuous_angle, normalized_angle, is_within_limits = self.angle_unwrappers[i].unwrap_angle(raw_position_deg)
            
            continuous_positions.append(continuous_angle)
            normalized_positions.append(normalized_angle)
            
            # 检查是否有轴超出安全限制
            if not is_within_limits:
                all_within_limits = False
        
        return continuous_positions, normalized_positions, raw_positions, all_within_limits

    def RunTorqueLoop(self):
        """
        7轴力矩控制主循环
        
        这是系统的核心控制循环，实现实时的力矩控制。
        循环中包括：
        - 状态反馈获取
        - 轨迹计算
        - 控制律计算
        - 安全监控
        - 命令发送
        - 数据记录
        """
        print(f"   开始动力学控制循环...")
        print(f"   - 控制持续时间: {self.run_duration}秒")
        print(f"   - 循环频率: 1000Hz (1ms周期)")
        print(f"   - 数据记录: 实时记录所有控制参数")
        print(f"   - 自适应参数: 实时监控K1、K2变化")
        
        start_time = time.time()    # 记录开始时间
        frame_id = 0                # 通信帧ID

        # ===========================
        # 主控制循环
        # ===========================
        while not self.kill_thread and (time.time() - start_time) < self.run_duration and not self.safety_stop_flag:
            loop_time = time.time() - start_time  # 当前循环时间

            # ===========================
            # 状态反馈获取和处理
            # ===========================
            # 获取所有轴的连续位置角度和安全状态
            continuous_positions_deg, normalized_positions_deg, raw_positions_deg, all_within_limits = self.get_continuous_positions()
            
            # 安全检查：如果有轴超出限制，启用安全停止
            if not all_within_limits:
                print(f"   ⚠️  检测到轴超出软限制，启用安全停止...")
                self.safety_stop_flag = True
            
            # 获取所有轴的速度反馈
            velocities_deg_per_sec = []
            for i in range(7):
                velocities_deg_per_sec.append(self.base_feedback.actuators[i].velocity)

            # ===========================
            # 各轴控制律计算
            # ===========================
            for axis_idx in range(7):
                # 单位转换：度 -> 弧度
                position_rad = continuous_positions_deg[axis_idx] * math.pi / 180.0
                velocity_rad_per_sec = velocities_deg_per_sec[axis_idx] * math.pi / 180.0

                # 计算该轴的期望轨迹（位置和速度）
                desired_rad, desired_vel = self.compute_desired_trajectory(axis_idx, loop_time)

                # 计算位置误差和真实速度误差（用于记录）
                position_error = position_rad - desired_rad      # 位置误差
                velocity_error = velocity_rad_per_sec - desired_vel  # 真实的关节角速度误差

                # 使用上一时刻的u1值计算修正后的速度误差（类似matlab的memory模块）
                modified_velocity_error = velocity_rad_per_sec - (desired_vel + self.previous_u1_values[axis_idx])

                # 调用该轴的自适应控制律，输入修正后的速度误差
                u1, u2 = self.controllers[axis_idx].control_law(position_error, modified_velocity_error)

                # 对u1进行限制
                u1 = max(min(u1, self.u1_limit), -self.u1_limit)

                # 使用u2作为力矩输出
                self.torque_values[axis_idx] = u2

                # 获取控制器的当前K1和K2值
                try:
                    # 根据control_main类的实现，K1和K2存储在积分器的output中
                    K1 = self.controllers[axis_idx].K1_integrator.output[0]
                    K2 = self.controllers[axis_idx].K2_integrator.output[0]
                except Exception as e:
                    # 如果获取失败，使用初始值
                    if loop_time < 1.0:  # 只在前1秒打印警告，避免刷屏
                        print(f"   ⚠️  警告: 获取关节{axis_idx+1}的K1、K2值失败: {e}")
                    K1 = self.axis_control_params[axis_idx]["K1_init"]
                    K2 = self.axis_control_params[axis_idx]["K2_init"]

                # ===========================
                # 安全力矩限制
                # ===========================
                # 如果检测到安全问题，逐渐减小力矩输出
                if self.safety_stop_flag:
                    safety_factor = max(0.0, 1.0 - (loop_time - start_time) * 2.0)
                    self.torque_values[axis_idx] *= safety_factor

                # 硬件保护：限制输出力矩范围
                max_torque = self.u2_max_torque if not self.safety_stop_flag else self.u2_safety_torque
                self.torque_values[axis_idx] = max(min(self.torque_values[axis_idx], max_torque), -max_torque)

                # ===========================
                # 数据记录
                # ===========================
                # 只在第一个轴时记录时间（避免重复）
                if axis_idx == 0:
                    self.time_history.append(loop_time)
                
                # 记录各轴的控制数据
                self.position_history[axis_idx].append(position_rad)
                self.velocity_history[axis_idx].append(velocity_rad_per_sec)
                self.torque_history[axis_idx].append(self.torque_values[axis_idx])
                self.u1_history[axis_idx].append(u1)  # 记录u1值
                self.desired_history[axis_idx].append(desired_rad)
                self.desired_velocity_history[axis_idx].append(desired_vel)
                self.error_history[axis_idx].append(position_error)
                self.velocity_error_history[axis_idx].append(velocity_error)
                self.K1_history[axis_idx].append(K1)  # 记录K1值
                self.K2_history[axis_idx].append(K2)  # 记录K2值
                
                # 更新u1值供下一次循环使用（memory模块功能）
                self.previous_u1_values[axis_idx] = u1

            # ===========================
            # 命令构建和发送
            # ===========================
            # 设置所有轴的位置和力矩命令
            for i in range(7):
                self.base_command.actuators[i].position = raw_positions_deg[i]        # 当前位置
                self.base_command.actuators[i].torque_joint = self.torque_values[i]   # 计算的力矩

            # 更新通信帧ID，确保数据同步
            frame_id = (frame_id + 1) % 65536
            self.base_command.frame_id = frame_id
            for i in range(7):
                self.base_command.actuators[i].command_id = frame_id

            # 发送命令并获取新的反馈
            try:
                self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
            except Exception as err:
                print(f"   ⚠️  警告: 发送命令失败: {err}")

            # 控制循环周期：1ms
            time.sleep(0.001)
            
        # ===========================
        # 控制结束处理：误差为0时的力矩保持阶段
        # ===========================
        print("   ✅ 动力学控制循环完成")
        print("   正在计算零误差力矩并保持0.005秒...")
        
        # 计算每个轴在误差为0时的控制器输出力矩
        zero_error_torques = []
        for axis_idx in range(7):
            # 调用控制器，传入位置误差=0，速度误差=0
            u1_zero, u2_zero = self.controllers[axis_idx].control_law(0.0, 0.0)
            # 限制u2输出
            u2_zero = max(min(u2_zero, self.u2_max_torque), -self.u2_max_torque)
            zero_error_torques.append(u2_zero)
        
        # 保持误差为0时的力矩0.005秒
        zero_torque_start_time = time.time()
        while (time.time() - zero_torque_start_time) < 0.005:
            try:
                # 获取当前状态
                self.base_feedback = self.base_cyclic.RefreshFeedback()
                
                # 设置所有轴的位置和计算的零误差力矩
                for i in range(7):
                    self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                    self.base_command.actuators[i].torque_joint = zero_error_torques[i]
                
                # 更新通信帧ID
                frame_id = (frame_id + 1) % 65536
                self.base_command.frame_id = frame_id
                for i in range(7):
                    self.base_command.actuators[i].command_id = frame_id
                
                # 发送命令
                self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                
            except Exception as err:
                print(f"   ⚠️  警告: 发送零误差力矩命令失败: {err}")
                
            time.sleep(0.001)  # 1ms循环周期
        
        print(f"   ✅ 零误差力矩保持完成")
        print(f"   零误差力矩值: {[f'{t:.3f}' for t in zero_error_torques]} Nm")
        self.control_active = False
        self.control_completed = True  # 标记控制正常完成
        
        # ===========================
        # 立即保存控制结果图表
        # ===========================
        if len(self.time_history) > 0:
            print("   正在生成分析图表...")
            try:
                # 保存独立关节误差分析图
                self.save_individual_joint_plots()
                # 保存综合控制参数分析图
                self.save_comprehensive_analysis_plot()
                print("   ✅ 所有图表已保存完成")
            except Exception as plot_error:
                print(f"   ❌ 图表保存失败: {plot_error}")
                print("   数据已记录，可手动导出分析")
        else:
            print("   ⚠️  无控制数据，跳过图表生成")

        print("   ✅ 控制程序完成，系统准备退出")

    def Stop(self):
        """
        停止7轴力矩控制
        
        安全地终止力矩控制，并生成控制结果的可视化图表。
        """
        # 如果控制已经正常完成，跳过重复处理
        if self.control_completed:
            print("控制已正常完成，跳过重复停止处理")
            return
            
        print(f"停止7轴力矩控制...")
        self.kill_thread = True      # 设置线程终止标志
        self.control_active = False  # 清除控制活动标志
        
        # 等待控制线程结束
        if self.thread:
            self.thread.join()

        # ===========================
        # 获取并显示当前状态
        # ===========================
        try:
            current_feedback = self.base_cyclic.RefreshFeedback()
            current_positions = []
            for i in range(7):
                current_positions.append(current_feedback.actuators[i].position)
            print(f"当前关节位置: {[f'{pos:.2f}°' for pos in current_positions]}")
        except Exception as err:
            print(f"警告: 获取当前位置失败: {err}")

        # ===========================
        # 检查是否需要生成图表（如果控制循环异常终止）
        # ===========================
        if self.control_started and len(self.time_history) > 0 and not self.plot_saved:
            print("控制异常终止，正在保存已记录的数据图表...")
            try:
                self.save_individual_joint_plots()
                self.save_comprehensive_analysis_plot()
            except Exception as plot_error:
                print(f"图表保存失败: {plot_error}")
        elif not self.control_started:
            print("控制未启动，无数据需要保存")

    def save_comprehensive_analysis_plot(self):
        """
        保存综合控制参数分析图表
        
        生成一个大图，每个关节对应一行，包含6列参数：
        - 列1：位置误差
        - 列2：速度误差
        - 列3：u1控制信号
        - 列4：u2控制信号(力矩)
        - 列5：K1自适应参数
        - 列6：K2自适应参数
        """
        if not self.time_history:
            print("没有数据可绘制综合分析图")
            return
        
        # 设置matplotlib为非交互模式
        import matplotlib
        matplotlib.use('Agg')
        plt.ioff()
        
        # 创建文件夹来存储图片
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"kinova_joint_tracking_errors_{timestamp}"
        current_dir = os.getcwd()
        folder_path = os.path.join(current_dir, folder_name)
        
        try:
            os.makedirs(folder_path, exist_ok=True)
        except Exception as e:
            print(f"创建文件夹失败: {e}")
            folder_path = current_dir
        
        # 转换时间数据为numpy数组
        t = np.array(self.time_history)
        
        # 创建大图：7行6列的子图布局 - 每个关节一行，每个参数一列
        fig, axes = plt.subplots(7, 6, figsize=(24, 28))
        fig.suptitle('Comprehensive Individual Joint Control Analysis\n(Each Row = One Joint, Each Column = One Parameter)', 
                     fontsize=18, fontweight='bold')
        
        # 定义列标题
        column_titles = ['Position Error (°)', 'Velocity Error (°/s)', 'Control u1', 'Control u2 (Nm)', 'Adaptive K1', 'Adaptive K2']
        
        # 为每列添加标题
        for col, title in enumerate(column_titles):
            axes[0, col].set_title(title, fontsize=14, fontweight='bold', pad=10)
        
        # 为每个关节(行)生成所有参数图
        for joint_idx in range(7):
            row_color = plt.cm.tab10(joint_idx)  # 每个关节使用不同颜色
            
            # 检查该关节是否有数据
            if len(self.error_history[joint_idx]) == 0:
                # 如果没有数据，在该行显示"No Data"
                for col in range(6):
                    axes[joint_idx, col].text(0.5, 0.5, f'Joint {joint_idx+1}\nNo Data', 
                                             ha='center', va='center', transform=axes[joint_idx, col].transAxes,
                                             fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgray'))
                    axes[joint_idx, col].set_xlim([0, 1])
                    axes[joint_idx, col].set_ylim([0, 1])
                continue
            
            # ===========================
            # 列1：位置误差
            # ===========================
            position_error_rad = np.array(self.error_history[joint_idx])
            position_error_deg = np.degrees(position_error_rad)
            axes[joint_idx, 0].plot(t, position_error_deg, color=row_color, linewidth=1.5, alpha=0.8)
            axes[joint_idx, 0].axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=1)
            axes[joint_idx, 0].grid(True, alpha=0.3)
            axes[joint_idx, 0].set_ylabel(f'Joint {joint_idx+1}', fontsize=11, fontweight='bold')
            
            # ===========================
            # 列2：速度误差
            # ===========================
            velocity_error_rad = np.array(self.velocity_error_history[joint_idx])
            velocity_error_deg = np.degrees(velocity_error_rad)
            axes[joint_idx, 1].plot(t, velocity_error_deg, color=row_color, linewidth=1.5, alpha=0.8)
            axes[joint_idx, 1].axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=1)
            axes[joint_idx, 1].grid(True, alpha=0.3)
            
            # ===========================
            # 列3：u1控制信号
            # ===========================
            u1_values = np.array(self.u1_history[joint_idx])
            axes[joint_idx, 2].plot(t, u1_values, color=row_color, linewidth=1.5, alpha=0.8)
            axes[joint_idx, 2].axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=1)
            axes[joint_idx, 2].grid(True, alpha=0.3)
            
            # ===========================
            # 列4：u2控制信号(力矩)
            # ===========================
            torque_values = np.array(self.torque_history[joint_idx])
            axes[joint_idx, 3].plot(t, torque_values, color=row_color, linewidth=1.5, alpha=0.8)
            axes[joint_idx, 3].axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=1)
            axes[joint_idx, 3].grid(True, alpha=0.3)
            
            # ===========================
            # 列5：K1自适应参数
            # ===========================
            if len(self.K1_history[joint_idx]) > 0:
                K1_values = np.array(self.K1_history[joint_idx])
                axes[joint_idx, 4].plot(t, K1_values, color=row_color, linewidth=1.5, alpha=0.8)
                axes[joint_idx, 4].grid(True, alpha=0.3)
                
                # 添加初始值参考线
                K1_init = self.axis_control_params[joint_idx]["K1_init"]
                axes[joint_idx, 4].axhline(y=K1_init, color='r', linestyle='--', alpha=0.5, linewidth=1)
            
            # ===========================
            # 列6：K2自适应参数
            # ===========================
            if len(self.K2_history[joint_idx]) > 0:
                K2_values = np.array(self.K2_history[joint_idx])
                axes[joint_idx, 5].plot(t, K2_values, color=row_color, linewidth=1.5, alpha=0.8)
                axes[joint_idx, 5].grid(True, alpha=0.3)
                
                # 添加初始值参考线
                K2_init = self.axis_control_params[joint_idx]["K2_init"]
                axes[joint_idx, 5].axhline(y=K2_init, color='r', linestyle='--', alpha=0.5, linewidth=1)
        
        # ===========================
        # 设置底部行的x轴标签
        # ===========================
        for col in range(6):
            axes[6, col].set_xlabel('Time (s)', fontsize=11)
        
        # ===========================
        # 调整字体大小和布局
        # ===========================
        for i in range(7):
            for j in range(6):
                axes[i, j].tick_params(labelsize=9)
        
        # 调整子图间距
        plt.subplots_adjust(left=0.06, bottom=0.04, right=0.98, top=0.93, 
                           wspace=0.25, hspace=0.35)
        
        # 生成文件名并保存
        filename = "comprehensive_individual_joint_analysis.png"
        full_path = os.path.join(folder_path, filename)
        
        try:
            plt.savefig(full_path, dpi=300, bbox_inches='tight', 
                       facecolor='white', edgecolor='none')
            print(f"综合个体关节控制分析图已保存: {filename}")
        except Exception as e:
            print(f"保存综合分析图失败: {e}")
        
        # 关闭图形，释放内存
        plt.close(fig)
        
        # ===========================
        # 输出详细统计摘要
        # ===========================
        print(f"\n{'='*100}")
        print(f"综合个体关节控制分析摘要:")
        print(f"{'='*100}")
        
        # 表头
        print(f"{'关节':<6} {'RMS位置误差(°)':<15} {'RMS速度误差(°/s)':<18} {'平均|u1|':<12} {'平均|u2|(Nm)':<15} {'最终K1':<12} {'最终K2':<12}")
        print("-" * 100)
        
        # 计算各项指标
        for joint_idx in range(7):
            if len(self.error_history[joint_idx]) > 0:
                # 位置和速度误差
                pos_err = np.degrees(np.array(self.error_history[joint_idx]))
                vel_err = np.degrees(np.array(self.velocity_error_history[joint_idx]))
                
                # 控制信号
                u1_vals = np.array(self.u1_history[joint_idx])
                u2_vals = np.array(self.torque_history[joint_idx])
                
                # 自适应参数
                K1_final = self.K1_history[joint_idx][-1] if len(self.K1_history[joint_idx]) > 0 else 0
                K2_final = self.K2_history[joint_idx][-1] if len(self.K2_history[joint_idx]) > 0 else 0
                
                # 计算统计值
                rms_pos = np.sqrt(np.mean(pos_err**2))
                rms_vel = np.sqrt(np.mean(vel_err**2))
                avg_u1 = np.mean(np.abs(u1_vals))
                avg_u2 = np.mean(np.abs(u2_vals))
                
                print(f"{joint_idx+1:<6} {rms_pos:<15.4f} {rms_vel:<18.4f} {avg_u1:<12.4f} {avg_u2:<15.4f} {K1_final:<12.4f} {K2_final:<12.4f}")
            else:
                print(f"{joint_idx+1:<6} {'无数据':<15} {'无数据':<18} {'无数据':<12} {'无数据':<15} {'无数据':<12} {'无数据':<12}")
        
        print(f"\n{'='*100}")
        print(f"说明:")
        print(f"   - 每行对应一个关节，每列对应一个控制参数")
        print(f"   - K1、K2图中虚线表示初始值")
        print(f"   - RMS: 均方根误差，表示整体跟踪精度")
        print(f"   - 最终K1、K2值显示自适应学习结果")
        print(f"{'='*100}")

    def save_individual_joint_plots(self):
        """
        为每个关节单独保存跟踪误差图，每个图包含±0.2度的放大视图
        
        生成7个独立的图形文件，每个文件包含：
        - 主图：完整的跟踪误差曲线
        - 插图：±0.2度范围内的放大视图
        """
        # 检查是否已经保存过图表，避免重复保存
        if self.plot_saved:
            print("图表已保存，跳过重复保存")
            return
            
        if not self.time_history:
            print("没有数据可绘制")
            return
        
        # 设置matplotlib为非交互模式，避免GUI线程警告
        import matplotlib
        matplotlib.use('Agg')  # 使用非GUI后端
        plt.ioff()  # 关闭交互模式
        
        # 创建文件夹来存储图片
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        folder_name = f"kinova_joint_tracking_errors_{timestamp}"
        current_dir = os.getcwd()
        folder_path = os.path.join(current_dir, folder_name)
        
        try:
            os.makedirs(folder_path, exist_ok=True)
            print(f"创建图表保存文件夹: {folder_path}")
        except Exception as e:
            print(f"创建文件夹失败: {e}")
            folder_path = current_dir  # 如果创建失败，使用当前目录
        
        # 转换时间数据为numpy数组
        t = np.array(self.time_history)
        
        # 为每个关节创建独立的图表
        for joint_idx in range(7):
            if len(self.error_history[joint_idx]) == 0:
                print(f"关节 {joint_idx+1} 无数据，跳过")
                continue
                
            # 获取该关节的位置误差数据
            position_error_rad = np.array(self.error_history[joint_idx])
            position_error_deg = np.degrees(position_error_rad)  # 转换为度
            
            # 创建图形和主轴
            fig, ax = plt.subplots(1, 1, figsize=(12, 8))
            
            # ===========================
            # 主图：完整的跟踪误差曲线
            # ===========================
            line_main = ax.plot(t, position_error_deg, 'b-', linewidth=1.5, 
                               label=f'Joint {joint_idx+1} Position Error', alpha=0.8)
            
            # 添加零线
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=1)
            
            # 设置主图标题和标签
            ax.set_title(f'Joint {joint_idx+1} Position Tracking Error with Zoomed Inset (±0.2°)', 
                        fontsize=14, fontweight='bold', pad=20)
            ax.set_xlabel('Time (s)', fontsize=12)
            ax.set_ylabel('Position Error (degrees)', fontsize=12)
            ax.legend(fontsize=11, loc='upper right')
            ax.grid(True, alpha=0.3)
            
            # 设置主图y轴范围，确保能看到所有数据
            error_range = np.max(np.abs(position_error_deg))
            if error_range > 0:
                ax.set_ylim([-error_range*1.1, error_range*1.1])
            
            # ===========================
            # 插图：±0.2度范围的放大视图
            # ===========================
            # 创建插图，位置在右上角
            inset_ax = inset_axes(ax, width="40%", height="40%", loc='upper right', 
                                 bbox_to_anchor=(-0.05, -0.05, 1, 1), bbox_transform=ax.transAxes)
            
            # 在插图中绘制同样的数据
            inset_ax.plot(t, position_error_deg, 'r-', linewidth=1.0, alpha=0.9)
            inset_ax.axhline(y=0, color='k', linestyle='-', alpha=0.4, linewidth=0.8)
            
            # 设置插图的y轴范围为±0.2度
            inset_ax.set_ylim([-0.2, 0.2])
            inset_ax.set_xlim([t[0], t[-1]])  # x轴范围与主图相同
            
            # 设置插图标签和网格
            inset_ax.set_title('Zoomed View (±0.2°)', fontsize=10, fontweight='bold')
            inset_ax.set_xlabel('Time (s)', fontsize=9)
            inset_ax.set_ylabel('Error (°)', fontsize=9)
            inset_ax.grid(True, alpha=0.4)
            inset_ax.tick_params(labelsize=8)
            
            # 添加±0.2度参考线
            inset_ax.axhline(y=0.2, color='orange', linestyle='--', alpha=0.6, linewidth=1)
            inset_ax.axhline(y=-0.2, color='orange', linestyle='--', alpha=0.6, linewidth=1)
            
            # 在主图中标出插图对应的y轴范围
            ax.axhline(y=0.2, color='orange', linestyle='--', alpha=0.4, linewidth=1, 
                      label='±0.2° Reference')
            ax.axhline(y=-0.2, color='orange', linestyle='--', alpha=0.4, linewidth=1)
            
            # 更新主图图例
            ax.legend(fontsize=11, loc='upper left')
            
            # ===========================
            # 添加性能统计信息
            # ===========================
            # 计算基本统计信息
            rms_error = np.sqrt(np.mean(position_error_deg**2))
            max_error = np.max(np.abs(position_error_deg))
            mean_abs_error = np.mean(np.abs(position_error_deg))
            
            # 计算在±0.2度范围内的数据点百分比
            within_02_deg = np.sum(np.abs(position_error_deg) <= 0.2)
            total_points = len(position_error_deg)
            within_02_percent = (within_02_deg / total_points) * 100 if total_points > 0 else 0
            
            # 在图上添加统计信息文本框
            stats_text = f'''Performance Statistics:
RMS Error: {rms_error:.4f}°
Max Error: {max_error:.4f}°
Mean Abs Error: {mean_abs_error:.4f}°
Within ±0.2°: {within_02_percent:.1f}% ({within_02_deg}/{total_points} points)'''
            
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
                   verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
            
            # ===========================
            # 调整布局和保存
            # ===========================
            plt.tight_layout()
            
            # 生成文件名并保存
            filename = f"joint_{joint_idx+1}_tracking_error.png"
            full_path = os.path.join(folder_path, filename)
            
            try:
                plt.savefig(full_path, dpi=300, bbox_inches='tight', 
                           facecolor='white', edgecolor='none')
                print(f"关节 {joint_idx+1} 跟踪误差图已保存: {filename}")
            except Exception as e:
                print(f"保存关节 {joint_idx+1} 图表失败: {e}")
            
            # 关闭图形，释放内存
            plt.close(fig)
        
        # 设置图表已保存标志
        self.plot_saved = True
        
        # ===========================
        # 生成简化的性能统计报告
        # ===========================
        print(f"\n{'='*80}")
        print(f"7关节跟踪误差统计报告:")
        print(f"{'='*80}")
        print(f"图表已保存到文件夹: {folder_name}")
        print()
        
        # 表头
        print(f"{'关节':<6} {'RMS误差(°)':<12} {'最大误差(°)':<12} {'平均误差(°)':<12} {'±0.2°内(%)':<12}")
        print("-" * 60)
        
        # 计算各关节性能指标
        overall_rms = []
        overall_max = []
        overall_mean = []
        overall_within_02 = []
        
        for joint_idx in range(7):
            if len(self.error_history[joint_idx]) > 0:
                # 获取数据
                pos_error_rad = np.array(self.error_history[joint_idx])
                pos_error_deg = np.degrees(pos_error_rad)
                
                # 计算统计指标
                rms_error = np.sqrt(np.mean(pos_error_deg**2))
                max_error = np.max(np.abs(pos_error_deg))
                mean_abs_error = np.mean(np.abs(pos_error_deg))
                
                # 计算在±0.2度范围内的百分比
                within_02_deg = np.sum(np.abs(pos_error_deg) <= 0.2)
                total_points = len(pos_error_deg)
                within_02_percent = (within_02_deg / total_points) * 100 if total_points > 0 else 0
                
                # 输出关节统计结果
                print(f"{joint_idx+1:<6} {rms_error:<12.4f} {max_error:<12.4f} "
                      f"{mean_abs_error:<12.4f} {within_02_percent:<12.1f}")
                
                # 收集整体统计数据
                overall_rms.append(rms_error)
                overall_max.append(max_error)
                overall_mean.append(mean_abs_error)
                overall_within_02.append(within_02_percent)
            else:
                print(f"{joint_idx+1:<6} {'无数据':<12} {'无数据':<12} {'无数据':<12} {'无数据':<12}")
        
        # ===========================
        # 整体性能总结
        # ===========================
        if len(overall_rms) > 0:
            avg_rms = np.mean(overall_rms)
            avg_max = np.mean(overall_max)
            avg_mean = np.mean(overall_mean)
            avg_within_02 = np.mean(overall_within_02)
            
            print(f"\n整体性能总结:")
            print(f"   平均RMS误差: {avg_rms:.4f}°")
            print(f"   平均最大误差: {avg_max:.4f}°")
            print(f"   平均绝对误差: {avg_mean:.4f}°")
            print(f"   平均±0.2°内百分比: {avg_within_02:.1f}%")
            
            # 性能评价
            print(f"\n性能评价:")
            if avg_rms < 0.5:
                print(f"   ✓ RMS误差优秀 (< 0.5°)")
            elif avg_rms < 1.0:
                print(f"   ◐ RMS误差良好 (< 1.0°)")
            else:
                print(f"   ✗ RMS误差需改进 (≥ 1.0°)")
                
            if avg_within_02 > 80:
                print(f"   ✓ 精度控制优秀 (>80%在±0.2°内)")
            elif avg_within_02 > 60:
                print(f"   ◐ 精度控制良好 (>60%在±0.2°内)")
            else:
                print(f"   ✗ 精度控制需改进 (<60%在±0.2°内)")
        
        print(f"\n{'='*80}")
        print(f"说明:")
        print(f"   - 每个关节都有独立的跟踪误差图，包含±0.2°放大视图")
        print(f"   - 综合控制分析图包含所有控制参数变化")
        print(f"   - 图表保存在文件夹: {folder_name}")
        print(f"   - ±0.2°范围显示了高精度跟踪性能")
        print(f"   - 统计信息直接显示在每个图表上")
        print(f"{'='*80}")

# ===========================
# 主程序入口
# ===========================

def main():
    """
    主函数 - 程序入口点
    
    负责程序初始化、用户交互和异常处理。
    提供简单的键盘命令界面控制机械臂。
    
    Returns:
        int: 程序退出代码（0=成功，1=失败）
    """
    # ===========================
    # 命令行参数解析
    # ===========================
    import argparse
    package_dir = os.path.dirname(__file__)
    sys.path.insert(0, find_kortex_examples_dir(package_dir))
    import utilities

    default_config = os.path.join(package_dir, "config", "default_config.json")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default=default_config,
        help="Path to ATS controller JSON config. Use your own copy for robot-specific tuning.",
    )
    args = utilities.parseConnectionArguments(parser)
    runtime_config = load_runtime_config(args.config)

    # ===========================
    # 建立机械臂连接
    # ===========================
    # 建立TCP连接（用于配置和高级命令）和UDP连接（用于实时控制）
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
            # 创建7轴力矩控制器实例
            torque_ctrl = TorqueControlSevenAxis(router, router_real_time, config=runtime_config)

            try:
                # ===========================
                # 程序启动流程
                # ===========================
                print("Kinova Gen3 7轴机械臂力矩控制程序启动（个体关节分析版）")
                print("="*80)
                print("步骤1: 确保机械臂处于0度起始位置...")
                print("   期望跟踪轨迹:")
                for i, config in enumerate(torque_ctrl.axis_configs):
                    print(f"   轴{i+1}: θ_d = {config['offset_deg']:+.1f}° + {config['amplitude_deg']:.1f}°×sin(2π×{config['frequency_hz']:.2f}×t + {config['phase_offset']:.1f})")
                
                print(f"\n个体关节分析功能:")
                print(f"   - 为每个关节生成独立的跟踪误差图")
                print(f"   - 每个图包含±0.2度范围的放大视图")
                print(f"   - 生成个体关节控制分析图：")
                print(f"     * 7行×6列布局：每行=一个关节，每列=一个参数")
                print(f"     * 列1：位置误差 | 列2：速度误差 | 列3：u1信号")
                print(f"     * 列4：u2力矩 | 列5：K1自适应参数 | 列6：K2自适应参数")
                print(f"     * 实时监视每个关节的K1、K2参数变化")
                print(f"     * K1、K2图中包含初始值参考线")
                print("="*80)
                
                # 步骤1：确保机械臂移动到0度起始位置
                print("\n正在移动到0度起始位置（所有关节角度=0°）...")
                if not torque_ctrl.example_move_to_start_position():
                    print("❌ 错误：移动到0度起始位置失败，程序退出")
                    return 1
                
                # 确认当前位置
                try:
                    current_feedback = torque_ctrl.base_cyclic.RefreshFeedback()
                    current_positions = []
                    for i in range(7):
                        current_positions.append(current_feedback.actuators[i].position)
                    print(f"✅ 成功：机械臂已到达起始位置")
                    print(f"   当前关节位置: {[f'{pos:.2f}°' for pos in current_positions]}")
                except Exception as err:
                    print(f"⚠️  警告: 无法获取当前位置: {err}")
                    print(f"✅ 假定已到达0度起始位置")
                
                print("\n" + "="*80)
                print("步骤2: 等待用户启动力矩控制...")
                print("控制说明:")
                print("   按 's' + Enter: 启动力矩初始化和动力学控制")
                print("   按 'q' + Enter: 退出程序")
                print("   Ctrl+C: 紧急停止")
                print("   注意: 控制完成后程序将自动生成个体关节分析图并退出")
                print("="*80)
                
                # ===========================
                # 主控制循环 - 用户交互
                # ===========================
                while True:
                    # 检查用户键盘输入（非阻塞方式）
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.readline().strip().lower()
                        
                        # 处理启动命令
                        if key == 's' and not torque_ctrl.control_active:
                            print("\n" + "="*80)
                            print("步骤3: 启动力矩控制流程...")
                            print("="*80)
                            
                            # 3.1: 力矩控制初始化
                            print("3.1 开始力矩控制初始化...")
                            print("   - 切换到低级伺服模式")
                            print("   - 设置所有执行器为力矩控制模式")
                            print("   - 保持0力矩状态0.005秒稳定")
                            
                            if torque_ctrl.InitTorqueControl():
                                print("✅ 力矩控制初始化成功")
                                
                                # 3.2: 启动动力学控制
                                print("\n3.2 启动动力学控制...")
                                print("   - 创建控制线程")
                                print("   - 开始实时力矩控制循环")
                                print("   - 实时监控K1、K2自适应参数")
                                
                                torque_ctrl.StartTorqueControl()
                                
                                print(f"✅ 动力学控制已启动")
                                print(f"   控制持续时间: {torque_ctrl.run_duration}秒")
                                print(f"   轨迹跟踪: 从0°到带偏移的正弦轨迹")
                                print(f"   数据记录: 位置、速度、力矩、u1、u2、K1、K2")
                                print(f"   图表生成: 独立关节误差图 + 个体关节控制参数图")
                                print("\n🔄 控制运行中，请等待自动完成...")
                                print("="*80)
                            else:
                                print("❌ 力矩控制初始化失败，请检查机械臂状态")
                                print("   建议：")
                                print("   1. 检查机械臂连接状态")
                                print("   2. 确认机械臂未处于错误状态")
                                print("   3. 重新启动程序")
                        
                        # 处理退出命令
                        elif key == 'q':
                            print("\n用户请求退出程序...")
                            # 如果控制正在运行，先停止控制
                            if torque_ctrl.control_active:
                                print("检测到控制正在运行，正在安全停止...")
                                torque_ctrl.Stop()
                            print("程序正在退出...")
                            break
                        
                        # 处理无效命令
                        elif key and key not in ['s', 'q']:
                            print(f"❌ 未知命令: '{key}'")
                            print("📋 可用命令: 's'(开始控制), 'q'(退出)")
                    
                    # 检查控制是否自动完成
                    if torque_ctrl.control_active and torque_ctrl.thread and not torque_ctrl.thread.is_alive():
                        print("\n" + "="*80)
                        print("步骤4: 控制完成，生成分析报告...")
                        print("="*80)
                        torque_ctrl.control_active = False
                        print("✅ 力矩控制完成")
                        print("✅ 独立关节跟踪误差分析图已自动生成")
                        print("✅ 个体关节控制参数分析图已自动生成")
                        print("\n📊 分析报告已保存，程序自动退出")
                        print("="*80)
                        return 0  # 直接退出程序
                    
                    # 短暂休眠，避免CPU占用过高
                    time.sleep(0.1)
                        
            # ===========================
            # 异常处理
            # ===========================
            except KeyboardInterrupt:
                print("\n用户中断（Ctrl+C）")
                if torque_ctrl.control_active:
                    print("正在安全停止控制...")
                    # 先保存图表再停止控制
                    if len(torque_ctrl.time_history) > 0:
                        print("中断前保存控制数据图表...")
                        try:
                            torque_ctrl.save_individual_joint_plots()
                            torque_ctrl.save_comprehensive_analysis_plot()
                        except Exception as plot_err:
                            print(f"图表保存失败: {plot_err}")
                    try:
                        torque_ctrl.Stop()
                    except:
                        pass  # 忽略停止过程中的错误
                    
            except Exception as e:
                print(f"程序异常: {e}")
                import traceback
                traceback.print_exc()  # 打印详细错误信息
                if torque_ctrl.control_active:
                    print("正在安全停止控制...")
                    # 先保存图表再停止控制
                    if len(torque_ctrl.time_history) > 0:
                        print("异常前保存控制数据图表...")
                        try:
                            torque_ctrl.save_individual_joint_plots()
                            torque_ctrl.save_comprehensive_analysis_plot()
                        except Exception as plot_err:
                            print(f"图表保存失败: {plot_err}")
                    try:
                        torque_ctrl.Stop()
                    except:
                        pass  # 忽略停止过程中的错误
                    
            finally:
                # ===========================
                # 清理和退出
                # ===========================
                # 确保控制被正确停止，并最后检查图表保存
                if torque_ctrl.control_active:
                    print("执行最终清理...")
                    # 最后机会保存图表数据
                    if len(torque_ctrl.time_history) > 0:
                        print("最终保存控制数据图表...")
                        try:
                            torque_ctrl.save_individual_joint_plots()
                            torque_ctrl.save_comprehensive_analysis_plot()
                        except Exception as plot_err:
                            print(f"图表保存失败: {plot_err}")
                    try:
                        torque_ctrl.Stop()
                    except:
                        pass  # 忽略停止过程中的错误
                print("程序安全退出")
            
            return 0

# ===========================
# 程序入口检查
# ===========================
if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
