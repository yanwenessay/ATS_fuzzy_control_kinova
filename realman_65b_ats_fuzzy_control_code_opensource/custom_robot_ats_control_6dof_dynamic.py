import sys
import os
import time
import math
import threading
import signal
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import json
import socket
from robot_config import get_robot_connection

__author__ = "WenYan"

# 导入vendor Python SDK（本地目录）
from Robotic_Arm.rm_robot_interface import *
from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t, rm_thread_mode_e


# 导入ATS控制器模块（同目录）
from control_main import control_main

# Windows平台键盘输入支持
try:
    import msvcrt  # Windows
    WINDOWS = True
except ImportError:
    import select  # Linux/Mac非阻塞输入
    WINDOWS = False


def normalize_angle_to_180(angle_deg: float) -> float:
    """
    将角度归一化到[-180, 180]度区间
    处理角度的周期性，确保角度值在标准范围内
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0


class AngleUnwrapper:
    """
    关节角度展开器
    功能：
    1. 处理关节角度的±180度跳变问题
    2. 将角度展开为连续值，便于控制算法使用
    3. 检查角度是否在安全限位内
    """
    
    def __init__(self, soft_limit: float = 170.0):
        """初始化角度展开器
        
        Args:
            soft_limit: 软限位角度值（度），超过此值将触发安全保护
        """
        self.previous_angle = None          # 上一次的角度值
        self.cumulative_offset = 0.0        # 累积的圈数偏移（360度的倍数）
        self.jump_threshold = 180.0         # 跳变检测阈值
        self.soft_limit = soft_limit        # 安全软限位

    def unwrap_angle(self, current_angle_deg: float) -> tuple[float, float, bool]:
        """
        展开角度并检查安全性
        
        Args:
            current_angle_deg: 当前读取的原始角度值（度）
            
        Returns:
            (连续角度, 归一化角度, 是否在安全范围)
            - 连续角度: 展开后的连续角度值，可能超过±180度
            - 归一化角度: [-180, 180]范围内的角度
            - 是否在安全范围: True表示安全，False表示超限
        """
        # 归一化到[-180, 180]
        normalized_current = normalize_angle_to_180(current_angle_deg)
        
        # 首次调用，直接返回
        if self.previous_angle is None:
            self.previous_angle = normalized_current
            return normalized_current, normalized_current, abs(normalized_current) <= self.soft_limit
        
        # 检测是否发生±180度跳变
        diff = normalized_current - self.previous_angle
        if diff < -self.jump_threshold:
            # 从+180跳到-180，实际是正向运动，累加360度
            self.cumulative_offset += 360.0
        elif diff > self.jump_threshold:
            # 从-180跳到+180，实际是负向运动，减去360度
            self.cumulative_offset -= 360.0
        
        # 更新状态
        self.previous_angle = normalized_current
        
        # 计算连续角度值
        continuous = normalized_current + self.cumulative_offset
        
        # 返回：连续角度、归一化角度、安全标志
        return continuous, normalized_current, abs(normalized_current) <= self.soft_limit

    def reset(self):
        """重置展开器状态，用于重新开始控制"""
        self.previous_angle = None
        self.cumulative_offset = 0.0



class JsonControl:
    """使用原生JSON协议进行电流环控制 - 简化版本（使用厂商 JSON 电流环协议）"""
    
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        
    def connect(self):
        """简单的阻塞模式连接"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.ip, self.port))
            print(f"✓ JSON Socket connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ JSON Socket connection failed: {e}")
            if self.sock:
                self.sock.close()
                self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

    def send_request(self, cmd_dict):
        """发送命令并等待响应"""
        if not self.sock:
            return None
        try:
            msg = json.dumps(cmd_dict) + "\r\n"
            self.sock.sendall(msg.encode('utf-8'))
            data = self.sock.recv(4096)
            return json.loads(data.decode('utf-8'))
        except Exception as e:
            print(f"⚠️ JSON Send/Recv error: {e}")
            return None

    def enable_current_mode(self, enable=True):
        """设置电流环控制功能使能状态"""
        return self.send_request({"command": "set_current_canfd_enable", "enable": enable})

    def get_current_mode_state(self):
        """获取电流环控制功能使能状态"""
        return self.send_request({"command": "get_current_canfd_enable"})

    def set_current(self, currents):
        """电流环开放控制 - currents: list of 6 integers (不等待响应)"""
        cmd = {"command": "current_canfd", "current": currents}
        try:
            msg = json.dumps(cmd) + "\r\n"
            self.sock.sendall(msg.encode('utf-8'))
        except Exception as e:
            print(f"⚠️ Send current error: {e}")


class RobotArmController:
    def __init__(self, ip, port, level=3, mode=2):
        # 1. 先初始化JSON电流环控制（TCP控制通道）
        print(f"\n正在初始化JSON电流环控制...")
        self.json_ctrl = JsonControl(ip, port)
        if not self.json_ctrl.connect():
            raise Exception("JSON Socket连接失败")
        
        # 2. 再连接机械臂SDK（仅用于UDP状态监听）
        print(f"\n正在初始化机械臂连接...")
        print(f"  IP: {ip}, Port: {port}, Mode: RM_TRIPLE_MODE_E (UDP支持)")
        print(f"  注意: SDK仅用于状态监听，电流控制通过JSON协议")
        
        self.thread_mode = rm_thread_mode_e(2)  # 强制使用TRIPLE_MODE
        self.robot = RoboticArm(self.thread_mode)
        self.handle = self.robot.rm_create_robot_arm(ip, port, level)
        
        # 控制线程/状态
        self.control_active = False
        self.control_thread = None
        self.shutdown_event = threading.Event()
        
        # ============ UDP实时推送配置 ============
        print(f"\n正在配置UDP实时推送...")
        try:
            push_config = rm_realtime_push_config_t(
                cycle=2,          # 2ms周期 = 500Hz
                enable=True,       # 启用推送
                port=8089,         # UDP端口
                ip=ip              # 目标IP
            )
            
            ret = self.robot.rm_set_realtime_push(push_config)
            if ret == 0:
                print(f"\u2713 UDP实时推送已启用")
                print(f"  - 推送频率: 500Hz (2ms周期)")
                print(f"  - UDP端口: 8089")
                print(f"  - 目标IP: {ip}")
                
                # 验证配置
                ret_check, config = self.robot.rm_get_realtime_push()
                if ret_check == 0 and config.get('enable'):
                    print(f"  - 配置验证: 成功 (周期={config.get('cycle')}ms)")
                else:
                    print(f"  \u26a0️  配置验证失败")
            else:
                print(f"\u26a0️  UDP推送配置失败 (code={ret})，将使用TCP模式")
        except Exception as e:
            print(f"\u26a0️  UDP配置异常: {e}，将使用TCP模式")
        
        # ============ 性能优化：UDP实时数据缓存 ============
        self.udp_realtime_data = None  # UDP回调接收的实时数据
        self.udp_data_lock = threading.Lock()  # 线程安全锁
        self.udp_received_count = 0  # UDP数据接收计数
        self.cached_joint_angles = None
        self.cached_joint_velocities = None
        self.last_read_time = 0.0
        self.read_count = 0
        self.total_read_time = 0.0
        
        # ============ 注册UDP实时推送回调 ============
        self._register_udp_callback()

        # ============ ATS控制参数配置 ============
        self.run_duration = 20.0                # 控制运行时长（秒）
        self.axis_limits = [178, 130, 135, 178, 128, 360]          # 各关节的角度软限位（度）
        self.u1_limit = 10.0                    # 虚拟速度控制量u1的限幅值（度/秒）
        self.max_current_mA = 100000.0          # 电流环的最大电流限制
        self.current_gain = 1.0              # 电流增益：放大ATS输出以驱动机械臂            # 电流环的最大电流限制（mA = 0.5A）【安全降低】
        
        # ============ CANFD电流精度配置 ============
        # 根据JSON协议：
        # 单位 0.001mA (即 1000 units = 1mA)
        # ATS控制器直接输出电流值(mA)，最后按精度换算成整数
        self.current_precision_mA = np.array([
            0.001,  # 关节1: CANFD精度 0.001mA (协议规定J1/J2)  
            0.001,  # 关节2: CANFD精度 0.001mA (协议规定J1/J2)
            0.001,  # 关节3: CANFD精度 0.001mA
            0.001,  # 关节4: CANFD精度 0.001mA
            0.001,  # 关节5: CANFD精度 0.001mA
            0.001,  # 关节6: CANFD精度 0.001mA
        ], dtype=float)

        # ============ 期望轨迹生成参数 ============
        # 轨迹公式：θ_desired(t) = offset + amplitude * sin(2π·frequency·t + phase)
        # 6轴协调运动配置：使用不同频率和相位避免共振
        self.axis_configs = [
            # 关节1: 中心10°, 振幅±10°, 频率0.2Hz
            {"offset_deg": 10.0,  "amplitude_deg": 10.0, "frequency_hz": 0.2, "phase_offset": 0.0},
            # 关节2: 中心0°, 振幅±8°, 频率0.18Hz（略慢），相位差90°
            {"offset_deg": 0.0, "amplitude_deg": 5.0, "frequency_hz": 0.18, "phase_offset": 0.5*np.pi},
            # 关节3: 中心0°, 振幅±6°, 频率0.15Hz（更慢），相位0°
            {"offset_deg": 0.0,  "amplitude_deg": 5.0, "frequency_hz": 0.15, "phase_offset": 0.0},
            # 关节4: 中心0°, 振幅±5°, 频率0.22Hz（略快），相位差90°
            {"offset_deg": 0.0, "amplitude_deg": 5.0, "frequency_hz": 0.22, "phase_offset": 0.5*np.pi},
            # 关节5: 中心0°, 振幅±4°, 频率0.17Hz，相位0°
            {"offset_deg": 0.0,  "amplitude_deg": 5.0, "frequency_hz": 0.17, "phase_offset": 0.0},
            # 关节6: 中心0°, 振幅±8°, 频率0.25Hz（最快），相位差90°
            {"offset_deg": 0.0, "amplitude_deg": 5.0, "frequency_hz": 0.25, "phase_offset": 0.5*np.pi},
        ]

        # ============ ATS自适应模糊控制器参数 ============
        # 每个关节独立配置一个ATS控制器
        # 参数说明：
        #   u1_init, u2_init: 控制量初值
        #   K1_init, K2_init: 初始增益
        #   w1, w2: 学习率
        #   dag_deg: 死区角度
        #   b1, b2: 模糊系统参数
        #   AF11_0, AF12_0, AF21_0, AF22_0: 自适应律初始参数
        self.axis_control_params = [
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0001, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0002, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0002, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0002, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0002, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05, "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.0002, "AF11_0": 0.0, "AF12_0": 0.0, "AF21_0": 0.0, "AF22_0": 0.05},
        ]

        # ============ 初始化6个关节的控制器和角度展开器 ============
        self.controllers = []           # ATS控制器列表
        self.angle_unwrappers = []      # 角度展开器列表
        
        for i in range(6):
            # 创建第i个关节的ATS控制器
            p = self.axis_control_params[i]
            self.controllers.append(control_main(
                u1_init=p["u1_init"], u2_init=p["u2_init"],
                K1_init=p["K1_init"], K2_init=p["K2_init"],
                w1=p["w1"], w2=p["w2"], dag_deg=p["dag_deg"],
                b1=p["b1"], b2=p["b2"], 
                AF11_0=p["AF11_0"], AF12_0=p["AF12_0"], 
                AF21_0=p["AF21_0"], AF22_0=p["AF22_0"]
            ))
            # 创建第i个关节的角度展开器
            self.angle_unwrappers.append(AngleUnwrapper(self.axis_limits[i]))

        # ============ 控制状态变量 ============
        self.previous_u1_values = np.zeros(6)   # 上一时刻的u1值（用于速度估计）
        self.current_mA = np.zeros(6)           # 当前下发的电流值（mA），ATS直接输出
        self.last_actual_current = None         # 最后一次查询到的实际电流
        self.last_deg = None
        self.last_time = None
        
        # ============ 性能监控：控制循环时延统计 ============
        self.timing_stats = {
            'read_joint': [],      # 关节角度读取时间
            'control_compute': [], # 控制律计算时间
            'current_send': [],    # 电流命令发送时间
            'total_loop': []       # 总循环时间
        }
        self.timing_window_size = 100  # 统计窗口大小
        
        self.data_time = []
        self.data_pos_error = []
        self.data_current = []  # ATS控制器命令电流(mA) - 用于内部计算
        self.data_actual_current = []  # 实际关节电流(mA) - 从API获取，用于画图
        self.data_actual_pos = []
        self.data_desired_pos = []
        self.data_actual_vel = []
        
        # ============ 图表更新控制 ============
        self.plot_update_thread = None
        self.last_plot_update_time = 0.0

        if self.handle.id == -1:
            print("\nFailed to connect to the robot arm\n")
            exit(1)
        else:
            print(f"\nSuccessfully connected to the robot arm: {self.handle.id}\n")
    
    def _udp_callback(self, arm_state):
        """
        UDP实时推送回调函数
        接收机械臂实时状态数据（1000Hz）
        
        Args:
            arm_state: rm_realtime_arm_joint_state_t 类型的实时状态数据
        """
        try:
            with self.udp_data_lock:
                self.udp_realtime_data = arm_state
                self.udp_received_count += 1
                
                # 每5000次打印一次统计信息（约5秒）
                if self.udp_received_count % 5000 == 0:
                    print(f"📡 UDP实时推送: 已接收 {self.udp_received_count} 个数据包")
        except Exception as e:
            print(f"⚠️ UDP回调异常: {e}")
    
    def _register_udp_callback(self):
        """注册UDP实时推送回调函数"""
        try:
            # 动态获取回调类型（运行时从已导入的模块中获取）
            import sys
            rm_module = sys.modules.get('Robotic_Arm.rm_ctypes_wrap')
            if rm_module is None:
                import Robotic_Arm.rm_ctypes_wrap as rm_module
            
            callback_type = getattr(rm_module, 'rm_realtime_arm_state_callback_ptr')
            
            # 创建回调函数（必须保持引用以防被垃圾回收）
            self._udp_callback_func = callback_type(self._udp_callback)
            self.robot.rm_realtime_arm_state_call_back(self._udp_callback_func)
            
            print(f"✓ UDP回调函数已注册，等待数据...")
            time.sleep(0.5)  # 等待第一个数据包
            
            if self.udp_received_count > 0:
                print(f"✓ UDP数据接收正常 (已收到 {self.udp_received_count} 个数据包)")
            else:
                print(f"⚠️  警告: 尚未接收到UDP数据包，请检查网络和防火墙设置")
                
        except Exception as e:
            print(f"⚠️  UDP回调注册失败: {e}")
            import traceback
            traceback.print_exc()
            print(f"   将降级使用TCP轮询模式")

    def disconnect(self):
        if self.control_active:
            self.stop_ats_control()
        self.robot.rm_delete_robot_arm()

    def movej(self, joint, v=20, r=0, connect=0, block=1):
        return self.robot.rm_movej(joint, v, r, connect, block)

    def _compute_desired_traj(self, axis_idx: int, t: float) -> tuple[float, float]:
        cfg = self.axis_configs[axis_idx]
        offset = math.radians(cfg["offset_deg"]) 
        amp = math.radians(cfg["amplitude_deg"]) 
        omega = 2.0 * math.pi * cfg["frequency_hz"]
        phase = cfg["phase_offset"]
        pos = offset + amp * math.sin(omega * t + phase)
        vel = amp * omega * math.cos(omega * t + phase)
        max_lim = math.radians(self.axis_limits[axis_idx] - 5.0)
        pos = float(np.clip(pos, -max_lim, max_lim))
        return pos, vel

    def _read_joint_deg(self) -> list[float]:
        """
        读取关节角度（优先使用UDP实时推送数据）
        
        使用UDP回调接收的数据可以获得真正的实时性（1000Hz推送）
        如果UDP数据不可用，降级到TCP轮询模式
        """
        read_start = time.perf_counter()
        ret = -1  # 初始化返回码
        
        try:
            # ============ 方法1: 优先使用UDP实时推送数据 ============
            with self.udp_data_lock:
                if self.udp_realtime_data is not None:
                    # 从UDP推送数据中提取关节角度和速度
                    udp_data = self.udp_realtime_data
                    
                    # UDP数据结构：udp_data.joint_status.joint_position（角度）
                    #             udp_data.joint_status.joint_speed（速度）
                    try:
                        # 提取关节角度（Python SDK已处理精度，直接就是度）
                        joint_angles = list(udp_data.joint_status.joint_position)[:7]
                        
                        # 提取关节速度（Python SDK已处理精度，直接就是度/秒）
                        joint_velocities = list(udp_data.joint_status.joint_speed)[:7]
                        
                        # 缓存速度数据供其他函数使用
                        self.cached_joint_velocities = joint_velocities
                        
                        # 性能统计
                        read_time = time.perf_counter() - read_start
                        self.read_count += 1
                        self.total_read_time += read_time
                        
                        # 每1000次打印一次平均耗时
                        if self.read_count % 1000 == 0:
                            avg_time = self.total_read_time / self.read_count * 1000
                            print(f"📊 [UDP模式] 数据读取: {avg_time:.3f}ms/次 (共{self.read_count}次, UDP包数:{self.udp_received_count})")
                        
                        return joint_angles
                    except AttributeError as e:
                        # UDP数据结构访问失败，打印详细错误（仅首次）
                        if self.read_count == 0:
                            print(f"⚠️ UDP数据结构访问错误: {e}")
                            print(f"   可用字段: {dir(udp_data)}")
                        # 继续降级到TCP模式
            
            # ============ 方法2: UDP数据不可用时，降级到TCP模式 ============
            # 优先使用 rm_get_current_arm_state（一次读取角度+速度）
            ret, state = self.robot.rm_get_current_arm_state()
            if ret == 0 and isinstance(state, dict):
                deg_raw = state.get('joint', None)
                vel_raw = state.get('joint_speed', None)
                
                # 缓存速度数据（Python SDK已处理精度，直接就是度/秒）
                if vel_raw is not None:
                    self.cached_joint_velocities = vel_raw[:7] if len(vel_raw) >= 7 else vel_raw + [0.0]*(7-len(vel_raw))
                
                if deg_raw is not None:
                    # Python SDK已处理精度，直接就是度
                    deg = deg_raw
                    if len(deg) < 7:
                        deg = deg + [0.0] * (7 - len(deg))
                    
                    # 性能统计
                    read_time = time.perf_counter() - read_start
                    self.read_count += 1
                    self.total_read_time += read_time
                    
                    # 每1000次打印一次
                    if self.read_count % 1000 == 0:
                        avg_time = self.total_read_time / self.read_count * 1000
                        print(f"📊 [TCP模式] 数据读取: {avg_time:.3f}ms/次 (共{self.read_count}次) ⚠️ UDP不可用")
                    
                    return deg[:7]
            
            # 降级：使用独立接口
            ret, deg = self.robot.rm_get_joint_degree()
            if ret == 0 and deg:
                if len(deg) < 7:
                    deg = deg + [0.0] * (7 - len(deg))
                return deg[:7]
                
        except Exception as e:
            print(f"⚠️ 读取关节角度异常: {e}")
            import traceback
            if self.read_count < 5:  # 仅打印前5次的详细traceback
                traceback.print_exc()
            
        raise RuntimeError(f"获取关节角度失败, ret={ret}, 所有方法均不可用")
    
    def get_continuous_positions(self):
        """
        获取所有关节的连续位置角度
        
        处理从机械臂获取的原始角度数据，通过角度连续性处理器
        消除360度跳跃，获得连续的角度值。
        
        Returns:
            tuple: (连续位置列表, 归一化位置列表, 原始位置列表, 是否全部在安全范围内)
        """
        continuous_positions = []   # 连续角度值
        normalized_positions = []   # 归一化角度值
        raw_positions = []          # 原始角度值
        all_within_limits = True    # 安全状态标志
        
        # 读取原始角度
        raw_deg = self._read_joint_deg()
        
        # 处理每个关节的角度数据
        for i in range(6):
            raw_position_deg = raw_deg[i]
            raw_positions.append(raw_position_deg)
            
            # 通过角度处理器获取连续角度和安全状态
            continuous_angle, normalized_angle, is_within_limits = self.angle_unwrappers[i].unwrap_angle(raw_position_deg)
            
            continuous_positions.append(continuous_angle)
            normalized_positions.append(normalized_angle)
            
            # 检查是否有关节超出安全限制
            if not is_within_limits:
                all_within_limits = False
        
        return continuous_positions, normalized_positions, raw_positions, all_within_limits


    def _convert_current_to_canfd_units(self, current_mA: np.ndarray) -> list:
        """
        将电流值(mA)转换为CANFD协议单位（使用四舍五入保证精度）
        
        根据JSON协议:
        - 关节1和关节2: 单位是 0.002mA (即发送值 = round(电流mA / 0.002))
        - 关节3-6: 单位是 0.001mA (即发送值 = round(电流mA / 0.001))
        
        使用round()而非int()，避免截断误差导致的阶梯效应
        
        Args:
            current_mA: 6个关节的电流值(mA)，浮点精度
            
        Returns:
            转换后的整数列表，用于JSON协议发送
        """
        canfd_values = []
        for i in range(6):
            precision = self.current_precision_mA[i]  # 0.002 或 0.001
            # 使用round四舍五入，避免int截断导致的精度损失
            canfd_values.append(round(current_mA[i] / precision))
        return canfd_values

    def _estimate_velocity_rad(self, deg_now: np.ndarray, t_now: float) -> np.ndarray:
        """
        使用后向差分估计关节角速度
        
        Args:
            deg_now: 当前关节角度（度）
            t_now: 当前时间戳
            
        Returns:
            关节角速度（弧度/秒）
        """
        if self.last_deg is None:
            self.last_deg = deg_now.copy()
            self.last_time = t_now
            return np.zeros(6)
        
        # 计算时间间隔，避免除零
        dt = max(1e-4, t_now - self.last_time)
        
        # 计算角度变化率（度/秒）
        vel_deg = (deg_now - self.last_deg) / dt
        
        # 更新上一次的值
        self.last_deg = deg_now.copy()
        self.last_time = t_now
        
        # 限制速度范围，防止异常值（根据实际机械臂性能调整）
        vel_deg = np.clip(vel_deg, -300.0, 300.0)
        
        # 转换为弧度/秒
        return np.deg2rad(vel_deg)

    def start_ats_control(self, freq_hz: float = 200.0) -> bool:
        # 使用JSON协议启用电流环（使用 JSON 电流环协议）
        print(f"\n>>> 启用电流环控制...")
        resp = self.json_ctrl.enable_current_mode(True)
        print(f"启用响应: {resp}")
        
        # 验证电流环是否真正启用
        state_resp = self.json_ctrl.get_current_mode_state()
        print(f"模式状态: {state_resp}")
        
        if not state_resp or not state_resp.get('set_state'):
            print(f"❌ 启用电流模式失败")
            return False
        
        print(f"✓ 电流模式已启用，开始控制循环...")

        for au in self.angle_unwrappers:
            au.reset()
        self.previous_u1_values[:] = 0.0
        self.current_mA[:] = 0.0
        self.last_deg = None
        self.last_time = None
        
        self.data_time.clear()
        self.data_pos_error.clear()
        self.data_current = []  # 命令电流 - 内部计算
        self.data_actual_current = []  # 实际电流 - API获取，用于画图
        self.data_actual_pos.clear()
        self.data_desired_pos.clear()
        self.data_actual_vel.clear()
        
        self.plot_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # 使用脚本所在目录的绝对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.plot_folder = os.path.join(script_dir, f"plots_{self.plot_timestamp}")
        print(f"�� 图表保存目录: {self.plot_folder}")
        os.makedirs(self.plot_folder, exist_ok=True)
        # print(f"✓ Plots will be saved to: {os.path.abspath(self.plot_folder)}")  # 已在上面打印
        
        # 启用matplotlib交互模式以支持平滑的实时更新
        plt.ion()

        self.control_active = True
        self.control_thread = threading.Thread(target=self._ats_loop, args=(freq_hz,), daemon=True)
        self.control_thread.start()
        
        # 启动独立的图表更新线程（1Hz更新）
        # 非daemon线程，确保Ctrl+C时也能保存图表
        self.plot_update_thread = threading.Thread(target=self._plot_update_loop, daemon=False)
        self.plot_update_thread.start()
        
        print(f"ATS控制已启动，频率: {freq_hz} Hz")
        print(f"图表更新频率: 1 Hz (每秒更新)")
        return True

    def _ats_loop(self, freq_hz: float):
        period = 1.0 / float(freq_hz)
        start_t = time.time()
        cycle_count = 0
        error_count = 0
        
        # 时延统计变量
        read_times = []
        compute_times = []
        send_times = []
        loop_times = []
        
        print(f"\n>>> 控制循环启动: 频率={freq_hz}Hz, 周期={period*1000:.2f}ms")
        print(f">>> 正在测试通信延迟...\n")
        
        # 预热：测试通信延迟
        test_times = []
        for _ in range(10):
            t0 = time.perf_counter()
            try:
                self._read_joint_deg()
                test_times.append(time.perf_counter() - t0)
            except:
                pass
        
        if test_times:
            avg_comm_time = sum(test_times) / len(test_times) * 1000
            max_comm_time = max(test_times) * 1000
            print(f"📊 通信延迟测试: 平均={avg_comm_time:.2f}ms, 最大={max_comm_time:.2f}ms")
            print(f"📊 理论最大频率: {1000/avg_comm_time:.0f}Hz (基于通信时间)\n")
        
        try:
            while self.control_active and not self.shutdown_event.is_set():
                loop_start = time.perf_counter()  # 记录循环开始时间
                loop_t0 = time.time()
                t = loop_t0 - start_t
                
                # 每5秒打印一次时间进度
                if cycle_count % int(freq_hz * 5) == 0 and cycle_count > 0:
                    print(f"⏱️ 运行进度: {t:.1f}s / {self.run_duration}s")
                
                # 在19.95秒时保存当前实际位置
                if t >= (self.run_duration - 0.05) and self.final_position_deg is None:
                    self.final_position_deg = normalized_deg.copy()
                    print(">>> 19.95秒保存当前位置:", [f"{deg:.2f}°" for deg in self.final_position_deg])
                
                if t >= self.run_duration:
                    print(f"\n>>> ATS控制达到预设时长 {self.run_duration}s，自动停止")
                    break
                
                try:
                    # ========== 1. 读取关节角度（计时） ==========
                    read_start = time.perf_counter()
                    continuous_deg, normalized_deg, raw_deg, safe_ok = self.get_continuous_positions()
                    read_elapsed = (time.perf_counter() - read_start) * 1000  # ms
                    read_times.append(read_elapsed)
                    
                    cont_deg = np.array(continuous_deg, dtype=float)

                    # 使用连续角度估计速度（保证速度连续性）
                    vel_rad = self._estimate_velocity_rad(cont_deg, loop_t0)
                    pos_rad = np.deg2rad(cont_deg)

                    if not safe_ok:
                        print(f"⚠️ [周期{cycle_count}] 关节角度超出安全限位，输出零电流")
                        self.current_mA[:] = 0.0
                        self.json_ctrl.set_current(self._convert_current_to_canfd_units(self.current_mA))
                        time.sleep(period)
                        continue


                    # ========== 2. 控制律计算（计时） ==========
                    compute_start = time.perf_counter()
                    
                    desired_pos = np.zeros(6)
                    desired_vel = np.zeros(6)
                    for i in range(6):
                        desired_pos[i], desired_vel[i] = self._compute_desired_traj(i, t)

                    pos_errors = np.zeros(6)
                    
                    # 所有6个关节都执行ATS控制律
                    for i in range(6):
                        x1 = pos_rad[i] - desired_pos[i]
                        x2 = vel_rad[i] - (desired_vel[i] + self.previous_u1_values[i])
                        pos_errors[i] = x1
                        
                        # ATS控制器直接输出电流值(mA)
                        u1, u2 = self.controllers[i].control_law(x1, x2)
                        u1 = float(np.clip(u1, -self.u1_limit, self.u1_limit))
                        self.previous_u1_values[i] = u1

                        # u2直接就是电流值(mA)，无需额外转换
                        # 只需在发送时按精度转换：J1-2除以0.002，J3-6除以0.001
                        current_val = u2 * self.current_gain  # 应用电流增益
                        # 保存调试信息（仅关节1）
                        if i == 0:
                            self._debug_u2_raw = u2
                            self._debug_current_before_clip = current_val
                        # 限幅后存储
                        self.current_mA[i] = float(np.clip(current_val, -self.max_current_mA, self.max_current_mA))
                    
                    compute_elapsed = (time.perf_counter() - compute_start) * 1000  # ms
                    compute_times.append(compute_elapsed)
                    
                    # ========== 3. 电流命令发送（计时） ==========
                    send_start = time.perf_counter()
                    rc = self.json_ctrl.set_current(self._convert_current_to_canfd_units(self.current_mA))
                    send_elapsed = (time.perf_counter() - send_start) * 1000  # ms
                    send_times.append(send_elapsed)
                    
                    if rc is not None and rc != 0:
                        error_count += 1
                        if error_count % 100 == 1:
                            print(f"⚠️ 电流下发失败, code={rc}")
                    else:
                        error_count = 0
                    
                    # ========== 4. 记录总循环时间 ==========
                    loop_elapsed = (time.perf_counter() - loop_start) * 1000  # ms
                    loop_times.append(loop_elapsed)
                    
                    # ========== 5. 定期打印时延统计 ==========
                    if cycle_count % 500 == 0 and cycle_count > 0:
                        # 计算最近100个周期的统计数据
                        window_size = min(100, len(read_times))
                        recent_read = read_times[-window_size:]
                        recent_compute = compute_times[-window_size:]
                        recent_send = send_times[-window_size:]
                        recent_loop = loop_times[-window_size:]
                        
                        print(f"\n[{cycle_count:6d}] ⏱️  控制循环时延统计 (最近{window_size}个周期):")
                        print(f"  📖 读取角度:   平均={np.mean(recent_read):.3f}ms, 最大={np.max(recent_read):.3f}ms")
                        print(f"  🧮 控制计算:   平均={np.mean(recent_compute):.3f}ms, 最大={np.max(recent_compute):.3f}ms")
                        print(f"  📤 发送电流:   平均={np.mean(recent_send):.3f}ms, 最大={np.max(recent_send):.3f}ms")
                        print(f"  🔄 总循环时间: 平均={np.mean(recent_loop):.3f}ms, 最大={np.max(recent_loop):.3f}ms")
                        print(f"  🎯 目标周期:   {period*1000:.2f}ms (实际频率: {1000/np.mean(recent_loop):.1f}Hz)")
                        print(f"  ⚡ 关节1状态:  I={self.current_mA[0]:+8.3f}mA → CANFD={round(self.current_mA[0]/self.current_precision_mA[0])}")
                    
                    # ========== 6. 每秒打印角度误差和电流输出（诊断电流环问题） ==========
                    if cycle_count % int(freq_hz) == 0 and cycle_count > 0:
                        print(f"\n[🔍诊断 {t:.1f}s] 关节1实时状态:")
                        print(f"  📐 位置 (度制): 期望={np.rad2deg(desired_pos[0]):+7.3f}°, 实际={cont_deg[0]:+7.3f}°, 误差={np.rad2deg(pos_errors[0]):+8.4f}°")
                        print(f"  📐 位置 (弧度): 期望={desired_pos[0]:+7.4f}rad, 实际={pos_rad[0]:+7.4f}rad, 误差(x1)={pos_errors[0]:+8.5f}rad")
                        print(f"  🚀 速度 (度制): 期望={np.rad2deg(desired_vel[0]):+7.3f}°/s, 实际={np.rad2deg(vel_rad[0]):+7.3f}°/s")
                        print(f"  🚀 速度 (弧度): 期望={desired_vel[0]:+7.4f}rad/s, 实际={vel_rad[0]:+7.4f}rad/s")
                        print(f"  🎯 控制误差: x1(位置)={pos_errors[0]:+8.5f}rad, x2(速度)={vel_rad[0] - (desired_vel[0] + self.previous_u1_values[0]):+8.5f}rad/s")
                        # 临时显示原始u2值（力矩）和转换后的电流值
                        if hasattr(self, '_debug_u2_raw') and hasattr(self, '_debug_current_before_clip'):
                            print(f"  ⚡ 控制输出: u1={self.previous_u1_values[0]:+7.4f}, u2(电流)={self._debug_u2_raw:+8.4f}mA")
                            print(f"  🔄 电流值: {self._debug_current_before_clip:+8.4f}mA (限幅前)")
                            print(f"  ✂️  限幅后电流: {self.current_mA[0]:+8.4f}mA (限制: ±{self.max_current_mA}mA)")
                        else:
                            print(f"  ⚡ 控制输出: u1={self.previous_u1_values[0]:+7.4f}, u2(电流)={self.current_mA[0]:+8.4f}mA")
                        print(f"  🔌 CANFD值: {round(self.current_mA[0]/self.current_precision_mA[0])} (精度={self.current_precision_mA[0]}mA)")
                        
                        # 查询实际关节电流（用于验证电流环控制效果）
                        try:
                            ret_cur, actual_currents = self.robot.rm_get_current_joint_current()
                            if ret_cur == 0 and actual_currents:
                                print(f"  🔋 实际关节电流: J1={actual_currents[0]:+8.3f}mA (命令={self.current_mA[0]:+8.3f}mA, 差值={actual_currents[0]-self.current_mA[0]:+8.3f}mA)")
                                # 打印所有6个关节的实际电流
                                current_str = ", ".join([f"J{i+1}={actual_currents[i]:+7.3f}mA" for i in range(min(6, len(actual_currents)))])
                                print(f"  🔋 全部关节实际电流: {current_str}")
                                # 保存实际电流到画图数据（每1秒采样一次，与诊断输出同步）
                                self.last_actual_current = actual_currents[:6]  # 保存到临时变量
                            else:
                                print(f"  ⚠️  查询关节电流失败 (ret={ret_cur})")
                        except Exception as e:
                            print(f"  ⚠️  查询关节电流异常: {e}")
                        
                        
# 检查电流输出是否平滑（不应出现阶梯效应）
                        if len(self.data_current) > int(freq_hz):
                            recent_currents = np.array(self.data_current[-int(freq_hz):])[:, 0]
                            current_std = np.std(recent_currents)
                            current_mean = np.mean(recent_currents)
                            print(f"  📊 最近1秒电流: 均值={current_mean:+7.4f}mA, 标准差={current_std:.4f}mA")
                    
                    # 每个周期采集数据（1000Hz）
                    self.data_time.append(t)
                    self.data_pos_error.append(np.rad2deg(pos_errors.copy()))
                    self.data_current.append(self.current_mA.copy())  # 命令电流
                    # 保存实际电流（如果有最新查询结果）
                    if hasattr(self, 'last_actual_current') and self.last_actual_current:
                        self.data_actual_current.append(self.last_actual_current.copy())
                    else:
                        self.data_actual_current.append([0.0] * 6)  # 如果还没查询到，填充零
                    self.data_actual_pos.append(cont_deg.copy())
                    self.data_desired_pos.append(np.rad2deg(desired_pos.copy()))
                    self.data_actual_vel.append(np.rad2deg(vel_rad.copy()))
                    
                    cycle_count += 1
                    
                except Exception as e:
                    print(f"⚠️ 控制循环内部错误: {e}")
                    self.json_ctrl.set_current([0, 0, 0, 0, 0, 0])
                    time.sleep(period)
                    continue

                # 精确维持频率
                elapsed = time.time() - loop_t0
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif elapsed > period * 3.0:  # 循环时间过长警告（放宽到3倍）
                    if cycle_count % 500 == 0:
                        print(f"⚠️ 控制循环耗时过长: {elapsed*1000:.1f}ms (目标: {period*1000:.1f}ms)")
                        
        except Exception as e:
            print(f"❌ 控制循环异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 打印最终统计
            if len(loop_times) > 0:
                print(f"\n{'='*70}")
                print(f"📊 控制循环最终统计 (总周期数: {cycle_count}):")
                print(f"  📖 读取角度:   平均={np.mean(read_times):.3f}ms, 最大={np.max(read_times):.3f}ms, 最小={np.min(read_times):.3f}ms")
                print(f"  🧮 控制计算:   平均={np.mean(compute_times):.3f}ms, 最大={np.max(compute_times):.3f}ms, 最小={np.min(compute_times):.3f}ms")
                print(f"  📤 发送电流:   平均={np.mean(send_times):.3f}ms, 最大={np.max(send_times):.3f}ms, 最小={np.min(send_times):.3f}ms")
                print(f"  🔄 总循环时间: 平均={np.mean(loop_times):.3f}ms, 最大={np.max(loop_times):.3f}ms, 最小={np.min(loop_times):.3f}ms")
                print(f"  🎯 实际频率:   {1000/np.mean(loop_times):.1f}Hz (目标: {freq_hz}Hz)")
                print(f"  ⚠️  超时次数:   {sum(1 for t in loop_times if t > period*1000*1.5)} 次 (>{period*1000*1.5:.1f}ms)")
                
                # 性能瓶颈诊断
                avg_send = np.mean(send_times)
                avg_read = np.mean(read_times)
                avg_compute = np.mean(compute_times)
                
                print(f"\n💡 性能瓶颈分析:")
                if avg_send > 10.0:
                    print(f"  ⚠️  电流发送时延过高 ({avg_send:.1f}ms) - 这是主要瓶颈")
                    print(f"     建议: 1) 联系厂商确认是否有非阻塞API")
                    print(f"           2) 使用更低控制频率 (建议: {max(10, int(1000/(avg_send*1.5)))}Hz)")
                    print(f"           3) 检查网络延迟和CANFD总线负载")
                
                # 性能评估
                actual_freq = 1000 / np.mean(loop_times)
                if actual_freq >= freq_hz * 0.95:  # 实际频率达到目标的95%以上
                    print(f"\n✅ 系统性能良好:")
                    print(f"   - 实际频率 {actual_freq:.1f}Hz ≥ 目标 {freq_hz}Hz ✓")
                    print(f"   - UDP读取延迟 {avg_read:.3f}ms (优秀)")
                    print(f"   - 控制周期稳定，当前配置为最优方案")
                if avg_read > 5.0:
                    print(f"  ⚠️  角度读取慢 ({avg_read:.1f}ms) - UDP可能未工作")
                    print(f"     建议: 检查UDP推送配置和防火墙")
                if avg_compute > 5.0:
                    print(f"  ⚠️  控制计算慢 ({avg_compute:.1f}ms)")
                    print(f"     建议: 优化控制算法或使用更快的硬件")
                
                print(f"{'='*70}\n")
            
            # 停止前发零电流
            print(f"\n控制循环结束，总周期数: {cycle_count}")
            try:
                self.json_ctrl.set_current([0, 0, 0, 0, 0, 0])
                time.sleep(0.01)
                self.json_ctrl.set_current([0, 0, 0, 0, 0, 0])  # 双重保险
            except Exception:
                pass
                
            # 等待图表更新线程结束
            if self.plot_update_thread and self.plot_update_thread.is_alive():
                time.sleep(1.5)  # 等待最后一次更新
            
            # 【强制保存】最终图表（无论如何都保存）
            print(f"\n>>> 正在保存图表...")
            if len(self.data_time) > 0:
                try:
                    self._save_plots_internal()
                    print(f"✓ 所有图表已保存到: {os.path.abspath(self.plot_folder)}")
                    print(f"  - 角度跟踪图、误差图、速度图、力矩图、电流图")
                except Exception as e:
                    print(f"⚠️ 图表保存失败: {e}")
            else:
                print(f"⚠️ 无数据可保存（控制时长过短）")

    def _plot_update_loop(self):
        """图表更新循环（1Hz更新，避免IO阻塞）"""
        print(">>> 图表更新线程已启动 (1Hz)\n")
        update_interval = 1.0  # 每秒更新一次
        last_update = time.time()
        
        while self.control_active and not self.shutdown_event.is_set():
            try:
                current_time = time.time()
                # 每秒更新一次图表
                if (current_time - last_update) >= update_interval and len(self.data_time) > 10:
                    self._save_plots_internal()
                    last_update = current_time
                    # 每10秒打印一次保存确认
                    if int(current_time) % 10 == 0:
                        print(f"📊 [实时保存] 图表已更新 (数据点: {len(self.data_time)})")
                
                # 休眠100ms，降低CPU占用
                time.sleep(0.1)
                
            except Exception as e:
                print(f"⚠️ 图表更新错误: {e}")
                time.sleep(1.0)
        
        # 线程结束前最后保存一次（确保Ctrl+C也能保存）
        if len(self.data_time) > 0:
            try:
                print(">>> 图表更新线程正在保存最终图表...")
                self._save_plots_internal()
                print(f"✓ 图表已保存到: {self.plot_folder}")
            except Exception as e:
                print(f"⚠️ 最终图表保存失败: {e}")
        print("✓ 图表更新线程已结束")
    
    def stop_ats_control(self):
        """
        停止ATS控制并安全退出电流环模式（退出电流环控制）
        
        退出逻辑：
        1. 设置停止标志
        2. 直接禁用电流环（位置环会自动接管并保持当前位置）
        3. 不需要movej或零电流（位置环自动工作）
        """
        if not self.control_active:
            return
        
        print("\n>>> 停止ATS控制...")
        self.control_active = False
        
        try:
            # 【关键】直接禁用电流环（参考控制器协议）
            # 禁用后，机械臂会自动切换回位置环模式，位置环会保持当前位置
            print(">>> 禁用电流环（位置环将自动接管）...")
            self.json_ctrl.enable_current_mode(False)
            print("✓ 电流环已禁用，机械臂由位置环保持当前位置")
            
        except Exception as e:
            print(f"⚠️ 停止控制异常: {e}")
        
        # 【强制保存】停止时保存图表
        if len(self.data_time) > 0:
            print(f"\n>>> 正在保存图表...")
            try:
                self._save_plots_internal()
                print(f"✓ 所有图表已保存")
            except Exception as e:
                print(f"⚠️ 图表保存失败: {e}")
        
        print("✓ ATS控制已停止\n")
    

    def _save_plots_internal(self):
        """Internal method to save plots to pre-created folder during control"""
        if len(self.data_time) == 0:
            return

        try:
            # Convert to numpy arrays for processing
            time_arr = np.array(self.data_time)
            pos_err_arr = np.array(self.data_pos_error)
            cmd_current_arr = np.array(self.data_current)
            actual_current_arr = np.array(self.data_actual_current)
            actual_pos_arr = np.array(self.data_actual_pos)
            actual_vel_arr = np.array(self.data_actual_vel)
            desired_pos_arr = np.array(self.data_desired_pos)

            # Use default fonts (English)
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
            plt.rcParams['axes.unicode_minus'] = False

            joint_colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']

            # ========== Figure 1: All Joints Tracking Error ==========
            fig1, axes1 = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
            axes1 = axes1.flatten()
            for joint_idx, ax in enumerate(axes1):
                joint_error = pos_err_arr[:, joint_idx]
                abs_error = np.abs(joint_error)
                ax.plot(time_arr, joint_error, color=joint_colors[joint_idx], linewidth=1.8, alpha=0.95)
                ax.axhline(y=0.0, color='k', linestyle='--', linewidth=0.9, alpha=0.45)
                ax.set_title(f'Joint {joint_idx + 1} Tracking Error', fontsize=12, fontweight='bold')
                ax.set_ylabel('Error (deg)', fontsize=10)
                ax.grid(True, alpha=0.3, linestyle='--')
                ax.tick_params(labelsize=9)
                stats_text = (
                    f'Mean|e|: {np.mean(abs_error):.3f}\n'
                    f'Max|e|: {np.max(abs_error):.3f}\n'
                    f'Std: {np.std(joint_error):.3f}'
                )
                ax.text(
                    0.02,
                    0.98,
                    stats_text,
                    transform=ax.transAxes,
                    fontsize=8.5,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.78)
                )
            for ax in axes1[-2:]:
                ax.set_xlabel('Time (s)', fontsize=10)
            fig1.suptitle('All Joints: Tracking Error Overview', fontsize=16, fontweight='bold')
            fig1.tight_layout(rect=[0, 0.02, 1, 0.97])
            filename1 = os.path.join(self.plot_folder, '1_All_Joints_Tracking_Error.png')
            fig1.savefig(filename1, dpi=150, bbox_inches='tight')
            plt.close(fig1)

            # ========== Figure 2: All Joints u2 Output ==========
            fig2, axes2 = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
            axes2 = axes2.flatten()
            for joint_idx, ax in enumerate(axes2):
                joint_u2 = cmd_current_arr[:, joint_idx]
                max_abs_u2 = np.max(np.abs(joint_u2)) if len(joint_u2) > 0 else 0.0
                ax.plot(time_arr, joint_u2, color=joint_colors[joint_idx], linewidth=1.8, alpha=0.95)
                ax.axhline(y=self.max_current_mA, color='r', linestyle='--', linewidth=1.0, alpha=0.4)
                ax.axhline(y=-self.max_current_mA, color='r', linestyle='--', linewidth=1.0, alpha=0.4)
                ax.axhline(y=0.0, color='k', linestyle='-', linewidth=0.8, alpha=0.25)
                ax.set_title(f'Joint {joint_idx + 1} Control Torque / u2', fontsize=12, fontweight='bold')
                ax.set_ylabel('u2 Output (mA)', fontsize=10)
                ax.grid(True, alpha=0.3, linestyle='--')
                ax.tick_params(labelsize=9)
                stats_text = (
                    f'Mean: {np.mean(joint_u2):+.3f} mA\n'
                    f'Max|u2|: {max_abs_u2:.3f} mA'
                )
                ax.text(
                    0.02,
                    0.98,
                    stats_text,
                    transform=ax.transAxes,
                    fontsize=8.5,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.78)
                )
            for ax in axes2[-2:]:
                ax.set_xlabel('Time (s)', fontsize=10)
            fig2.suptitle('All Joints: Control Torque / u2 Output Overview', fontsize=16, fontweight='bold')
            fig2.tight_layout(rect=[0, 0.02, 1, 0.97])
            filename2 = os.path.join(self.plot_folder, '2_All_Joints_u2_Output.png')
            fig2.savefig(filename2, dpi=150, bbox_inches='tight')
            plt.close(fig2)

            # ========== Figure 3: Joint 1 Angle (Actual vs Desired) ==========
            fig3, ax = plt.subplots(figsize=(14, 7))
            ax.plot(time_arr, actual_pos_arr[:, 0], 'b-', linewidth=2.0,
                   label='Joint 1 Actual Position', alpha=0.9)
            ax.plot(time_arr, desired_pos_arr[:, 0], 'r--', linewidth=2.0,
                   label='Joint 1 Desired Position', alpha=0.75)
            ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
            ax.set_ylabel('Joint Angle (deg)', fontsize=13, fontweight='bold')
            ax.set_title('Joint 1: Real-time Angle Tracking', fontsize=15, fontweight='bold')
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.legend(loc='best', fontsize=11, framealpha=0.9)
            ax.tick_params(labelsize=11)
            filename3 = os.path.join(self.plot_folder, '3_Joint1_Angle_Tracking.png')
            fig3.savefig(filename3, dpi=150, bbox_inches='tight')
            plt.close(fig3)

            # ========== Figure 4: Joint 1 Velocity ==========
            fig4, ax = plt.subplots(figsize=(14, 7))
            ax.plot(time_arr, actual_vel_arr[:, 0], 'g-', linewidth=2.0, alpha=0.9)
            ax.axhline(y=0, color='k', linestyle='--', linewidth=1.0, alpha=0.5)
            ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
            ax.set_ylabel('Joint Velocity (deg/s)', fontsize=13, fontweight='bold')
            ax.set_title('Joint 1: Real-time Angular Velocity', fontsize=15, fontweight='bold')
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.tick_params(labelsize=11)
            filename4 = os.path.join(self.plot_folder, '4_Joint1_Velocity.png')
            fig4.savefig(filename4, dpi=150, bbox_inches='tight')
            plt.close(fig4)

            # ========== Figure 5: Joint 1 Current Output ==========
            fig5, ax = plt.subplots(figsize=(14, 7))
            ax.plot(time_arr, cmd_current_arr[:, 0], color='tab:purple', linewidth=1.8,
                   label='Joint 1 Command Current (u2)', alpha=0.9)
            if actual_current_arr.size > 0:
                ax.plot(time_arr, actual_current_arr[:, 0], color='tab:orange', linewidth=1.5,
                       label='Joint 1 Actual Current', alpha=0.8)
            ax.axhline(y=self.max_current_mA, color='r', linestyle='--', linewidth=1.3,
                      alpha=0.45, label=f'Limit (+/-{self.max_current_mA} mA)')
            ax.axhline(y=-self.max_current_mA, color='r', linestyle='--', linewidth=1.3, alpha=0.45)
            ax.axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
            ax.set_xlabel('Time (s)', fontsize=13, fontweight='bold')
            ax.set_ylabel('Current (mA)', fontsize=13, fontweight='bold')
            ax.set_title('Joint 1: Current / Control Output', fontsize=15, fontweight='bold')
            ax.grid(True, alpha=0.3, linestyle='--')
            ax.legend(loc='best', fontsize=11, framealpha=0.9)
            ax.tick_params(labelsize=11)
            stats_text = (
                f'Cmd Mean: {np.mean(cmd_current_arr[:, 0]):+.3f} mA\n'
                f'Cmd Max|I|: {np.max(np.abs(cmd_current_arr[:, 0])):.3f} mA'
            )
            ax.text(
                0.02,
                0.98,
                stats_text,
                transform=ax.transAxes,
                fontsize=10,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
            )
            filename5 = os.path.join(self.plot_folder, '5_Joint1_Current_mA.png')
            fig5.savefig(filename5, dpi=150, bbox_inches='tight')
            plt.close(fig5)

        except Exception as e:
            print(f"?? Plot saving error: {e}")

    def plot_results(self):
        if len(self.data_time) == 0:
            return
        self._save_plots_internal()
        print(f"\n>>> All 5 plots saved to: {os.path.abspath(self.plot_folder)}")
        print(f"    1. All Joints Tracking Error")
        print(f"    2. All Joints Control Torque / u2 Output")
        print(f"    3. Joint 1 Angle Tracking (Actual vs Desired)")
        print(f"    4. Joint 1 Angular Velocity")
        print(f"    5. Joint 1 Current Output\n")


# 全局变量用于信号处理
_robot_instance = None
_shutdown_requested = False

def signal_handler(signum, frame):
    global _robot_instance, _shutdown_requested
    if _shutdown_requested:
        print("\n>>> 强制退出...")
        sys.exit(1)
    _shutdown_requested = True
    print("\n\n>>> 检测到中断信号 (Ctrl+C)，正在安全停止并保存图表...")
    if _robot_instance is not None:
        # 先停止控制
        _robot_instance.control_active = False
        _robot_instance.shutdown_event.set()
        
        # 等待图表更新线程完成（最多等2秒）
        if _robot_instance.plot_update_thread and _robot_instance.plot_update_thread.is_alive():
            print(">>> 等待图表保存完成...")
            _robot_instance.plot_update_thread.join(timeout=2.0)
        
        # 强制再保存一次
        if len(_robot_instance.data_time) > 0:
            try:
                _robot_instance._save_plots_internal()
                print(f"✓ 图表已保存 ({len(_robot_instance.data_time)} 个数据点)")
            except Exception as e:
                print(f"⚠️ 保存失败: {e}")
        
        _robot_instance.stop_ats_control()

def get_input_windows(timeout=0.1):
    """Windows平台非阻塞输入"""
    if msvcrt.kbhit():
        return msvcrt.getch().decode('utf-8', errors='ignore').lower()
    return None

def get_input_linux(timeout=0.1):
    """Linux/Mac平台非阻塞输入"""
    # 使用select检查stdin是否有数据可读
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1).lower()
    return None

def main():
    global _robot_instance, _shutdown_requested
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    if hasattr(signal, 'SIGTERM'):
        signal.signal(signal.SIGTERM, signal_handler)
    
    print("\n" + "="*70)
    print("Custom robot - ATS自适应力矩控制系统")
    print("="*70)
    
    try:
        # 创建控制器
        print("\n正在连接机械臂...")
        robot = RobotArmController(*get_robot_connection(), 3)
        # robot = RobotArmController(*get_robot_connection(), 3)
        _robot_instance = robot
        print("✓ 连接成功")
        
        # 移动到起始位置
        print("\n>>> 移动到起始位置 (全0度)...")
        robot.movej([0, 0, 0, 0, 0, 0], v=20)
        time.sleep(2)
        print("✓ 就绪")

        print("\n" + "="*70)
        print("控制说明:")
        print("  [S] - 启动ATS控制 (电流环, 运行时长: {}s)".format(robot.run_duration))
        print("  [Q] - 停止并退出程序")
        print("  [Ctrl+C] - 紧急停止")
        print("="*70 + "\n")

        ats_started = False
        
        # 设置stdin为非阻塞模式（Linux/Mac）
        if not WINDOWS:
            import termios
            import tty
            old_settings = termios.tcgetattr(sys.stdin)
            try:
                tty.setcbreak(sys.stdin.fileno())
            except:
                pass  # 如果设置失败，继续使用默认模式
        
        print("等待按键输入 (s=启动, q=退出, Ctrl+C=紧急停止)...")
        
        # 主循环
        while not _shutdown_requested:
            # 获取键盘输入（非阻塞）
            if WINDOWS:
                key = get_input_windows(timeout=0.1)
            else:
                key = get_input_linux(timeout=0.1)
            
            if key is None:
                # 【关键】检查控制线程是否自然结束
                if ats_started and robot.control_thread and not robot.control_thread.is_alive():
                    print("\n>>> ATS控制自动完成 (图表已实时保存)")
                    ats_started = False
                    break  # 自动退出
                time.sleep(0.05)
                continue
            if key == 's':
                if not ats_started:
                    print("\n>>> 启动ATS控制...")
                    # 使用200Hz (5ms周期) 控制频率
                    if robot.start_ats_control(freq_hz=200.0):
                        ats_started = True
                        print("✓ ATS控制运行中... (按 'q' 停止，Ctrl+C 紧急停止)\n")
                    else:
                        print("✗ ATS控制启动失败\n")
                else:
                    print("\n>>> ATS控制已在运行中\n")
                    
            elif key == 'q':
                print("\n>>> 正常退出程序...")
                if ats_started:
                    robot.stop_ats_control()
                break
            

    except KeyboardInterrupt:
        print("\n>>> 用户中断 (Ctrl+C) - 正在保存数据...")
        if _robot_instance is not None:
            _robot_instance.stop_ats_control()
            # 【强制保存】Ctrl+C时也保存图表
            if len(_robot_instance.data_time) > 0:
                try:
                    _robot_instance._save_plots_internal()
                    print(f"✓ 图表已保存（急停模式）")
                except Exception as e:
                    print(f"⚠️ 图表保存失败: {e}")
            
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        if _robot_instance is not None:
            _robot_instance.stop_ats_control()
            # 【强制保存】异常退出时也保存图表
            if len(_robot_instance.data_time) > 0:
                try:
                    _robot_instance._save_plots_internal()
                    print(f"✓ 图表已保存（异常退出模式）")
                except Exception as e2:
                    print(f"⚠️ 图表保存失败: {e2}")
            
    finally:
        # 恢复终端设置（Linux/Mac）
        if not WINDOWS:
            try:
                import termios
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass
        
        print("\n>>> 清理资源...")
        if _robot_instance is not None:
            # 【关键】确保退出电流环模式
            try:
                if _robot_instance.control_active:
                    print(">>> 正在停止ATS控制并退出电流环...")
                    _robot_instance.stop_ats_control()
                else:
                    # 即使控制未启动，也需要安全退出（防止上次异常退出）
                    print(">>> 确保安全退出电流模式...")
                    try:
                        # 【简化逻辑】直接禁用电流环（参考控制器协议）
                        # 位置环会自动接管并保持当前位置
                        _robot_instance.json_ctrl.enable_current_mode(False)
                        print("✓ 电流环已安全退出，位置环接管")
                        
                    except Exception as e:
                        print(f"⚠️ 禁用电流环时出错: {e}")
            except Exception as e:
                print(f"⚠️ 清理电流环时出错: {e}")
            
            # 断开连接
            _robot_instance.disconnect()
        
        print("程序已退出\n")
        
        # 【关键】强制终止所有线程和进程
        import os
        print(">>> 正在终止进程...")
        # 【关键】保持机械臂稳定1秒后再退出
        print(">>> 保持机械臂稳定状态 1 秒...")
        time.sleep(1.0)
        print("✓ 稳定完成")
        
        os._exit(0)  # 强制退出，确保所有线程都被杀死


if __name__ == "__main__":
    main()
    
