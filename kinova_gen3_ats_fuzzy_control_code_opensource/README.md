# Kinova Gen3 ATS Joint-Space Impedance Control

这是一个面向 Kinova Gen3 7 自由度机械臂的 ATS 自适应关节空间阻抗/力矩控制示例包。程序会移动到起始关节位置，然后执行 7 个关节的正弦轨迹跟踪，并在结束后生成各关节跟踪误差和控制参数分析图。

## 文件结构

```text
kinova_gen3_ats_open_source/
  ats_control_7dof_dynamic.py    # 主程序
  control_main.py                # ATS 控制律
  DiscreteIntegrator.py          # 离散积分器
  ts_fuzzy_output.py             # TS 模糊输出
  fuzzy_membership_fcn.py        # 模糊隶属度函数
  fuzzyoutput.py                 # 模糊输出辅助函数
  config/default_config.json     # 可公开的默认控制配置
  requirements.txt
```

## 安全提醒

这个程序会让真实机械臂进入低层伺服/力矩控制流程。运行前请确认：

- 机械臂周围没有人员和障碍物。
- 急停、示教器和安全限位可用。
- 机械臂处于无 fault 状态。
- 你已经理解并按自己的平台重新调试 `config/default_config.json`。
- 第一次运行建议把 `run_duration`、`amplitude_deg`、`u2_max_torque` 调小，并随时准备急停。

## 安装

建议把本目录放在 Kinova Kortex Python API 的 `api_python/examples/` 目录下，例如：

```text
api_python/examples/kinova_gen3_ats_open_source/
```

安装依赖：

```bash
python3 -m pip install -r requirements.txt
```

如果你已经按 Kinova 官方方式安装了 `kortex_api`，可以只安装：

```bash
python3 -m pip install numpy matplotlib
```

## 运行

不要把你的机械臂 IP、用户名或密码写进仓库。运行时由用户自行填写：

```bash
python3 ats_control_7dof_dynamic.py --ip <ROBOT_IP> -u <USERNAME> -p <PASSWORD>
```

例如公开 README 中只保留占位符：

```bash
python3 ats_control_7dof_dynamic.py --ip <YOUR_ROBOT_IP> -u <YOUR_USERNAME> -p <YOUR_PASSWORD>
```

程序启动后：

- 输入 `s` 并回车：初始化力矩控制并开始跟踪。
- 输入 `q` 并回车：退出。
- `Ctrl+C`：紧急中断，程序会尝试保存已有数据并停止控制。

## 自定义配置

复制默认配置到本地文件，不要提交到 Git：

```bash
cp config/default_config.json config/local_config.json
```

然后运行：

```bash
python3 ats_control_7dof_dynamic.py --config config/local_config.json --ip <ROBOT_IP> -u <USERNAME> -p <PASSWORD>
```

常用字段：

- `run_duration`: 控制运行时间，单位秒。
- `u1_limit`: 中间控制量限幅。
- `u2_max_torque`: 正常控制力矩限幅。
- `u2_safety_torque`: 安全模式力矩限幅。
- `axis_limits_deg`: 7 个关节的软限位角度。
- `axis_trajectory`: 7 个关节的轨迹参数，使用 `offset_deg + amplitude_deg * sin(2*pi*frequency_hz*t + phase_offset)`。
- `axis_control_params`: 7 个关节的 ATS 控制参数。

## 输出

运行结束后，程序会在当前目录生成类似 `kinova_joint_tracking_errors_<timestamp>/` 的结果目录，包含：

- 每个关节的 tracking error 图。
- 综合控制参数分析图。
- 终端中的 RMS、最大误差、平均绝对误差等统计信息。

## 开源隐私说明

本目录不包含真实机械臂 IP、登录用户名、密码或现场网络信息。请保持 `config/local*.json`、运行日志和图片结果不提交到公开仓库。
