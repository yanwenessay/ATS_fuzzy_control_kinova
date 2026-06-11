# ATS_fuzzy_control_kinova（以kinova gen 3 机械臂为示例）

There are some free-model control (ATS control) for robot arm dynamics system, and it is applicable to low-speed control without any dynamic feedforward.

这是一些ATS控制器设计方案对于机械臂动力学系统（python），适用于低速控制，不需要任何动态前馈。若有问题，请联系我，相关论文：
1. Yan, Wen, et al. "Adaptive TS fuzzy control for an unknown structure system with a self-adjusting control accuracy." IEEE Transactions on Automation Science and Engineering 22 (2024): 944-957.
2. Yan, Wen, Tao Zhao, and Edmond Q. Wu. "Prescribed-time fuzzy control for MIMO coupled systems with unknown structure and control direction: Application to robotic arm." IEEE Transactions on Automation Science and Engineering 22 (2024): 9013-9028.

# Kinova Gen3 ATS 关节空间阻抗控制

本仓库提供一个面向 Kinova Gen3 7 自由度机械臂的 ATS 自适应关节空间阻抗/力矩控制示例。

程序会先将机械臂移动到初始关节位置，然后执行 7 个关节的正弦轨迹跟踪，并在结束后生成跟踪误差和控制性能分析图。

## 文件结构

```text
kinova_gen3_ats_open_source/
  ats_control_7dof_dynamic.py    # 主程序
  control_main.py                # ATS 控制律
  DiscreteIntegrator.py          # 离散积分器
  ts_fuzzy_output.py             # TS 模糊输出
  fuzzy_membership_fcn.py        # 模糊隶属度函数
  fuzzyoutput.py                 # 模糊输出辅助函数
  config/default_config.json     # 默认公开配置
  requirements.txt
```

## 安全提醒

本程序会让真实机械臂进入低层力矩控制流程。运行前请确认：

* 机械臂周围没有人员和障碍物；
* 急停、示教器和安全限位可用；
* 机械臂处于无 fault 状态；
* 已根据自己的平台检查并调整配置参数；
* 首次测试时建议调小 `run_duration`、`amplitude_deg` 和 `u2_max_torque`。

请在充分理解风险后使用本代码。

## 安装

建议将本目录放在 Kinova Kortex Python API 的 examples 目录下，例如：

```text
api_python/examples/kinova_gen3_ats_open_source/
```

安装依赖：

```bash
python3 -m pip install -r requirements.txt
```

如果已经安装好 `kortex_api`，通常只需安装：

```bash
python3 -m pip install numpy matplotlib
```

## 运行

不要将机械臂 IP、用户名或密码写入仓库。运行时使用占位符：

```bash
python3 ats_control_7dof_dynamic.py --ip <YOUR_ROBOT_IP> -u <YOUR_USERNAME> -p <YOUR_PASSWORD>
```

程序启动后：

```text
s = 开始控制
q = 退出程序
Ctrl+C = 中断并尝试保存已有数据
```

## 配置

复制默认配置文件，并在本地修改：

```bash
cp config/default_config.json config/local_config.json
```

使用本地配置运行：

```bash
python3 ats_control_7dof_dynamic.py \
  --config config/local_config.json \
  --ip <YOUR_ROBOT_IP> -u <YOUR_USERNAME> -p <YOUR_PASSWORD>
```

常用配置项包括：

* `run_duration`：控制运行时间；
* `u1_limit`：中间控制量限幅；
* `u2_max_torque`：正常控制力矩限幅；
* `u2_safety_torque`：安全模式力矩限幅；
* `axis_limits_deg`：7 个关节的软限位；
* `axis_trajectory`：各关节正弦参考轨迹参数；
* `axis_control_params`：各关节 ATS 控制参数。

## 输出结果

运行结束后，会生成类似如下结果目录：

```text
kinova_joint_tracking_errors_<timestamp>/
```

其中包括：

* 各关节跟踪误差图；
* 控制参数分析图；
* RMS 误差、最大误差、平均绝对误差等统计信息。


## License

This project is licensed under the Apache License 2.0.
