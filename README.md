# 面向未知结构机器人系统的自适应 T-S 模糊控制开源项目

# Adaptive T-S Fuzzy Control for Unknown-Structure Robotic Systems



https://github.com/user-attachments/assets/bfaeb7f4-26dc-45a2-81a3-bbf3efeac4be



https://ieeexplore.ieee.org/abstract/document/10416664/

本仓库开源了我们 IEEE T-ASE 论文中自适应 T-S 模糊控制方法的多机器人验证代码。
This repository provides open-source multi-robot validation codes for the adaptive T-S fuzzy control method proposed in our IEEE T-ASE paper.

> **Adaptive T-S Fuzzy Control for an Unknown Structure System With a Self-Adjusting Control Accuracy**
> W. Yan, T. Zhao, and X. Wang, *IEEE Transactions on Automation Science and Engineering*, vol. 22, pp. 944–957, 2025.

本仓库包含 Kinova Gen3、RealMan RM65-B 和达野 DY05S-600 等真实机器人实控代码，也包含 UR10e、RealMan 和 KUKA 等机器人在 Isaac Sim 高保真物理环境中的仿真验证代码。
The repository includes real-robot control codes for Kinova Gen3, RealMan RM65-B, and DaYe DY05S-600, as well as Isaac Sim high-fidelity simulation codes for UR10e, RealMan, and KUKA robots.

本项目旨在支持自适应模糊控制方法的可复现研究、控制器对比实验以及多类型机器人平台上的实际部署。
This project aims to support reproducible research, controller comparison, and practical deployment of adaptive fuzzy control methods on different robotic platforms.

各子文件夹内均提供独立的 `README.md` 文件，用于说明对应机器人平台或仿真环境的具体配置方式、运行步骤和注意事项。
Each subfolder provides an individual `README.md` file describing the specific environment configuration, running steps, and important notes for the corresponding robot platform or simulation environment.

## 仓库内容

## Repository Contents

| 文件夹 / Folder               | 平台 / Platform                                                                  | 语言或环境 / Language or Environment |
| -------------------------- | ------------------------------------------------------------------------------ | ------------------------------- |
| `realman65b_control/`      | RealMan RM65-B 协作机器人 / RealMan RM65-B collaborative robot                      | Python                          |
| `kinova_gen3_control/`     | Kinova Gen3 协作机器人 / Kinova Gen3 collaborative robot                            | Python                          |
| `dy05s600_matlab_control/` | 达野 DY05S-600 工业机器人 / DaYe DY05S-600 industrial robot                           | MATLAB                          |
| `isaac_sim_validation/`    | UR10e、RealMan 和 KUKA 高保真仿真 / UR10e, RealMan, and KUKA high-fidelity simulation | Isaac Sim                       |

## 主要特点

## Features

* 提供面向未知结构机器人系统的自适应 T-S 模糊控制实现。
  Provides an adaptive T-S fuzzy control implementation for unknown-structure robotic systems.

* 支持协作机器人和工业机器人上的真实实控验证。
  Supports real-robot validation on both collaborative and industrial manipulators.

* 提供带有物理世界模型的 Isaac Sim 高保真仿真验证。
  Provides Isaac Sim high-fidelity simulation validation with physics-based world models.

* 可作为自适应模糊控制、鲁棒控制和机器人控制方法的对比基线。
  Can be used as a comparison baseline for adaptive fuzzy control, robust control, and robotic manipulator control.

## 相关 ATS 控制工作

## Related ATS-Based Works

本仓库主要对应上述 IEEE T-ASE 论文，相关 ATS 控制框架也可参考以下工作。
This repository is mainly associated with the IEEE T-ASE paper above, while the related ATS-based control framework can also be found in the following works.

* W. Yan, T. Zhao, B. Niu, X. Wang, and X. Xie, “Nonsingular Adaptive T-S Fuzzy Model-Based Control for Constrained Unknown-Structure Heterogeneous Multi-Agent Systems With a Predefined Accuracy,” *Information Sciences*, 2025.
* W. Yan, T. Zhao, and E. Q. Wu, “Prescribed-Time Fuzzy Control for MIMO Coupled Systems With Unknown Structure and Control Direction: Application to Robotic Arm,” *IEEE Transactions on Automation Science and Engineering*, 2025.

## 引用

## Citation

如果本仓库或其中的机器人验证代码对您的研究有帮助，请引用我们的主要论文。
If this repository or the provided robotic validation examples help your research, please cite our main paper.


@article{yan2025adaptive,
  title   = {Adaptive T-S Fuzzy Control for an Unknown Structure System With a Self-Adjusting Control Accuracy},
  author  = {Yan, W. and Zhao, T. and Wang, X.},
  journal = {IEEE Transactions on Automation Science and Engineering},
  volume  = {22},
  pages   = {944--957},
  year    = {2025}
}

## License

This project is licensed under the Apache License 2.0.
