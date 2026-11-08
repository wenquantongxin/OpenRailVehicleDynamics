# 计算性能

平台测量见 [Linux x86-64](platform_evaluations/LINUX_X86_64.md) 与
[macOS Apple silicon](platform_evaluations/MACOS_APPLE_SILICON.md)，包含车辆工况、积分配置、线程设置与完整窗口计时。
[GZ18 单线程积分器评价](integrator_evaluations/LINUX_X86_64_SINGLE_THREAD.md)给出四种方法的计算量、墙钟与数值差值。
运行命令和积分配置见[车辆运行工具](../../tools/dynamics_qualification/README.md)。

## 计时定义

- **推进／控制**：数值推进墙钟；受控运行另加保持量同步和控制计算。
- **观测**：在保存的状态样本上求车辆响应和轮轨接触观测。
- **写出**：结果表与元数据的组织、写入。
- **完整进程**：从运行程序启动到退出，包含模型载入、数值计算、观测和结果发布。

实时系数为仿真时长与相应墙钟时间之比，度量平均吞吐。后端构造单独计时，包含在完整进程中。
机械输出周期决定采样密度；控制周期决定保持输入的更新时刻。
