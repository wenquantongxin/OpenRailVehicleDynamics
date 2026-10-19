# ORVD 计算性能文档

本目录管理 ORVD 在**保持既有车辆动力学资格结论**前提下的计算性能工作。它与已经封存的
G01–G82 迁移路书分开：车辆能力是否存在由迁移路书回答，现有能力怎样减少墙钟、CPU 和内存开销
由本目录回答。

## 当前文档

| 文档 | 职责 |
|---|---|
| [platform_evaluations/LINUX_X86_64.md](platform_evaluations/LINUX_X86_64.md) | Linux x86-64 计算评价：现行 `main` 的 GCC Release 四工况两轮计时、分项、实时系数与积分工作量；首页性能表的来源。 |
| [platform_evaluations/MACOS_APPLE_SILICON.md](platform_evaluations/MACOS_APPLE_SILICON.md) | macOS Apple silicon 计算评价：AppleClang／GCC、多 worker、四工况计时、资源与固定槽确定性。 |
| [integrator_evaluations/LINUX_X86_64_SINGLE_THREAD.md](integrator_evaluations/LINUX_X86_64_SINGLE_THREAD.md) | 时间积分方法单线程计算评价：CVODE、Zhai、Newmark 在四工况上的计算量、墙钟及相对 Radau5 参考轨迹的差值。 |

## 计时术语

- **动力学推进墙钟**：当次运行配置的数值后端、车辆右端和接受态事务的墙钟时间。
- **观测墙钟**：在既有密集状态上重放车辆与轮轨观测的墙钟时间；归档样本不成为额外积分停靠点。
- **运行器分段总计**：推进、观测和工件写出的分段计时之和；不包含随后写性能摘要、
  发布目录以及进程启动／退出的尾部。
- **完整进程墙钟**：从启动资格程序到其退出的实测墙钟；这是端到端实时系数的权威分母。
- **实时系数**：模拟时长除以墙钟时间。大于 `1` 表示平均吞吐快于实时，但不证明最坏时延或硬实时。

引用性能数字时必须说明工况、积分方法与参数、Release 配置、线程数、CPU 亲和性和上述计时边界。
短窗只用于筛选与定位，不能线性外推为 `16/20/30 s` 长窗结果。

## 证据与提交纪律

1. 临时轨迹、性能摘要、剖析结果和试验脚本放在仓库 `tmp/` 或仓外临时目录，不进入 Git，也不作为
   产品输入、测试期望或永久金标。
2. 仓内只保存算法、解析误差界、接受／否决结论和可重算的证据入口；不固化某台机器的速度门或
   输出哈希。
3. 逐位等价优化必须保持物理观测、接触斑记录和积分统计完全一致。具有明确误差预算的优化必须
   重新运行 GZ18 与 IRW 的冻结长窗，并继续按现行 SIMPACK 口径观察；IRW 阈值是启动完整时序、
   模态、相位、峰值次序、包络、接触切换和最长零力复核的报警线，不由孤立峰值自动裁决。
4. 候选通过后直接替换唯一实现；候选失败则撤回，不在产品中留下兼容开关、旧算法分支或自由组合
   的性能旋钮。
5. 直接父提交不重新构建或运行，复用上一轮留存的父工件和性能数据；本轮候选只在 Release、当前
   目标线程预算和固定 CPU 亲和性下运行两次，结果以算术平均和最小—最大范围报告。跨轮比较不冒称
   同场配对；两次后仍无法区分收益与噪声时挂起或否决，不补第三次追逐最好值。

## 当前状态

现行 `main` 的 Linux 四工况正式计时见
[platform_evaluations/LINUX_X86_64.md](platform_evaluations/LINUX_X86_64.md)：Release、GCC 13、`8` worker、
固定 CPU `8–15`、`0.5 ms` 采样、各场景默认配方，两轮均值的推进实时系数为 `4.992× / 4.844× / 1.139× / 1.394×`，
完整进程为 `2.455× / 2.337× / 0.891× / 1.001×`；首页性能表取自该文档。macOS Apple silicon 的多 worker 与
双编译器计时见 [platform_evaluations/MACOS_APPLE_SILICON.md](platform_evaluations/MACOS_APPLE_SILICON.md)；
Windows 计时待补。五种时间积分方法的单线程计算量与相对 Radau5 参考轨迹的差值见
[integrator_evaluations/LINUX_X86_64_SINGLE_THREAD.md](integrator_evaluations/LINUX_X86_64_SINGLE_THREAD.md)。

各场景的默认积分配方（GZ18 与 100 Hz 受控 IRW 为 CVODE BDF2，R300＋AAR5 被动 IRW 为 CVODE BDF5 及其收紧
容差）登记在 [tools/dynamics_qualification/README.md](../../tools/dynamics_qualification/README.md)；轮轨低层
策略按车型绑定的架构裁决见 [ADR-0005](../adr/0005-bind-wheel-rail-low-level-strategies-by-vehicle.md)。
专项性能分支与迁移路书的阶段记录已不再维护；本目录只保留现行 `main` 的可复算评价。
