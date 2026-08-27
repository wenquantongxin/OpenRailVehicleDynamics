# macOS Apple silicon 计算评价

## 配置选择

| 用途 | 工具链 | worker | 依据 |
|---|---|---:|---|
| 默认 Release | AppleClang 21＋LLVM libomp | 16 | 四工况主计时和最低 |
| 降低线程和内存占用 | AppleClang 21＋LLVM libomp | 12 | 完整进程墙钟和最低，性能与 16 worker 基本相同 |
| GCC 备用 | GCC 15.2＋libgomp | 12 | GCC 四工况均在 12 worker 最快 |

32 worker 在全部八个“工况 × 编译器”组合中都不是最快档。对每种编译器，generic 与
`-mcpu=native` 构建生成的三个最终运行器均逐字节相同，因此本轮计时不区分两者。同一实际二进制
在 `8/12/16/32` worker 下保持固定槽逐位一致。

## 测试基线

Apple M3 Max arm64，16 核（12 performance＋4 efficiency）；Release、C++23、`-O3 -DNDEBUG`、
LTO 关闭；AppleClang 21.0.0.21000101＋LLVM libomp 与 GCC 15.2＋libgomp。不设置 CPU affinity。

12 worker 是线程预算，不限定 macOS 调度器只使用 performance 核；32 worker 超过物理核心总数。

## 计算工况

| 工况 | 仿真时长 | 输出／控制钟 | 积分配方 | 容差 `rtol / q / v / z` | 主计时量 |
|---|---:|---|---|---|---|
| GZ18 直线＋AAR6，60 km/h | 20 s | 机械输出 0.5 ms | CVODE BDF2 | `1e-6 / 1e-7 / 1e-6 / 0.1 N` | `advance_wall_seconds` |
| GZ18 R300＋AAR5，60 km/h | 16 s | 机械输出 0.5 ms | CVODE BDF2 | `1e-6 / 1e-7 / 1e-6 / 0.1 N` | `advance_wall_seconds` |
| IRW R300＋AAR5，被动，60 km/h | 30 s | 机械输出 0.5 ms | CVODE BDF5 | `1e-8 / 1e-8 / 1e-7 / 1e-6 N` | `advance_wall_seconds` |
| IRW R300＋AAR5，100 Hz 全状态受控，60 km/h | 30 s | 机械输出 0.5 ms；控制 10 ms | CVODE BDF2 | `1e-6 / 1e-6 / 1e-5 / 1e-6 N` | `integrated_dynamics_wall_seconds` |

前三个工况的主计时是动力学推进墙钟；受控 IRW 的主计时是推进／同步与控制墙钟之和。完整进程
墙钟另含模型载入、观测重放、诊断、结果组织和写出。

## 执行与验证

每次运行均设置 `OMP_NUM_THREADS=<worker>`、`OMP_THREAD_LIMIT=<worker>`、`OMP_DYNAMIC=FALSE` 和
`OMP_MAX_ACTIVE_LEVELS=1`。worker 集合为 `{8,12,16,32}`。

每个“工况 × worker”按 GCC generic、GCC native、AppleClang generic、AppleClang native 正序运行，
再按逆序运行，共 `128` 个 fresh process。generic/native runner 相同后，每个
“工况 × 编译器 × worker”单元合并为 4 次运行；主计时相对极差为 `0.16%–1.91%`。

运行器逐次确认 Jacobian worker 等于目标 worker，接触批处理探针观测到 8-worker OpenMP team。
主计时来自运行器内部单调时钟；完整进程墙钟和峰值 RSS 来自 `/usr/bin/time -l`。

| 检查 | 结果 |
|---|---|
| 长窗运行 | `128/128` 成功 |
| 固定槽分组 | 所有编译配置分组和实际二进制分组均通过 |
| 跨 worker 工件 | 同一实际二进制的连续状态、观测、接触斑、控制事件和端点诊断逐位一致 |
| 跨 worker 积分统计 | 除请求 worker 数记录外，同一实际二进制一致 |

## 主计时

数值为四次运行的算术平均，单位为秒。粗体表示该工况、该编译器的最低值。

| 工况 | 编译器 | 8 worker | 12 worker | 16 worker | 32 worker |
|---|---|---:|---:|---:|---:|
| GZ18 直线＋AAR6 | AppleClang | 6.135 | 5.977 | **5.948** | 6.039 |
| GZ18 直线＋AAR6 | GCC 15.2 | 6.996 | **6.818** | 6.849 | 7.026 |
| GZ18 R300＋AAR5 | AppleClang | 4.983 | **4.889** | 4.899 | 4.923 |
| GZ18 R300＋AAR5 | GCC 15.2 | 5.708 | **5.500** | 5.502 | 5.670 |
| IRW R300＋AAR5 被动 | AppleClang | 38.606 | **37.418** | 37.563 | 37.819 |
| IRW R300＋AAR5 被动 | GCC 15.2 | 41.384 | **39.969** | 40.197 | 41.298 |
| IRW R300＋AAR5 100 Hz 受控 | AppleClang | 28.367 | 27.179 | **27.022** | 27.799 |
| IRW R300＋AAR5 100 Hz 受控 | GCC 15.2 | 31.676 | **30.294** | 30.317 | 31.405 |

相同 worker 下，AppleClang 各工况比 GCC 快 `6.38%–14.04%`。统一使用 16 worker 时，AppleClang
四工况主计时和比 GCC 低 `8.97%`。

## 实时系数

| 工况 | AppleClang worker | 主计时 / s | 仿真时长 ÷ 主计时 |
|---|---:|---:|---:|
| GZ18 直线＋AAR6 | 16 | 5.948 | 3.362× |
| GZ18 R300＋AAR5 | 12 | 4.889 | 3.273× |
| IRW R300＋AAR5 被动 | 12 | 37.418 | 0.802× |
| IRW R300＋AAR5 100 Hz 受控 | 16 | 27.022 | 1.110× |

## 统一 worker

| 编译器 | worker | 四工况主计时和 / s | 相对 8-worker 加速 | 四工况完整进程和 / s |
|---|---:|---:|---:|---:|
| AppleClang | 8 | 78.091 | 1.0000× | 117.915 |
| AppleClang | 12 | 75.463 | 1.0348× | **114.990** |
| AppleClang | 16 | **75.432** | **1.0353×** | 115.028 |
| AppleClang | 32 | 76.582 | 1.0197× | 116.235 |
| GCC 15.2 | 8 | 85.763 | 1.0000× | 128.697 |
| GCC 15.2 | 12 | **82.582** | **1.0385×** | **125.295** |
| GCC 15.2 | 16 | 82.865 | 1.0350× | 125.550 |
| GCC 15.2 | 32 | 85.399 | 1.0043× | 128.207 |

AppleClang 12 与 16 worker 的主计时和相差 `0.031 s`；16 worker 主计时最低，12 worker 完整进程
墙钟最低。

## CPU 与内存

平均有效 CPU 核按完整进程 `(user + sys) / real` 计算；峰值 RSS 为四工况全部运行的平均。

| 编译器 | worker | 平均有效 CPU 核 | 平均峰值 RSS / MiB |
|---|---:|---:|---:|
| AppleClang | 8 | 3.639 | 259.4 |
| AppleClang | 12 | 3.755 | 292.3 |
| AppleClang | 16 | 3.843 | 325.5 |
| AppleClang | 32 | 3.929 | 458.0 |
| GCC 15.2 | 8 | 3.843 | 259.8 |
| GCC 15.2 | 12 | 3.969 | 292.7 |
| GCC 15.2 | 16 | 4.061 | 325.7 |
| GCC 15.2 | 32 | 4.048 | 457.7 |

从 16 增至 32 worker，平均峰值 RSS 增长约 `41%`。32 worker 相对各编译器最佳档：AppleClang
各工况慢 `0.70%–2.88%`，四工况主计时和慢 `1.52%`；GCC 各工况慢 `3.05%–3.67%`，四工况
主计时和慢 `3.41%`。
