# IRW 被动多速度 SIMPACK 直接求解器对拍

## 当前状态

本文是 `experiments/` 下第一个功能性示例的执行入口。当前已冻结目的、输入身份、运行边界和结果
口径。R300／60 km/h 已完成并写回仓库根 README；直线与 R600／80 km/h 已完成编排、双精度
SBR 提取、ORVD 串行运行及响应图生成。直线／120 km/h 已完成首个 AAR6 短窗；R1000／120 km/h 的四条
30 s 数值路径已经完成，跨软件比较严格止于 `8.71 s` 的共同可比线路范围。本文不代表多速度工况
已经整体通过资格。

本示例要回答一个明确问题：同一 IRW 车辆从不同纵向初速度开始，在八个独立轮转矩全程为零、车辆
自然减速的条件下，ORVD 与 SIMPACK 对相同线路几何和相同冻结轨道不平顺的动力学响应能否闭合。
SIMPACK 侧采用直接求解器（direct SLV），不使用 Realtime API；Realtime 仍只服务需要外部 100 Hz
控制事件的受控工况。

## 仓库与工件边界

- 本目录保存稳定的示例说明、具名场景表、SIMPACK 编排入口、`libsbr` 提取入口和响应绘图入口；
  这些通用代码进入 Git，但不构建、不安装，也不进入默认 CTest。
- 编排入口不生成第二份车型资源树。每轮只在原
  `vehicle_library/irw/reference_models/simpack/main_model/` 目录独占创建一个具名临时 SPCK，使
  `../ref_files/` 等相对引用保持原语义。
- SBR、求解日志、提取表和短期图表放在 `tmp/irw_passive_multispeed_simpack_slv_comparison/`；大型
  运行结果放在外置实验数据根。这些数值工件不进入 Git。
- 编排器从当前 Git 修订读取 [IRW SIMPACK 模型](../../vehicle_library/irw/reference_models/simpack/main_model/irw_vehicle.spck)
  的受管文本，再在原目录写临时副本；工作区中尚未固化的 GUI 或求解器状态不会进入运行。
- 不计算或保存 SHA 校验；来源由 Git 修订、场景清单和精确的临时差异记录。

编排入口先在 `tmp/` 获取跨进程运行锁；同一模型只允许一个 SIMPACK 求解进程，第二个请求响亮拒绝，
不尝试证明共享子结构在并发下可安全复用。临时 SPCK 再以专用前缀互斥创建，文件名包含场景身份与
本次运行身份。正常结束、异常和可处理的终止信号都清理该副本；不可处理的进程终止可能留下孤儿
文件，后续入口只能依据自己保存的运行清单精确清理，禁止用宽泛通配符删除。求解使用
绝对路径参数 `--file <临时 SPCK> --integration --output-path <空的运行输出目录> --log-file <日志>`，
把 `.sbr/.sir/.spckst/.intinfo/.licreq.log` 全部定向到本轮独立目录，不能依赖默认的
`<模型名>.output`，也不能在 `main_model/` 留下结果目录。原始 `irw_vehicle.spck` 在整个过程中保持
不变。临时 SPCK 不加入 `.gitignore`；若异常遗留，`git status` 应把它公开暴露，运行收尾检查也必须
因本轮副本未清除而失败，不能静默隐藏。

一次 SIMPACK 运行只有同时满足进程退出码为零、日志明确正常结束、预期 SBR 存在且可由 `libsbr`
完整打开时才写完成标志。中断运行的 SBR、SIR 和日志均视为不完整工件；收尾只依据本轮运行清单处理
本轮路径，并且只删除由本进程独占创建、身份仍匹配的临时 SPCK。SBR 文件名由临时 SPCK 的 stem
派生，提取器必须从运行清单取得预期路径，不能硬编码为 `irw_vehicle.sbr`。

## 功能入口

- [irw_passive_multispeed_simpack_direct_slv_scenario_catalog.py](irw_passive_multispeed_simpack_direct_slv_scenario_catalog.py)
  冻结场景身份，不从 Track 名称推断速度或时长。
- [run_irw_passive_simpack_direct_slv_and_orvd_scenario.py](run_irw_passive_simpack_direct_slv_and_orvd_scenario.py)
  持锁串行运行 SIMPACK direct SLV 与 ORVD，并保留精确临时模型差异和运行清单。
- [extract_irw_passive_simpack_direct_slv_sbr_channels.py](extract_irw_passive_simpack_direct_slv_sbr_channels.py)
  通过 `libsbr` 读取 binary64 SBR，提取四轴响应、八轮转速、八轮三向接触力和电机转矩。
- [plot_irw_passive_simpack_direct_slv_and_orvd_dynamics_response.py](plot_irw_passive_simpack_direct_slv_and_orvd_dynamics_response.py)
  按 README 图式生成四轴横移／摇头图和八轮逐轮接触斑汇总三向力图。

编排入口面向安装 SIMPACK 2021x 的 Linux 主机，默认使用 `/opt/Simpack-2021x`、Release 资格程序和
`16` 个 ORVD 工作者；`--simpack-slv`、`--libsbr`、`--qualification-binary` 与 `--worker-count`
可以显式覆盖，并会把实际程序路径及工作者设置写入运行清单。

R300＋AAR5／60 km/h 场景的调用方式为：

```bash
python experiments/irw_passive_multispeed_simpack_slv_comparison/\
run_irw_passive_simpack_direct_slv_and_orvd_scenario.py \
  --scenario irw_r300_aar5_v60_passive \
  --run-root <仓外空目录>

python experiments/irw_passive_multispeed_simpack_slv_comparison/\
plot_irw_passive_simpack_direct_slv_and_orvd_dynamics_response.py \
  --scenario irw_r300_aar5_v60_passive \
  --run-root <同一运行目录>
```

## 已冻结的物理身份

### 被动运行

- 八个独立轮的驱动转矩在整个仿真中严格为零。
- 不启用纵向巡航、保速或导向控制。
- 车辆具有指定初始纵向速度，随后按动力学自然减速。
- SIMPACK 每轮必须在结果中核对八个外部 Simat 转矩输入均为零；不能仅凭“未连接控制器”推断。
- ORVD 每轮必须核对八个上下文转矩输入均为零。

### SIMPACK 启动

临时 SPCK 每轮显式设置：

```text
track.active = <场景指定线路>
vehicle.startvel = { <场景速度>/3.6 }
vehicle.applystartvel = 1
slv.integ.tend.time = { <场景时长> s }
slv.integ.tout.freq = { <输出频率> Hz }
```

`vehicle.applystartvel=1` 由普通初始化依据纵向速度建立相容速度状态。不得使用延续运行，也不得把
模型中保存的旧轮速直接当成本轮起始轮速。临时脚本必须逐项确认每个目标字段只匹配一次；字段缺失、
重复或活动线路名称不存在时立即失败。

速度来自本文的显式场景表，脚本不得从 Track 名称自动解析。尤其不能把其他车型中带速度字样的历史
Track 名称当作当前求解速度的权威。

### ORVD 启动

[60 km/h 已解析启动状态](../../vehicle_library/irw/startup_states/moving_startup_60kmh.json) 是第一轮
高速启动假设的唯一来源。对目标速度 \(v\)，临时 JSON 按 \(v/(60\ \mathrm{km/h})\) 同时缩放：

1. `initial_longitudinal_speed_meters_per_second`；
2. 八个独立轮的显式角速度。

其余已解析位姿、横竖向速度、内部力状态和车轮相位保持不变。临时 JSON 不覆盖产品资产。每轮在
`t=0` 明确比较 SIMPACK 配置起速、ORVD 报告初速、两侧四轴站位和八轮角速度；如果高速下两侧启动
身份未闭合，先归因启动，不继续解释后续动力学差异。

这一缩放是待动力学对拍验证的第一轮假设，不预先宣称它对全部速度都等价于重新做静力平衡或速度
相关配平。

## 轨道不平顺身份

跨求解器逐点对拍只使用随包的冻结场：

| 场 | JSON 定义域 | SIMPACK 门控 | 满幅区间 |
|---|---:|---:|---:|
| AAR5 | `0–1100 m` | `160–1100 m`，两端 `40 m` | `200–1060 m` |
| AAR6 | `0–300 m` | `50–300 m`，两端 `50 m` | `100–250 m` |
| ERRI low | `0–500 m` | `50–500 m`，两端 `50 m` | `100–450 m` |

这些冻结点列已经包含 SIMPACK 最终门控包络，ORVD 不得叠加第二次淡入淡出。实现依据见
[轨道不平顺谱说明](../../docs/models_and_algorithms/track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)
和[冻结／生成场解析接口](../../libs/configuration/include/orvd/configuration/resolve_track_irregularity_field.h)。

新生成器产生的 AAR realization 与 SIMPACK 冻结 realization 相互独立。即使 PSD、频带和统计幅值
相同，也不作逐时刻或逐站位差值；生成场只用于总体趋势、统计响应和后续蒙特卡洛研究。不得把
“相同 seed 数字”解释成两个软件共享相位。

## 场景矩阵

下面是完整目标矩阵。执行按建议顺序推进，不要求首个闭环一次跑完八个场景。

| 建议顺序 | SIMPACK 活动线路 | ORVD 几何 | 冻结不平顺 | 初速度 | 首轮时长 | 作用 |
|---:|---|---|---|---:|---:|---|
| 0 | `$Trk_Curve_R300m_60kmph` | `r300_centerline_superelevation_1100m.json` | AAR5 | 60 km/h | 30 s | 校准 direct SLV、`libsbr`、时间键、符号和启动身份 |
| 1a | `$Trk_AAR5_80kmph` | `straight_level_1100m.json` | AAR5 | 80 km/h | 30 s | 80 km/h 直线基座 |
| 1b | `$Trk_Curve_R600m_80kmph` | `r600_centerline_superelevation_1100m.json` | AAR5 | 80 km/h | 30 s | 与 1a 隔离线路几何效应 |
| 2a | `$Trk_AAR6_120kmph` | `straight_level_1100m.json` | AAR6 | 120 km/h | 8 s | 120 km/h 直线基座 |
| 2b | `$Trk_Curve_R1000m_120kmph` | `r1000_centerline_superelevation_300m.json` | AAR6 | 120 km/h | 8 s | 稳定入口与 2a 隔离线路几何效应；另有一次性四算法 30 s 运行，资格比较仍止于共同定义域 |
| 3 | `$Trk_Curve_R800m_100kmph` | `r800_centerline_superelevation_1100m.json` | AAR5 | 100 km/h | 30 s | 补齐中间速度曲线工况 |
| 4 | `$Trk_AAR6_160kmph` | `straight_level_1100m.json` | AAR6 | 160 km/h | 6 s | 在 AAR6 `300 m` 边界前结束 |
| 5 | `$Trk_ERRI_low_200kmph` | `straight_level_1100m.json` | ERRI low | 200 km/h | 8 s | 高速良好线路工况，在 `500 m` 边界前结束 |

短窗不是资格不足。AAR6 的比较在 `300 m` 后天然失去共同输入身份，因此宁可在共同域内结束，也不为
走完整条 SIMPACK 长线路而扩充冻结点列。表中时长按初速度和前轴初始站位留有边界余量；正式运行仍
须用每个轴桥的实际站位确认所有用于比较的样本都在共同定义域内，不能只以 `v_0 t` 估算。

R300／60 km/h 闭合后才进入 80 km/h，再进入 120 km/h。其余场景只在两组同速直线／曲线对拍没有
暴露公共管线错误后运行。

## SIMPACK 输出与 SBR 提取

当前模型的 `slv.integ.tout.freq=100 Hz` 是结果写出时钟，不是内部积分步长。直接求解器仍按自身
积分设置推进；在 `Meet output points` 关闭时，除描述状态（descriptive states）外的状态值使用所在
内部积分步的同一多项式在输出时刻插值；描述状态不作这种插值，因此不能把两类通道混为一种采样语义。模型中
同时存在的 `slv.rt.stepsize=1 ms` 和固定步长字段也不能用来推断 direct SLV 实际采用固定步长。
每轮必须从求解日志／积分信息确认实际求解器身份。

第一轮的宏观车体、构架和轴桥响应可以使用原生 100 Hz 时钟。200 km/h 下 `0.333 cycles/m`
对应约 `18.5 Hz` 的几何激励，100 Hz 对总体趋势具有足够采样余量；但这不证明 100 Hz 能保存接触
切换形成的三向力尖峰或描述状态跳变。正式轮轨力峰值资格前提高 SBR 输出频率，并预先冻结该频率，
不能看完结果再选择。高频运行是否只改变输出观察而不改变端点和积分统计，也要用同场小窗复核；不
为追逐输出点而默认打开 `Meet output points`，因为那会截断内部步并改变求解成本。

SBR 使用 SIMPACK 安装目录中的 `libsbr` ABI 直接读取，不经 SIMPACK Post 文本导出。提取器至少要：

- 读取并拒绝非 binary64 的浮点精度或存储宽度；
- 以通道完整路径建立唯一映射，保存单位和坐标方向；
- 输出显式时间键，不按行号拼接两条轨迹；
- 提取四轴横移、摇头和站位、八轮角速度以及八轮三向接触力；
- 提取或旁证八轮转矩为零；
- 对缺失、重复、非有限或非严格递增时间通道响亮失败。

既有 `libsbr` 读取实现可作为实现证据，但本示例不能在运行时依赖另一个源码仓库。稳定提取入口随
本示例进入 `experiments/irw_passive_multispeed_simpack_slv_comparison/`，只依赖本机 SIMPACK 安装
提供的 `libsbr` ABI。

## ORVD 资格入口

当前 [IRW 被动场景分派入口](../../tools/dynamics_qualification/irw_passive_scenario_recipe_dispatch.cc) 使用闭合的具名场景
身份，不再仅按不平顺标识选择积分配方。R300/AAR5/60 km/h、直线或 R600/AAR5/80 km/h 以及
直线/AAR6/120 km/h 分别绑定线路几何、冻结不平顺、启动速度和逐实例私有积分配方；不匹配的线路、
不平顺或启动速度会在推进前拒绝。每个身份至少同时绑定：

- 初速度；
- 线路几何资产；
- 冻结不平顺资产；
- 被动零转矩人格；
- 仿真时长和输出时钟；
- 逐实例私有积分配方。

不开放任意字符串拼装，不把数值容差变成公共产品配置，也不把 R300 已验证的 BDF-5/t8 结果外推为
所有速度和曲线半径的默认结论。直线／80 km/h、R600／80 km/h 与直线／120 km/h 分别持有独立的五阶严格对比配方，首轮数值均为
`rtol/q/v/z = 1e-9/1e-9/1e-8/1e-7 N`；同速直线—曲线对比不静默改变数值合同，也不据此声明吞吐
性能。后续场景仍须先增加具名身份与独立配方，不能借用既有场景身份运行。

## 比较口径

R300／60 km/h 先钉住通道与坐标，不以误差大小反向选择符号。之后每个场景同时报告：

1. 原生同时间序列：检验同一初值问题随时间的整体响应；
2. 四轴桥站位序列：确认比较未越过冻结不平顺的共同定义域，并诊断纵向推进差异；
3. SIMPACK 配置起速、ORVD 报告初速与八轮初始角速度：核对启动身份；
4. 四轴横移、摇头，以及八轮 `N/Tx/Ty`；
5. SIMPACK 与 ORVD 各自的总计算时间、推进时间和实时系数。

所有均方根、峰值和资格判断都使用未移时的原生时间键或原生站位键。最佳滞后如作诊断，只单独报告，
不得用于移动曲线、降低误差或通过资格。一次性固定步研究是唯一例外：状态不作插值，只按样本序号
把亚微秒级输出时钟偏差恢复为名义 100 Hz 时间键，且不移动响应曲线；该处理不属于稳定入口的默认
比较口径。三向力在 100 Hz 首轮中只作诊断，不以离散峰值决定成败。

## 已完成结果与后续边界

- R300／AAR5／60 km/h 已用 direct SLV 与当前 ORVD 私有配方各运行 30 s；两侧初始四轴站位完全一致，
  八轮前向角速度最大差为 `7.11e-15 rad/s`，未施加时移。
- SIMPACK SBR 为 binary64、`3001` 个 10 ms 样本；八路电机转矩逐点为零。ORVD 为 `60001` 个
  0.5 ms 样本。宏观横移 RMS 差为 `0.0103–0.0231 mm`，摇头 RMS 差为
  `0.0151–0.0226 mrad`。
- 本轮工件位于外置实验数据根
  `ORVD_Assist/irw-passive-multispeed-simpack-slv-comparison-20260821/stage0-r300-frozen-aar5-60kmph-30s/`；
  结果已经人工签收并写回仓库根 README。
- 直线/AAR5/80 km/h 已完成被动 30 s 对拍。两侧初始四轴站位完全一致，八轮前向角速度
  最大差仍为 `7.11e-15 rad/s`，纵向里程基本同步；两侧都在冻结 AAR5 的 `160 m` 淡入起点之前
  出现明显的轮缘—钢轨限界振荡，但 ORVD 更早形成振荡，因此同时间横移 RMS 差为 `4.306–6.822 mm`、摇头 RMS 差为
  `1.397–5.481 mrad`。这不是已闭合结果；两端都在 AAR5 开启前进入轮缘—钢轨限界作用下的
  横向往复碰撞，但 ORVD 更早起振，初始微扰差异被迅速放大。
- 该直线工况的 SIMPACK 积分为 `52.58 s / 0.571×`，完整进程为 `60.07 s / 0.499×`；ORVD 严格
  对比配方推进为 `112.02 s / 0.268×`，完整进程为 `119.36 s / 0.251×`。这些数字如实记录本次
  对比口径，不构成性能配方推荐。
- 该直线工况的工件位于外置实验数据根
  `ORVD_Assist/irw-passive-multispeed-simpack-slv-comparison-20260821/stage1a-straight-frozen-aar5-80kmph-30s/`；
  其中 t9—t10 收紧诊断表明起振时刻与原生相位未收敛，而限界振荡包络和碰撞冲量分布较稳定。
- R600/AAR5/80 km/h 已完成被动 30 s 对拍。两端初始四轴站位完全一致，八轮前向角速度
  最大差为 `2.65e-7 rad/s`，八路驱动转矩逐点为零。原生同时横移 RMS 差为
  `0.0241–0.0292 mm`，摇头 RMS 差为 `0.0306–0.1159 mrad`，未施加时移。八轮在两端全程均未失去轮轨接触。
- 该 R600 工况的 SIMPACK 积分为 `50.04 s / 0.600×`，完整进程为 `58.19 s / 0.516×`；ORVD 推进为
  `63.02 s / 0.476×`，完整进程为 `70.55 s / 0.425×`。SIMPACK 在启动和测量阶段均对八个一系
  固定拉杆报告转角超过 10° 的小角近似告警；求解正常完成，该告警保留为本工况的模型边界。
- 该 R600 工况的工件位于外置实验数据根
  `ORVD_Assist/irw-passive-multispeed-simpack-slv-comparison-20260821/stage1b-r600-frozen-aar5-80kmph-30s/`。
- 直线/AAR6/120 km/h 已完成被动 8 s 对拍。两端初速与四轴初始站位一致，八轮前向角速度
  最大差为 `3.98e-7 rad/s`，八路驱动转矩逐点为零；末端前轴站位约 `276.57 m`，没有越出
  AAR6 的 `300 m` 共同定义域。AAR6 淡入后两端均进入轮缘—钢轨限界振荡，独立响应幅值接近，
  但 ORVD 约早 `0.21–0.52 s` 达到 `1 mm` 横移，随后相位分离。原生同时横移 RMS 差为
  `10.073–14.623 mm`，摇头 RMS 差为 `2.112–11.629 mrad`；相同站位比较不能消除差异。
- 该直线工况两端均出现短时零接触样本；ORVD 的最长连续段为 `18 ms`，SIMPACK 在 100 Hz 输出钟上
  观测到的最长连续段为 `40 ms`。这只表明限界碰撞工况中存在间歇失联，不能用两种采样频率直接
  比较失联持续时间或接触力峰值。
- 该直线工况的 SIMPACK 积分为 `14.39 s / 0.556×`，完整进程为 `18.20 s / 0.439×`；ORVD 推进为
  `36.64 s / 0.218×`，完整进程为 `38.47 s / 0.208×`。工件位于外置实验数据根
  `ORVD_Assist/irw-passive-multispeed-simpack-slv-comparison-20260821/stage2a-straight-frozen-aar6-120kmph-8s/`。
- R1000/AAR6/120 km/h 使用同一 IRW 被动零转矩、中心线超高及完整横向／垂向冻结不平顺，
  120 km/h 初速，完成了四条 30 s 数值路径：ORVD 五阶严格配方，以及 SIMPACK 定步长
  FixBDF2 `62.5 µs`、DOPRI5、SODASRT2＋`Meet output points`。默认 SODASRT2 此前在
  `1.13 s` 的终止依赖求解器设置，不能据此判定该物理工况不可积分。
- 该四路径研究由一次性脚本执行；其求解器变体工件没有闭合为“Git 修订＋完整模型差异”，因此本节
  只记录已观察到的响应，不纳入上述稳定编排入口或正式资格。
- 比较只保留所有轴桥均未越过 `300 m` 上边界的 100 Hz 样本 `0–8.71 s`，共 `872` 点；`8.72 s`
  时前轴已经越界。起步时后两轴的负站位沿两端共同的无激励直线延长。固定步路径不插值状态量，
  提取时按样本序号重建 100 Hz 名义时间键，原始时间最大修正 `0.916 µs`；其余路径保留原生时间键，
  各路径都未移动响应曲线。固定步与 DOPRI5 在 `0–8 s` 的四轴横移 RMS 差不超过 `0.0024 mm`、摇头 RMS
  差不超过 `0.0082 mrad`；SODASRT2 相对另两条 SIMPACK 路径的对应上界为 `0.0113 mm` 和
  `0.0539 mrad`。ORVD 相对三条 SIMPACK 路径的横移 RMS 差为 `0.051–0.075 mm`，摇头 RMS
  差为 `0.062–0.300 mrad`。
- 八轮逐轮接触斑汇总三向力 RMS 差为 `N=1.80–2.10 kN`、`Tx=0.429–0.457 kN`、
  `Ty=0.716–0.782 kN`。SODASRT2 路径的法向力瞬时最大差为 `97.25 kN`，明显高于另两条
  路径约 `50 kN`；100 Hz 接触切换峰值仍只作诊断，不据此宣称力峰精确闭合。
- ORVD 推进为 `47.13 s / 0.637×`，完整进程为 `55.68 s / 0.539×`；SIMPACK SODASRT2
  推进为 `35.80 s / 0.838×`，完整进程为 `43.30 s / 0.693×`；DOPRI5 推进为
  `765.47 s / 0.039×`。既有定步长 SBR 没有随本轮保存可审计的独立计时，因此不为它补写
  实时系数。工件与逐算法两线图位于
  `ORVD_Assist/irw-passive-r1000-aar6-120kmph-solver-comparison-20260821/`。各路径使用的线程数和
  输出密度不同，这些数字是现行运行的实测墙钟，不是等核、等输出负担的算法基准。
- 本工况为 ORVD 在高速度、曲线与完整 AAR6 联合作用下的数值稳定性和宏观响应精度提供了较强的
  单工况证据；三条 SIMPACK 积分路径也降低了结论依赖单一参考求解器的风险。但本轮没有做 ORVD
  自身的容差、阶数或物理扰动消融，不能据此宣称广义鲁棒性；力峰尚未资格化；`0.539×` 的端到端
  实时系数也不构成实时性能声明。速度方面，ORVD 显著快于 DOPRI5，但慢于调好的 SODASRT2。
- 正式轮轨力输出频率尚未冻结；宏观 100 Hz 对拍可以先行，高频资格随后单独裁决。
- R800/100 km/h 及更高速度场景的逐实例积分配方尚未建立或验证。
- R800/100 km/h、AAR6/160 km/h 和 ERRI low/200 km/h 不阻塞最小闭环；先闭合 60、80 与 120 km/h。

在以上未完成项关闭之前，本示例不宣称多速度跨求解器一致、不宣称新的实时性能，也不把其余速度
场景写入仓库根 README 的现行动力学图和数值。
