# INT-07 等误差资格协议与串行基线

状态：INT-07A 已落位并保留；INT-07B 自 2026-08-24 起暂停。本协议冻结比较坐标、资格工件和串行
正确性 oracle；独立正确性审查后的近奇异线性求解资格与严格浮点构建门已经闭合。本协议仍不授权
Radau5 性能排名、并行 Jacobian 候选保留、默认后端切换或研究后端准入。

## 1. 权威边界

INT-07 不把相同 tolerance 数字、较少步数或一次较快墙钟当作公平比较。一次可复核比较由四部分共同
确定：

1. 本协议定义误差、事件和完成门；
2. `tools/dynamics_qualification/int07a_serial_comparison_manifest.json` 以 `schema_version=2`、
   `manifest_identifier=int07a_serial_baseline_v2` 冻结场景、资产相对路径、时钟、状态布局、八个闭集
   case、串行执行身份和严格浮点语义；
3. runner 发布 `metadata.json`、`continuous_states.tsv`、`observations.tsv`、`contact_patches.tsv` 和
   `performance.json`；
4. `run_qualification_with_metrics.py` 在仓外记录 revision、二进制、编译器、硬件、CPU affinity、完整
   argv、OpenMP 环境、进程墙钟、CPU 时间和峰值内存。

轨迹、计时、执行 identity、哈希和比较报告不进入 Git。manifest 不写死当前未提交工作树 revision，
也不以一个顶层输入哈希冒充传递资产闭包；实际 revision、manifest identifier 和每个 artifact 的
canonical input paths 必须一起出现。禁止事后原地修改 manifest 来迁就结果；需要改变预算或场景时应
增加 schema/manifest 版本，并把旧结果留在原身份下。

## 2. INT-07A 冻结场景

| 场景 | runner | 连续窗口 | 共同观察时钟 | 默认 CVODE recipe |
|---|---|---:|---:|---|
| GZ18 直线＋AAR6、60 km/h 被动 | `orvd_gz18_dynamics_qualification` | `20 s` | `500000 ns` | BDF2 |
| IRW R300＋AAR5、60 km/h 被动 | `orvd_irw_passive_scenario` | `30 s` | `500000 ns` | BDF5 |

两者都从各自随包 60 km/h 已解析初态的 `t=0` 独立开始，不续跑、不拼接。GZ18 R300/AAR5 `16 s`
和 IRW 100 Hz 受控 `30 s` 不进入本阶段；它们连同四个根 README 完整工况的最终 CVODE 对标属于
INT-08。INT-07A 不修改根 README 或既有 SIMPACK 结论。

每个场景只允许下列八个完整 case：

- `scenario_default_cvode_{coarse,nominal,fine,reference}`；
- `radau5_{coarse,nominal,fine,reference}`。

四档相对场景配方的缩放固定为 `10 / 1 / 0.1 / 0.01`。case 同时决定 backend 和 tier；不得拆成两个
独立字符串旋钮，也不得把 Newmark/Zhai 占位加入闭集。

两场景另冻结共同平滑状态窗 `[0,10000000] ns`。该窗内每个具名轮轨接口的
`contact_patch_count` 必须是正整数，在单条 run 内逐样本恒定，并且在两条 reference 与每条待判定
candidate 轨迹之间逐接口完全一致；任一条件不满足时平滑状态门直接失败，不能另挑结果更好的窗口。
q/v/z 的资格 RMS 和最大范数只在这个预登记窗口上计算；完整 `20/30 s` 状态误差仍可报告，但只作
非光滑诊断。
长窗时钟和预算在本阶段预登记；若恢复后续资格，实际 16-run 矩阵及裁决才从 INT-07B 开始。根
README 四工况最终收口仍属于 INT-08。

## 3. 无损连续状态工件

被动资格 runner 必须直接把已经用于 observation replay 的 dense state matrix 写成
`continuous_states.tsv`，不得为写文件增加积分 stop、RHS 求值或第二条积分路径。格式固定为：

```text
sample_index  time_nanoseconds  time_seconds  q.0 ... q.(nq-1)  v.0 ... v.(nv-1)  z.0 ... z.(nz-1)
```

- 分隔符为 tab，浮点按足以 round-trip `double` 的精度写出；
- `(sample_index,time_nanoseconds)` 是唯一 join key；`time_seconds` 只作审计；
- 首行必须与两后端收到的共同已解析初态逐值相同；末行必须与 accepted terminal state 逐值相同；
- metadata 必须给出 `nq/nv/nz`、`[q;v;z]` 布局、文件名和 join key；
- 所有数值必须有限，行数必须由整数纳秒时钟唯一确定。

`observations.tsv` 仍是车型物理响应宽表，`contact_patches.tsv` 仍是一时刻一接口内的逐斑表。patch
ordinal 只在一次求值内有效，绝不是跨时刻接触斑身份。

## 4. configuration-aware 的状态误差

原始 `q` 含自由体 quaternion 和 Ball-RPY 关节，不能全部逐项欧氏相减。离线比较器必须加载同一机械
拓扑。每个 declared free body 的原始四元数＋平移七元组改写为七个误差坐标：

1. `log(R_reference^T R_candidate)` 的最短三维旋转向量，角度取 `[0,pi]`，因此 `q` 与 `-q` 等价；
2. 两个 quaternion 范数之差一维；
3. 世界坐标下平移差三维。

Ball-RPY 的具名 q-range 必须从同一已加载车型拓扑取得，不能靠 `nq` 或 JSON 出现顺序猜测；按模型
合同以 `R_FM=Rz(yaw) Ry(pitch) Rx(roll)` 重构旋转矩阵，再使用
`log(R_reference^T R_candidate)` 的主值三维旋转向量。其余关节位置直接相减，轮角保存连续 phase，
不按 `2*pi` 取模。这样 q 误差组维数仍与原 `nq` 一致。
构造旋转矩阵前必须分别归一化有限且非零的 quaternion；零或非有限 quaternion 直接失败。同时分别报告
两条轨迹在完整存储窗上的 `max(abs(norm(quaternion)-1))`，避免姿态旋转看似一致却掩盖范数漂移。
reference 每条 run 的该值必须不超过 `1e-6`，参与等误差选择的 coarse/nominal/fine run 必须不超过
`1e-4`。`v` 和 `z` 按冻结状态序逐分量比较。

归一化尺度不得依赖正在受评的 candidate。对共同样本 `k` 上的普通 q/v/z 误差坐标 `i`：

```text
scale[i,k] = nominal_absolute_tolerance(group)
             + nominal_relative_tolerance
               * max(abs(cvode_reference[i,k]), abs(radau5_reference[i,k]))
```

自由体和 Ball-RPY 的旋转向量、quaternion 范数差使用 `q_nominal_atol + nominal_rtol`；自由体平移
使用 q 组普通尺度。candidate 到每条 reference 的逐点误差必须先使用同一 `scale[i,k]` 独立归约；
分别对 q、v、z 报告平滑窗内全部“共同样本 × 误差坐标”的 dimensionless RMS 和 global maximum，
再对每个已归约 scalar 取两条 reference 结果的较大值。不允许先把 reference 轨迹或逐点误差混合后
再归约；不把 q/v/z 三组混成一个总分，不移时、不滤波、不去均值。

## 5. 双 reference 门

`scenario_default_cvode_reference` 与 `radau5_reference` 都只是独立 reference candidate。禁止平均二者，
禁止把默认 CVODE 输出定义成真值，也禁止在不一致时临时选择看起来更顺眼的一条。

进入 tier 比较前必须同时满足：

1. 两条 reference 的初态、整数时钟、资产语义身份和状态布局完全一致；
2. 平滑窗每个具名接口的 patch count 为正、run 内恒定且两条 reference 逐接口一致；两条 reference
   的 quaternion norm defect 各不超过 `1e-6`；
3. q、v、z 各自的 dimensionless RMS 不超过 `0.1`，global maximum 不超过 `1.0`；
4. 严格按 `CVODE nominal -> CVODE fine -> CVODE reference` 和
   `Radau5 nominal -> Radau5 fine -> Radau5 reference` 配对；每个后端的 `fine -> reference` RMS
   不大于它的 `nominal -> fine` RMS；
5. 第 6 节的接触事件、物理包络和端点残差门全部通过。

通过后两条 reference 形成一个 reference band。任一 coarse/nominal/fine candidate 的每项误差定义为
它到两条 reference 距离的较大值。candidate 的 q、v、z 各组必须同时满足 dimensionless RMS
不超过 `1.0`、global maximum 不超过 `10.0`，否则该 tier 不具资格；这条状态门与第 6 节物理门是
并列条件，不能以物理观测相近掩盖内部状态误差。reference 门失败时，所有墙钟都只能作为诊断；不得产生 speedup、
赢家或保留并行候选的结论。若 `reference` 仍不够严格，应先版本化增加更严闭集 tier，而不是静默改变
现有 `reference` 含义。

## 6. 非光滑接触与物理响应

两场景均以 `0.5 ms` 冻结采样钟发布接触证据。它是明确的 sampled-event 资格，不冒充连续事件根定位，
也不声称看见短于一个 tick 的接触变化。接触事件按完全相同的具名 interface 集合上
`contact_patch_count > 0` 的 on/off 转换定义；事件时刻是布尔值改变后的第一个样本时刻。比较必须
fail closed：`t=0` 的逐接口初始 on/off vector 完全一致；每个 interface、每个极性的事件数相同；
任何 unmatched event 直接失败；其余事件只按 `(interface, polarity, ordinal)` 匹配。对应事件允许相差
一个 tick；不得新增采样可见的持续失联。一个非末端零接触段从第一个 off 样本到下一 on 样本计时，
因此单独一个 off 样本是可见事件且时长为 `0.5 ms`。资格窗结束时仍 off 的区间按右删失处理，只记录
截至末样本的观测下界，不虚构“末样本后一个 tick”的 on 事件或完整时长。完整区间的最长零接触时长
差不得超过一个 tick。事件联合 `±1 tick` guard 内不做逐点绝对差裁决，但样本仍进入固定窗口的
包络和冲量。

对从 `t=0` 对齐的闭区间 `10 ms` bin（恰含 21 个 `0.5 ms` 样本），逐具名 interface 比较总支持力
Q、总 N 与 carrier Track-T 下总 Fx/Fy/Fz。相邻闭 bin 共享边界样本；每个 interface/channel/bin 以
含两端点的复合梯形积分除以 `10 ms` 得到 impulse-equivalent mean，同时取样本 minimum 和 maximum。
candidate 到每条 reference 的差先分别在全部 interface/channel/bin/output 上归约 RMS 和 global max，
再逐 scalar 取两者较大值。最大-N主斑 Tx/Ty 因 patch selector 可能随微扰跳变，只作为诊断 channel，
不进入 eligibility gate。所有 contact carrier 的横移／摇头按同一整数时刻作绝对差，在包含首末端点的
完整资格时钟上取“全部 carrier × 全部样本”的 global max；同样先分别比较两条 reference，再取两个
scalar 的较大值。不把 carbody/bogie representative-body 列混入该门。车型预算为：

| 场景 | 横移 max | 摇头 max | 力 RMS | 力 max |
|---|---:|---:|---:|---:|
| GZ18 | `0.25 um` | `0.25 urad` | `5 N` | `50 N` |
| IRW 被动 | `2 um` | `2 urad` | `25 N` | `250 N` |

每条 run 还必须独立满足端点装配门：广义力残差和绝对 virtual-power 残差均 `<1e-7`，qdot 与 z slice
一致性均 `<1e-14`。这些残差证明装配和切片自洽，不替代时间积分误差。

## 7. 等误差 tier 与统计

每个后端分别从 coarse/nominal/fine 中选择同时满足第 5 节 candidate q/v/z 状态门、第 6 节整组物理预算
及逐 run 端点门的最粗 tier，再比较其成本。相同 tolerance 不构成等误差。若两个入选 tier 的 q、v、z
任一组 dimensionless RMS 或 hard force RMS，
有效误差之比较大值与较小值超过 `2`（较小值为零而较大值非零时视为无穷），只报告
wall-versus-error 加密梯子，不给单一“等误差速度比”，也不插值出从未实际运行的计时点。

后端中立可比统计仅为：successful internal steps、ordinary RHS、linear-solver RHS、error-test failures、
nonlinear iterations、nonlinear convergence failures、linear setups、Jacobian evaluations 和 requested
dense-FD Jacobian workers。error-test 与 nonlinear failure 分列；两者及 recoverable RHS abandonment
不得合并成通用 `rejected steps`。

## 8. 串行基线与正式计时边界

INT-07A manifest v2 冻结 `Release + orvd.strict_ieee_no_fast_math.v1 + OMP_NUM_THREADS=1 +
OMP_DYNAMIC=FALSE + OMP_MAX_ACTIVE_LEVELS=1 + OMP_NESTED=FALSE + 单核 affinity`。配置期必须拒绝
`-ffast-math`、`-Ofast`、有限数学假设及其已知等价选项；Radau5 核心另以编译宏守住最终翻译单元，
唯一编译启动器还要检查生成表达式、目录、目标、传递依赖和可读响应文件展开后的实际编译命令，并在
检查通过后亲自注入资格 runner 要求的证明宏。不执行启动器的生成器、既存启动器链及不透明的编译器
配置入口必须在构建前失败。两类 runner 的 metadata 必须记录严格浮点语义标识、两层审计、实际构建
类型和编译器身份；manifest-bound
wrapper 在成功后逐项核对，并拒绝命令行声明与二进制自报不一致。该处方是一条整套执行均串行的
correctness oracle：
CVODE 与 Radau5 的 Jacobian worker identity 都必须为 `1`，接触批也为 `1`。它用于回退、工件一致性
和 reference 诊断，`performance_decision_eligible=false`，不能代表目标多核吞吐。

`orvd.strict_ieee_no_fast_math.v1` 是浮点安全语义类别，不是跨构建逐位复现身份。它拒绝 fast-math、
finite-only 及其已知等价假设，但不钉死 contraction；编译器默认 contraction、显式
`-ffp-contract=fast/off` 与融合乘加计算均可进入该类别。因此不同编译器、版本、目标 ISA 或
contraction 选择仍是不同资格身份，证据继续绑定编译器与可执行文件身份，并按本协议数值
预算比较，不能仅因同属 v1 就要求工件跨构建逐位相同。同一二进制在不同固定槽 worker 数下的逐位
一致性仍是独立的并行确定性合同，不因该政策而放宽。

“当前 Radau5 Jacobian 串行”只表示 Jacobian worker identity 为 `1`，不等于整个 runner 串行。在未来
目标线程预算 N 下，现有 Radau5 可保持 Jacobian=1 而接触 RHS 消费相同 OpenMP 预算；该运行可以作为
INT-07C 候选前 provisional baseline，但 reference 门完成前仍不能排名。全局 OMP 值同时影响接触批和
CVODE provider，因此任何只想归因于 Jacobian 的结论都必须明确报告完整执行配置，不能把整套配置的
加速冒称单一子因子收益。

正式计时只在 reference/equal-error 门通过后开始：同一 revision、Release 二进制、主机、affinity 和
OpenMP 身份下，每个 arm 使用 fresh process 按 ABBA 顺序各运行两次，报告算术均值及 `[min,max]`；
区间重叠则结论为“无稳定收益”，不追加第三次追逐最好值。GZ18 与 IRW 分开报告，reference case 不
参加速度排名。单一积分器速度比的主计时量固定为 `performance.json` 的 `advance_wall_seconds`；wrapper
的 `process_wall_seconds` 作为端到端次级门单独报告，Radau5 candidate 的该项均值不得慢于 CVODE
baseline 均值，否则即使主计时较快也不保留候选。observation、endpoint diagnostics 与写文件分项只作
成本解释，不得替换主计时量。

## 9. INT-07A 完成门与下一阶段

INT-07A 完成要求：

- 机器 manifest `int07a_serial_baseline_v2` 严格拒绝未知字段、非闭集 case、非整数时钟、未冻结的
  平滑窗、非串行执行处方、非严格浮点语义和性能排名开关；manifest-bound wrapper 还必须在启动前
  fail closed 核对处方与实际
  OMP/affinity，并在成功后核对 exact COMPLETE、完整时钟 TSV、逐
  `(sample,interface,ordinal)` patch 声明、primary timing、全部统计字段、metadata/performance 的接触
  批与 Jacobian worker identity，以及 metadata 中 `floating_point_compilation_contract` 的语义标识、
  CMake／最终编译命令审计成功、实际 Release 身份、fast／finite-math 宏状态和与 wrapper 声明一致的
  非空编译器名称／版本；
- 16 个场景/case runner argv 可确定性展开，但测试不启动完整车辆长窗；
- GZ18 与 IRW 被动工件无损发布完整 `[q;v;z]`，并由真实短窗测试核对布局、整数时钟、初／末态与
  1/4/8/12/16/32-worker 工件一致性；
- Radau5 核心以多列测试冻结串行列顺序、逐列 recoverable 缩扰动、fatal 截止、RHS 分类和端点原子性；
- 不修改公共默认、安装 API、根 README、轮轨物理或 GPT Pro 正在审查的八份生产源码。

INT-07B 原计划实现 configuration-aware comparator 和双 reference gate，INT-07C 原计划试验后端
中立扰动 RHS 批求值层；两者当前均暂停。实／复近奇异线性系统的相对残差 oracle 与严格浮点构建门
已完成，但这只关闭恢复前置条件，不自动授权下一阶段。候选无稳定收益时按主路书完整回滚，以串行
Jacobian 的 Radau5 研究后端完成 INT-07。
