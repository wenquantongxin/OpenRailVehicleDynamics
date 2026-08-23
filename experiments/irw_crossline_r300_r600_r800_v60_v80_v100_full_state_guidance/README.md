# IRW 多曲率变速全状态导向实验

本实验展示如何只使用 OpenRailVehicleDynamics 的安装接口，将一条包含 R300、R600、R800 和末端直线的线路，与按里程变化的速度参考和全状态导向参数组合成一个完整的 100 s IRW 运行。完整日程是实验代码中的强类型常量，不进入公共控制器 JSON，也不为公共加载器增加新的配置模式。

实验是独立的安装包消费者：不进入仓库根构建、不安装、不注册为 CTest。它既可作为复杂控制编排的功能示例，也把工况特有的插值、事件事务和输出口径留在工况边界内。

R300／R600／R800 的 SIMPACK／ORVD 主动控制串行矩阵及当前进度见
[对比备忘录](SIMPACK_ORVD_R300_R600_R800_PASSIVE_NORMAL_BETTER2_SCBP_COMPARISON.md)。

## 运行身份

- 线路从 60 km/h 的直线起步，依次通过 R300、R600、R800 及末端直线；速度参考在指定里程间以 \(v^2\) 线性变化。
- 四轴速度与导向参数按各轴自己的投影站位求值；两个标量外环参数按四轴平均站位求值。
- 控制日程使用独立的计划曲率，不以接缝平滑后的瞬时几何曲率反向驱动控制器。
- 初始时刻直接计算并发布第一拍转矩；后续控制事件位于严格的 10 ms 整数时钟。
- 轴内先保留纵向共模，再由公共逐轮转矩调理器实施幅值和变化率约束。
- 机械响应与逐接触斑观测使用严格的 0.5 ms 原生时钟；完整运行固定为 100 s。
- 投影半窗为 `30 m/s × 0.01 s = 0.30 m`，与本实验的最高计划速度和控制区间对应。

[线路几何资产](../../track_library/geometries/crossline_r300_r600_r800_superelevation_2396p9m.json) 的平面曲率与中心线超高来自同一跨线模型。Bloss 过渡用既有三次 Hermite 段表示；各内部边界的 `3.0 m` 接缝窗口对应 SIMPACK 的左右各 `Lsmo/2 = 1.5 m`。

轨道不平顺采用 ORVD 当前公共 `aar5_irregularity` 合同，而不是外部跨线工程的历史随机场：门控、频带、随机相位和谱系数均以当前公共资产为准。因此，本实验复现的是线路几何与控制日程，不宣称复演外部工程的原始随机激励时序。

SIMPACK 参考模型中保留了同名物理线路供后续直接对照，但活动线路仍是原有平面—竖向组合线路；打开模型不会改变既有默认工况。

### R300 `normal` 差速基线

同一实验还提供孤立 R300、60 km/h、AAR5 的 `normal` 入口。该控制器不使用全状态外环：每根轴按自己的站位，在 `50–100 m` 将计划曲率从零线性建立到 `1/300`，再以

\[
v_w=v_0(1+\mathrm{side}\cdot0.75\kappa)
\]

生成左右轮参考，并交给相同的八轮轮速递推、纵向共模优先和逐轮转矩调理链。所有外层增益和平衡值严格为零。计划曲率属于源控制器的线路日程；100 m 后保持 `1/300`，不从轨道几何的 Hermite 段反查，也不在出口缓和中自行撤掉。

运行在首个满足四轴站位均不低于 `600 m` 的 10 ms 控制边界提交终端事件；`45 s` 只是不满足站位条件时的运行失败上限。这样四轴各自的 `100–600 m` 圆曲线响应窗口都完整存在。

### 孤立曲线 `better2` 工作点

R300、R600 和 R800 另有彼此独立的 `better2` 入口，分别使用
`r300_pd_m30_m6`、`r600_wear_champion_t343` 和
`r800_wear_champion_t853`。三套完整参数以实验内强类型常量保存，完整跨线日程与
孤立曲线入口引用同一份数值，不新增公共控制器资产或配置格式。

孤立曲线不带入前一半径的工作点：每根轴在自己的 `50–100 m` 入口区间内，将计划
曲率、外层增益和平衡值从零连续建立到目标工作点；滤波常数从入口默认值连续过渡。
`100–600 m` 保持目标参数，最后一根轴到达 `600 m` 后结束。

## 构建与运行

先完成 ORVD 的 Release 安装，再从独立目录配置本实验：

```bash
cmake -S experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance \
  -B <实验构建目录> \
  -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_PREFIX_PATH=<ORVD安装前缀>;<依赖安装前缀>'
cmake --build <实验构建目录> -j
```

可执行目标分别为：

```text
orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance
orvd_irw_r300_aar5_v60_normal_differential_wheel_speed
orvd_irw_r600_aar5_v80_normal_differential_wheel_speed
orvd_irw_r800_aar5_v100_normal_differential_wheel_speed
orvd_irw_r300_aar5_v60_better2_full_state_guidance
orvd_irw_r600_aar5_v80_better2_full_state_guidance
orvd_irw_r800_aar5_v100_better2_full_state_guidance
orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_full_state_guidance
orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_full_state_guidance
orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_full_state_guidance
```

如需构建同工况的 SIMPACK Realtime 对拍入口，在独立实验构建中显式开启可选目标：

```bash
cmake -S experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance \
  -B <实验构建目录> \
  -DCMAKE_BUILD_TYPE=Release \
  -DORVD_ENABLE_SIMPACK_REALTIME_COMPARISON=ON \
  -DORVD_SIMPACK_ROOT=<SIMPACK安装目录> \
  '-DCMAKE_PREFIX_PATH=<ORVD安装前缀>;<依赖安装前缀>'
cmake --build <实验构建目录> \
  --target orvd_irw_r300_aar5_v60_normal_simpack_realtime \
           orvd_irw_r600_aar5_v80_normal_simpack_realtime \
           orvd_irw_r800_aar5_v100_normal_simpack_realtime \
           orvd_irw_r300_aar5_v60_better2_simpack_realtime \
           orvd_irw_r600_aar5_v80_better2_simpack_realtime \
           orvd_irw_r800_aar5_v100_better2_simpack_realtime \
           orvd_irw_r300_aar5_v60_scbp_recorded_wear_champion_simpack_realtime \
           orvd_irw_r600_aar5_v80_scbp_recorded_wear_champion_simpack_realtime \
           orvd_irw_r800_aar5_v100_scbp_recorded_wear_champion_simpack_realtime -j
```

运行器只接收安装数据根和一个尚不存在的输出目录：

```bash
OMP_NUM_THREADS=16 <实验构建目录>/\
orvd_irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance \
  <ORVD安装前缀>/share/OpenRailVehicleDynamics \
  tmp/irw_crossline_full_state_guidance_run
```

R300 `normal` 首跑使用同样的两个参数：

```bash
OMP_NUM_THREADS=16 <实验构建目录>/\
orvd_irw_r300_aar5_v60_normal_differential_wheel_speed \
  <ORVD安装前缀>/share/OpenRailVehicleDynamics \
  tmp/irw_r300_normal_differential_wheel_speed_run
```

R600、80 km/h 的 `normal` 入口只更换实验内的速度、计划半径和线路资产；80 km/h
启动由随包 60 km/h 已解析状态同比缩放纵向速度与八轮显式转速，其余状态不变：

```bash
OMP_NUM_THREADS=16 <实验构建目录>/\
orvd_irw_r600_aar5_v80_normal_differential_wheel_speed \
  <ORVD安装前缀>/share/OpenRailVehicleDynamics \
  tmp/irw_r600_normal_differential_wheel_speed_run
```

R800、100 km/h 使用同一入口合同，只替换速度、计划半径和线路资产：

```bash
OMP_NUM_THREADS=16 <实验构建目录>/\
orvd_irw_r800_aar5_v100_normal_differential_wheel_speed \
  <ORVD安装前缀>/share/OpenRailVehicleDynamics \
  tmp/irw_r800_normal_differential_wheel_speed_run
```

三条 `better2` ORVD 入口与 `normal` 使用相同的两个 CLI 参数。例如 R300：

```bash
OMP_NUM_THREADS=16 <实验构建目录>/\
orvd_irw_r300_aar5_v60_better2_full_state_guidance \
  <ORVD安装前缀>/share/OpenRailVehicleDynamics \
  tmp/irw_r300_better2_full_state_guidance_run
```

SIMPACK 编排脚本在参考模型旁创建临时模型，只替换活动线路、初速度和求解设置；运行后
删除临时模型及其副产物，不改写参考模型。Realtime 直调要求内部积分点与交换点严格
一致，因此这里冻结 `FIXBDF`、`1 ms` 内部步长和 `1 ms` 交换时钟；仅设置
`meet output points` 不能避免变步长积分器越过请求时刻。

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
run_irw_single_curve_guidance_simpack_realtime_comparison_arm.py \
  <实验构建目录>/orvd_irw_r600_aar5_v80_normal_simpack_realtime \
  <ORVD数据根>/vehicle_library/irw/reference_models/simpack/main_model/irw_vehicle.spck \
  <ORVD数据根>/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json \
  tmp/irw_r600_normal_simpack_realtime_run \
  --active-track '$Trk_Curve_R600m_80kmph' \
  --initial-speed-kilometres-per-hour 80
```

R800 的 SIMPACK 侧命令为：

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
run_irw_single_curve_guidance_simpack_realtime_comparison_arm.py \
  <实验构建目录>/orvd_irw_r800_aar5_v100_normal_simpack_realtime \
  <ORVD数据根>/vehicle_library/irw/reference_models/simpack/main_model/irw_vehicle.spck \
  <ORVD数据根>/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json \
  tmp/irw_r800_normal_simpack_realtime_run \
  --active-track '$Trk_Curve_R800m_100kmph' \
  --initial-speed-kilometres-per-hour 100
```

同一编排脚本也运行 `better2` 入口。例如 R300 只需选择对应可执行文件：

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
run_irw_single_curve_guidance_simpack_realtime_comparison_arm.py \
  <实验构建目录>/orvd_irw_r300_aar5_v60_better2_simpack_realtime \
  <ORVD数据根>/vehicle_library/irw/reference_models/simpack/main_model/irw_vehicle.spck \
  <ORVD数据根>/vehicle_library/irw/drive_torque_conditioners/irw_reference_wheel_drive_torque_conditioner.json \
  tmp/irw_r300_better2_simpack_realtime_run \
  --active-track '$Trk_Curve_R300m_60kmph' \
  --initial-speed-kilometres-per-hour 60
```

完成两侧运行后，在严格重合的 `1 ms` 原生时钟上比较；脚本不插值、不搜索滞后，且每个
子图只绘制 ORVD 与 SIMPACK 两条线：

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
compare_irw_single_curve_simpack_realtime_and_orvd.py \
  tmp/irw_r600_normal_differential_wheel_speed_run \
  tmp/irw_r600_normal_simpack_realtime_run \
  tmp/irw_r600_normal_simpack_orvd_comparison \
  --title 'IRW R600 + AAR5, 80 km/h normal baseline'
```

R800 使用相同比较程序，只替换两侧运行目录与标题。比较程序只读取轴桥运动学、轮端
转矩、站位差分纵向速度和三向接触力；蠕滑率与磨耗不进入本组 SIMPACK／ORVD 对拍。

运行失败时，未完成的临时目录会被精确移除；成功后所有文件通过一次目录重命名共同发布，并写入 `COMPLETE`。运行工件位于 `tmp/`，不被 Git 索引。

## 输出与绘图

运行目录包含：

- `control_events.tsv`：初始化、周期和终端事件的工作点、原始请求、共模探针、最终请求、实际转矩和控制记忆；
- `observations.tsv`：四轴站位、纵向速度、横移、摇头、八轮转速与实际转矩，以及轮级接触汇总；
- `contact_patches.tsv`：逐接触斑三向力、蠕滑率、接触位置和载荷；
- `endpoint_diagnostics.tsv`：每个控制区间端点的导数、广义力与虚功残差；
- `performance.json` 和 `metadata.json`：积分计数、运行口径、资产身份与执行元数据。

绘图脚本先核对 `COMPLETE`、严格 0.5 ms 时钟、样本序号和有限性，再从逐斑切向力与蠕滑率计算

\[
W = |T_x\gamma_x| + |T_y\gamma_y|.
\]

调用方式：

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
plot_irw_guidance_experiment_response.py \
  tmp/irw_crossline_full_state_guidance_run
```

绘制 R300 `normal` 时可显式设置图题和文件前缀：

```bash
python experiments/irw_crossline_r300_r600_r800_v60_v80_v100_full_state_guidance/\
plot_irw_guidance_experiment_response.py \
  tmp/irw_r300_normal_differential_wheel_speed_run \
  --title 'IRW R300 + AAR5, 60 km/h normal baseline' \
  --file-prefix irw_r300_aar5_v60_normal
```

脚本生成三张图：第一／第三轴的横移、摇头、左轮转矩和左轮瞬时磨耗数 2×2 图；第一／第三轴纵向速度图；全车瞬时磨耗数及其累计积分图。数值结果属于本实验自身，不包装成公共产品资格。

SIMPACK 对拍脚本另生成四张图：第一／第三轴横移与摇头、左轮转矩、纵向速度，以及
左轮三向接触力。跨软件比较明确不包含蠕滑率或磨耗，也不从三向力推测这些量。

## 端到端复核

一次完整 100 s 运行发布了 `200001` 个 0.5 ms 机械观测、`10001` 个控制事件和 `1666471` 个接触斑记录。初始化、`9999` 个周期事件与终端事件严格落在整数控制时钟；四轴投影站位始终单调前进，单个机械输出间隔内的站位增量为 `8.32–13.94 mm`，未发生投影窗口失配。

第一轴依次在约 `5.40 s`、`35.33 s`、`41.13 s`、`63.60 s`、`69.30 s`、`76.50 s` 和 `78.30 s` 越过 `100 m`、`600 m`、`708 m`、`1208 m`、`1346.9 m`、`1546.9 m` 和 `1596.9 m` 的日程边界；其余三轴按自身站位随后越过同一组边界。100 s 末四轴站位为 `2179.86–2199.86 m`，已经覆盖 R300、R600、R800 及末端直线四类区段。最大广义力残差为 `2.53e-8`，最大虚功残差绝对值为 `1.66e-9 W`。

这些数字用于确认实验编排、整数时钟、投影连续性和输出事务闭合；运行工件与图不进入 Git，需要时由上述命令重新生成。
