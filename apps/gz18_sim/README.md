# apps/gz18_sim

GZ18 刚性轮对仿真可执行入口。**里程碑**：M3（CLI/快照/输出契约收口）。

链接 `orvd::vehicles` + `orvd::systems` + `orvd::integrators`（CVODE）+ `orvd::forces` + `orvd::equilibrium`。

CLI 须复现仓库 `rigid_wheelset_sim` 的契约要点：正式路径强制 `--startup_snapshot PATH`
（或显式 `--runtime_init`），两者互斥；输出 CSV/NPZ/JSON 三套 + 接触诊断。详见设计基线 §C（遗漏层）。
