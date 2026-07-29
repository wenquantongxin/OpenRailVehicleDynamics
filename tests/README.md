# tests

| 目录 | 内容 |
|---|---|
| `unit/` | 逐模块单元测试（Context 版本失效、事件稳定序、拒步回滚、力元闭式等语义点）。 |
| `parity/` | Drake 差分预言机台架 —— 三层金标 + 有序轨迹主指标。见其 [README](parity/README.md)。 |
| `golden/` | 冻结的金标向量与有序轨迹夹具（M0 产出，二进制资产）。 |

测试框架待定（建议 GoogleTest 或 Catch2，外置）。CI 上 parity 构建强制关闭 `-ffast-math`/`-march=native`，
并跑 OMP 1/2/4/8 线程逐位试验。
