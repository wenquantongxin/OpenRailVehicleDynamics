# tools

预实施占位。规划中的脚本（多为 M0/M3）：

| 脚本 | 阶段 | 用途 |
|---|---|---|
| `freeze_oracle.*` | M0 | 冻结 oracle 身份：内容哈希清单、`libdrake.so` 哈希、SUNDIALS config、CPU 特性掩码、OMP/浮点环境、编译选项。 |
| `archive_snapshot.*` | M0 | 把 gitignore 掉的 `*.npz` 启动快照抢救为版本化二进制资产。 |
| `scan_no_drake.*` | M2 | 零 Drake 判据：链接图 + `readelf`/符号扫描 + 干净环境安装测试（不以 `ldd` 单证）。 |
| `check_vendor_allowlist.*` | M1 | vendored 树 include 禁入检查（geometry/FEM/plant）。 |
| `diff_output_contract.*` | M3 | CSV/NPZ/JSON 输出契约字段级/字节级比对。 |

语言待定（Python 或 shell）。这些是构建/CI 辅助，不进运行时产物。
