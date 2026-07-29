# tools/drake_source_boundary

开发期工具，不是交付物。产品构建不依赖它，`BUILD_TESTING=OFF` 时连 Python 都不查找。

| 文件 | 作用 |
|---|---|
| `calculate_required_drake_source_closure.py` | 从处置清单与 Drake 源码现场计算准入闭包 |
| `verify_source_closure_analyzer.py` | 用现造的合成源码树检验上面那支分析器 |

只用 Python 3 标准库，不引第三方包。

## 分析器做什么、不做什么

**做**：从清单声明的候选目录中逐文件核对分类，以其中的 `vendor` 文件为起点，跟随
Drake include、头文件的同名实现翻译单元、以及清单显式声明的拆分实现关系，算出传递
闭包。候选漏分类、闭包触达未分类或禁入文件、准入支撑未被候选触达，均报告并返回非零。

**不做**：不调用编译器、链接器或 `nm`，不解析符号，不声称准入集合能链接。触达一个头
文件不等于需要它里面的每个符号；链接完整性是编译才能回答的问题（G11），读 include
回答不了。

不运行预处理器，条件编译不求值，因此已识别的字面 include 是**过近似**。宏或其他无法
解析的 include 操作数直接失败，不会被当成“没有依赖”略过。

## 用法

合成树自检（CTest 常驻注册，不需要 Drake；启用测试时 Python 3.10+ 是必需工具）：

```bash
python3 tools/drake_source_boundary/verify_source_closure_analyzer.py
```

对真实 Drake 源码克隆算闭包（需要带 `.cc` 的源码树；安装树只有头文件，不够用）：

```bash
python3 tools/drake_source_boundary/calculate_required_drake_source_closure.py --drake-source-root <Drake 源码克隆> --disposition-ledger external/drake_mbtree/SOURCE_DISPOSITION.txt --require-source-commit
```

## 四种违规的区别

| 报告 | 含义 | 怎么处理 |
|---|---|---|
| `FORBIDDEN EDGE` | 边界画了，且被跨越 | 改源码或改准入集合 |
| `UNCLASSIFIED EDGE` | 这里的边界从没画过 | 有人得做决定，写进清单 |
| `UNMET DEPENDENCY` | 边界画了——明确不 vendor——但准入代码仍需要它 | 要么改决定，要么改准入集合 |
| `UNREACHED ADMISSION` | 文件被标为 vendor，但没有候选文件需要它 | 从准入清单删除，不复制无用支撑 |

四者分开报，是因为要做的事完全不同。把「没决定」「决定了但没兑现」和「根本没人需要」
报成同一句，读的人无从知道该补决定、改实现还是删掉多余准入。

## 真实运行当前必然返回非零，这是正确结果

本阶段验收的是**分析器能发现并拒绝边界违规**，不是边界已经干净。当前真实闭包报告：

- **1 条禁入边**：`element_collection.cc → deformable_body.h`（include 加对
  `DeformableBody` 的显式实例化）。由 G15 在 vendor 时消除，边界最终干净由 G19 把关。
- **12 条未满足依赖**，它们不是噪声，而是一份精确的剩余工作清单，每条都指向某个 Goal：

  | 未满足依赖 | 由谁解决 |
  |---|---|
  | `common/{autodiff,default_scalars}.h`、`common/symbolic/expression.h`、`math/autodiff{,_gradient}.h` | G16 删除非 double 标量路径。这五个文件的闭包合计 69 个文件，全部是 G16 要删掉的消费者所需——搬进来再删是白搬 |
  | `systems/framework/{context,scalar_conversion_traits}.h`、`multibody/tree/{multibody_tree_system,parameter_conversion}.h` | G20–G28 用 ORVD 自己的运行时契约替换 systems 层 |
  | `math/fast_pose_composition_functions.h` | G12 明确不准入 Highway；G15 按数学定义独立实现四个组合函数并接入，G17 编译验证。Drake 的实现会把 Highway 派发代码静态编入产品，不是独立动态库 |
  | `common/text_logging.h` | G15：`unit_inertia.cc:4` include 了它却从不使用，复制时删掉这一行 |
  | `common/is_approx_equal_abstol.h` | G16：与未消费的 quaternion 比较/速率 API 一起删除 |

  随着这些 Goal 完成，对应的条目应当消失。**条目消失本身就是完成证据**，比任何计数门都
  可靠：它是从源码现场重算出来的，不是记下来的。

只有 **include 边**会产生未满足依赖。仅经同名实现关系抵达一个清单已明确不 vendor 的
`.cc`，不算矛盾——上游有不少 `.cc` 只是为了证明同名头自洽，一个符号都不定义；把它们
一并报出来会把真正的未满足依赖埋掉。

因此真实闭包**不注册为普通 CTest**，也不用 `WILL_FAIL`、`|| true`、忽略禁入边或虚拟
删边把它伪装成通过——那样做等于把唯一能报警的东西关掉。
