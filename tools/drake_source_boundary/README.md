# tools/drake_source_boundary

开发期工具，不是交付物。产品构建不依赖它，`BUILD_TESTING=OFF` 时连 Python 都不查找。

| 文件 | 作用 |
|---|---|
| `calculate_required_drake_source_closure.py` | 从处置清单与 Drake 源码现场计算准入闭包 |
| `verify_source_closure_analyzer.py` | 用现造的合成源码树检验上面那支分析器 |
| `compile_admitted_drake_translation_units_and_link_generated_objects.py` | 把准入集合暂存后逐个真实编译，并链接产出的对象 |
| `verify_admitted_translation_unit_compile_probe.py` | 用合成源码树检验上面那支探针 |
| `compile_landed_double_multibody_translation_units.py` | 编译**已落位**的 double-only 源码，报告编译前沿、源码层运行时依赖表面与禁忌标量符号 |
| `verify_landed_double_compile_frontier.py` | 用合成源码树检验上面那支工具 |
| `verify_landed_drake_source_provenance.py` | 对着钉死的上游现场核验落位源码的来源与许可证义务 |
| `verify_landed_drake_source_provenance_audit.py` | 用合成源码树检验上面那支审计 |
| `verify_product_source_drake_boundary.py` | 检查产品源码既不是禁入文件、也不 include 禁入文件 |
| `verify_product_source_drake_boundary_check.py` | 用合成源码树检验上面那支闸门 |

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

## 真实 Drake 源码运行

分析器读取的是固定 commit 的**未修改上游源码**，因此它会持续报告那些后来在
`external/drake_mbtree/drake/` 中通过源码裁剪解决的边。这个结果用于解释为什么需要某项
处置，不能冒充 landed 源码的进度表。

上游扫描会暴露三类已知工作：禁入的 deformable 边、G16 要删除的非 `double` 标量依赖，
以及 G20–G28 要替换的 systems/runtime 依赖。G15 对 landed 源码做的禁入边清理与位姿组合
替代不会改变未修改上游克隆的扫描结果；产品实际边界由 G19 对 landed 构建检查。

只有 **include 边**会产生未满足依赖。仅经同名实现关系抵达一个清单已明确不 vendor 的
`.cc`，不算矛盾——上游有不少 `.cc` 只是为了证明同名头自洽，一个符号都不定义；把它们
一并报出来会把真正的未满足依赖埋掉。

因此真实闭包**不注册为普通 CTest**，也不用 `WILL_FAIL`、`|| true`、忽略禁入边或虚拟
删边把它伪装成通过——那样做等于把唯一能报警的东西关掉。

## 编译探针为什么必须暂存

读 include 能说出边界**触达**什么，说不出它是否**编译**得过，更说不出它链得上。编译探针
把准入集合复制进一棵临时树，**只**把那棵树交给编译器。对着上游克隆跑会让一个已被裁掉
的文件顺手满足某个 include，报出一个边界并未挣得的成功。

其余几条同样是为了不自欺：

- 用 C++23 与真实 `-c`，不用 `-fsyntax-only`——语法成立不等于能产出对象。
- 成功必须伴随**非空**对象文件，编译器报成功却没产出同样算失败。
- 源码克隆的 HEAD 必须与处置清单声明的固定 commit 相同。
- 某个 TU 失败后继续尝试其余，一个坏文件不该掩盖其余翻译单元的状态。
- 保留编译器原始输出，不写跨编译器的正则分类器——那种解析器离说谎只差一个编译器版本。
- 链接时不建静态库、不开 LTO、不开 section GC：一个没人调用的函数引用了缺失符号，
  链接**仍然必须失败**。否则报出的"边界成立"只是因为那个符号从没被走到。
- 不落盘符号表、符号数或允许列表。今天记下的数字，明天会变成一道因错误理由而通过的门。

链接只覆盖**当次编译成功的子集**，工具会明说这一点。整个边界的全部翻译单元编译与
全对象链接由 G28 验收。

## 落位前沿工具与准入探针的分工

两支工具读的是**两种源码身份**，不可互相代用：

- 准入探针读**钉死 commit 的未修改上游克隆**，回答"处置清单声明的准入集合能不能编译"。
  它校验克隆 HEAD 与清单声明的 commit 一致，这道校验不得为了顺便读落位树而放宽。
- 落位前沿工具读**仓库里已落位的源码**，回答"double-only 手术之后真实的编译前沿在哪"。
  它不校验 commit，因为落位源码相对上游本就有记录在案的改动。

落位工具把落位树作为**唯一**的 `drake/` include 目录。任何其他 Drake 树在场——经参数、
经环境变量、或经编译器自身默认路径——都会让一个本边界故意不具备的头满足某个 include，
于是报出一个边界并未挣得的前沿。它用 `__has_include` 问编译器"看不看得见本边界没有的头"
来判断，而不是匹配路径字符串:装在意料之外位置的 Drake 恰恰是字符串匹配漏掉的那种。

符号检查用**限定名**。裸搜 `symbolic` 会命中 `Eigen::symbolic::SymbolExpr`——那是 Eigen
自己给 `Eigen::last` / `Eigen::seq` 索引表达式建立的命名空间，与 Drake 的符号标量无关；
自检里有一条专门的负控守着这件事。定义与未定义符号都看：只是**调用**了被裁构造的翻译
单元，和定义了它的一样越界。编译期就消解、根本到不了符号表的构造（`default_scalars.h`
的 include、`scalar_predicate` 分支）由另一条源码扫描负责。

在 G20–G28 完成前，对完整落位树的整轮运行会返回非零：仍需运行时的翻译单元尚未编译
得过。因此 CTest 只注册它的合成源码树自检，不用 `WILL_FAIL` 把失败伪装成绿色——一道
允许失败的测试，很快就没人再读它了。真实运行是开发期命令：

```bash
python3 tools/drake_source_boundary/compile_landed_double_multibody_translation_units.py \
    --landed-root external/drake_mbtree \
    --compiler <编译器> \
    --third-party-include-directory <Eigen include 目录>
```

临时对象放在 `TMPDIR` 指向的位置（本机指向外置卷），运行结束即消失。工具不落盘通过数、
符号表或允许列表：今天记下的数字，明天会变成一道因错误理由而通过的门。它输出的是
**源码层**运行时依赖表面，不宣称完整 ABI，也不宣称最小实现契约。

## 来源审计

`verify_landed_drake_source_provenance.py` 回答的是分发义务里的**可追溯性**:任取落位树
中的一个文件，读者能否说出它来自哪个上游仓库、哪个修订、哪个路径。账本持有这三件事实，
该工具从克隆中的**固定 Git 对象**读取上游内容，不读取可被未提交改动污染的工作区。它核验
账本的 commit 与 tag、落位路径集合和固定对象树一致；Drake 的 BSD 正文除平台换行外与固定
对象文本相同；
Apache 正文包含标准标题、全部编号章节及第 4(b) 条；固定对象中实际带 Apache 头的文件与
账本元数据一致；被修改的 Apache 文件保留原声明块，并在首个 include 之前带有 ORVD 改动声明。

**来源身份由仓库、commit 与路径组成，不用冻结哈希清单。** 哈希只说两个文件是否逐字节
相同，不说任何一个来自哪里；它在我们行使许可证授予的修改权那一刻就失效；而一份今天提交的
哈希清单，明天会因为正确的理由在错误的日子失败。路径经得起修改，这正是来源追溯需要的性质。
本地 clone 可以是镜像，工具不把其可变 remote 配置当成来源事实；仓库 URL 由账本声明。

与上游有差异**不是**本工具报错的理由：落位树经过了记录在案的 double-only 手术。它检查的
是——那些既有差异、其许可证又要求改动声明的文件，是否真的带着声明。Apache-2.0 第 4(b) 条
要求被修改的文件声明自己被改过;BSD-3-Clause 没有这一条，因此被修改的 BSD 文件只需随仓库
携带许可证正文，文件内无须任何声明。合成自检里有一条专门的负控守着这个区别。

上游克隆只经命令行给出。它很大，是别人仓库的工作副本，某台机器把它放在哪里不是本项目的
事实。CTest 只注册合成自检；真实审计是开发期命令:

```bash
python3 tools/drake_source_boundary/verify_landed_drake_source_provenance.py \
    --landed-root external/drake_mbtree \
    --upstream-clone <钉死修订的上游克隆> \
    --disposition-ledger external/drake_mbtree/SOURCE_DISPOSITION.txt \
    --git-executable "$(command -v git)"
```

## 第三方依赖按参数传入

工具只接受通用的编译器、第三方 include 与库选择参数，不硬编码任何库的路径、版本或专用
开关。隐式 include 与 library 环境变量会被清除；编译器默认搜索路径和显式第三方 include
均不得暴露另一棵 `drake/` 头目录。宿主机恰好装了某个库，不等于项目已经声明该依赖——
正式处置属于 G12。

该探针当前使用 GNU 风格的编译器命令行，是源码边界研究工具，不是产品构建接口。其 CTest
自检只在原生 UNIX、GNU 风格编译器前端下注册；跨平台产品构建由后续 CMake 目标验证。
