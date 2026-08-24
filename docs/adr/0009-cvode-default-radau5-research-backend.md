# ADR-0009：保持 CVODE 默认，增加纯 C++ Radau5 研究后端

- 状态：**Accepted**
- 决策日期：2026-08-23
- 相关：ADR-0003

## 背景

ORVD 当前唯一实现的连续状态后端是 CVODE BDF。公共构造固定最大二阶，源码树内另有显式的最大
五阶资格配方。现有 `ContinuousStateAdvancer` 已把单成功内步、稠密输出、统计、失败封锁和重初始化
从系统 accepted/candidate Context 与轮轨站位事务中分离；增加第二个数值方法不需要重新定义这些
系统语义。

Radau5 适合作为刚性整车问题的对照方法，但它尚无 ORVD 产品实现。直接替换默认后端会同时改变
数值人格和既有消费者；提前增加公共字符串或枚举又会在资格完成前承诺一个尚不可用的产品能力。
仓库历史记录中的固定步 `Radau3` 也没有足够来源证据，不能按名称由 Radau5 静默替代。

可采用的上游材料包括 UNIGE 托管的 Blake Ashby C++ 版本和 Hairer/Wanner Fortran 版本。C++ 包是
纯 C++，但采用旧式全局回调、裸所有权和整段 `Integrate()`，并已确认存在一处 Jacobian 刷新条件
反译错误；Fortran 更适合作为数值行为 oracle，而不是产品依赖。

## 决策

1. CVODE BDF 继续是唯一公共默认后端。无参数公共构造仍创建最大二阶 BDF；现有源码树最大五阶
   BDF 资格入口不变。
2. Radau5 作为显式构造的研究第二后端推进。资格完成前，它只通过源码树私有的强类型 recipe／
   factory 到达真实消费者，不增加公共字符串、环境变量、整数选择或未实现枚举值。
3. Radau5 产品实现必须是纯 C++。首版只准入三阶段五阶 Radau IIA、正向显式一阶 ODE、`M=I`、
   `double`、full-dense 数值 Jacobian、simplified Newton、实／复稠密线性系统和最近成功步稠密输出。
4. C++ donor 与官方许可的确切身份及逐文件处置只由
   [`external/radau5/SOURCE_DISPOSITION.txt`](../../external/radau5/SOURCE_DISPOSITION.txt)维护；任何
   派生源码继续作为有归属的第三方派生材料处理。
5. 官方 Fortran 只作为仓外临时 oracle。按项目负责人裁决，仓库只记录官方 URL，不提交 Fortran
   源码，也不记录其 SHA-256；Fortran 不进入 CMake、产品 target、安装或离线产品源码包。
6. 完整数值行为由
   [`INT_01_RADAU5_SOLVER_CONTRACT.md`](../planning/integrator_migration/INT_01_RADAU5_SOLVER_CONTRACT.md)
   冻结。C++ donor 与该合同或官方 Fortran 控制流冲突时，不继承已知 donor 漂移。
7. Newmark、Zhai 和历史 Radau3 不借本决策预建接口或实现。Radau5 的研究身份不能回填这些方法的
   历史结果。
8. Radau5 资格完成后，只有出现真实外部研究消费者时才另行裁决是否安装一个只列真实后端的强类型
   选择接口；这项后续裁决也不得改变 CVODE 默认。

## 后果

- BDF 基线和现有公开 API 不因 Radau5 研究而变化，CVODE 与 Radau5 可以在共同系统事务语义下做
  等误差比较。
- INT-02 只打开后端中立的组合接缝；INT-03 只搬运收窄的纯 C++ 数值核心，两者可在 INT-01 后并行。
- C++ donor 不能原样 vendor。全局回调、一次运行到终点、调用方容差改写、控制台退出和已确认的
  控制流错误都必须按合同处置并留下修改记录。
- URL-only 的 Fortran oracle 不提供历史字节身份或离线可得性保证；未来核验只能声明当时从所列
  官方 URL 取得并运行，不能宣称与先前下载逐字节相同。
- UNIGE 网站把该材料置于统一宽松许可下，但 C++ 归档内部没有独立许可文件，且 Blake Ashby 的
  2003 年改写早于 UNIGE 2004 许可文本。研究搬运不因此阻断；对外或商业发布前仍需按实际分发材料
  复核权利链和许可义务。
