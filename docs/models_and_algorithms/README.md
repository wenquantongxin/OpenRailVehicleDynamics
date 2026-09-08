[English](README.en.md)

# 理论模型与计算算法

本目录解释 ORVD 所采用的轨道车辆理论、数学模型与计算算法，并把公式中的量对应到实际代码。它不是软件使用指南，也不承担接口、配置、测试、异常、实验或性能验收说明。

## 与技术文档的边界

| 目录 | 职责 |
|---|---|
| `models_and_algorithms/` | 物理模型、坐标与符号、数学推导、离散算法、理论近似及其代码实现 |
| `design/` | 软件架构、模块关系、接口契约与运行时所有权 |
| `engineering/` | API、配置、输入格式、失败行为、测试与工程纪律 |
| `performance/` | 缓存、工作区、并行调度、硬件利用、计时与优化验收 |
| `planning/` | 实施顺序、迁移记录与项目裁决 |

计算加速只有在改变数学计算本身时才进入本目录，例如代数等价变换、离散算法复杂度降低或带误差预算的近似。缓存键、内存复用、线程调度、零分配和机器计时属于技术文档。

理论文档可以说明模型的数学假设与适用条件，但不以测试覆盖、实验工况或资格结果定义模型。尚未落入代码的理论统一标为 **仅理论**；已经落入代码的内容必须准确说明本库实际采用的公式和离散方式。

## 已发布主题

| 分类 | 内容 |
|---|---|
| `track_geometry/` | 线路平纵断面、超高、轨道坐标系、站位投影和竖向剖面 |
| `track_irregularity_spectra/` | 轨道不平顺 PSD、有限空间频带、随机实现和多方向相关性 |
| `wheel_rail_contact/` | 型面与插值、位姿归约、接触几何、法向力、蠕滑、Kalker 系数、FASTSIM 与扳手组装 |
| `force_elements/` | 连接运动学与扳手、三向平动弹簧—阻尼、侧滚力偶、串联 Maxwell、饱和折线阻尼与半角中点 RPY 衬套 |
| `numerical_methods/` | 时间积分、隐式非线性迭代、误差控制与稳定性 |

## 理论文档形式

- 中文主文档与在扩展名前加 `.en` 的英文同名文件相邻存放，内容等价并在首行互链。
- 以 GitHub Markdown 为发布格式；行内公式使用 `$...$`，行间公式使用独立的 `$$` 块。
- 篇章按内容需要组织，但应能辨认出：范围与实现状态、记号、理论模型、计算算法、代码实现与理论假设。
- “代码实现”解释公式如何落实到核心数据结构、函数和计算顺序，不展开 API 清单、配置字段、异常消息或测试文件。
- 常数只有在参与模型或算法定义时才进入正文；必须说明其数学角色，不能把源码字面量表当作配置参考。
- 必须说明理论来源时，在相关正文就地简短链接原始论文、专著或权威综述；暂不维护集中或逐篇的文献表。仓库源码只能证明 ORVD 采用了什么，不能代替理论出处。
- 测试证据、实验结果、资格记录、性能实测、提交历史和迁移过程不进入本目录。

全部文档共用[坐标与记号约定](CONVENTIONS_AND_NOTATION.md)。

## 文档索引

### 共享基座

- [坐标与记号约定](CONVENTIONS_AND_NOTATION.md)：轨道惯性系与轨型系、正号、站位与弧长、位姿、状态块、扳手和核心中英术语。

### 线路几何

- [线路几何与轨道坐标系](track_geometry/TRACK_GEOMETRY_AND_FRAMES.md)：标量剖面、Bloss/Hermite 曲率过渡、五次接缝、平面积分、轨型系、切线延长与局部站位投影。
- [轨道竖向剖面建模及其三维耦合](track_geometry/TRACK_VERTICAL_PROFILE_MODELLING.md)：恒坡段、PL2、CIR、竖向接缝及其与平面线形和超高的三维耦合。

### 轨道不平顺

- [轨道不平顺谱及其空间随机实现](track_irregularity_spectra/TRACK_IRREGULARITY_SPECTRA.md)：空间频率、FRA/AAR 谱、有限频带、随机实现和多方向关系。

### 轮轨接触

- [型面点列与插值](wheel_rail_contact/PROFILES_AND_INTERPOLANTS.md)：型面侧解析、自然三次样条、保形三次插值、弧长、等弧长重扫与轨距基准。
- [轮轨位姿归约与不平顺输入](wheel_rail_contact/WHEEL_RAIL_POSE_REDUCTION.md)：三维轮对态到四个位姿标量、局部坐标系、不平顺输入与 X-Z-Y 姿态解析。
- [接触几何](wheel_rail_contact/CONTACT_GEOMETRY.md)：可见轮廓、包络、接触岛、逐岛求积、公法线角、接触坐标系角与三维纵向弦长。
- [法向接触力](wheel_rail_contact/NORMAL_CONTACT_FORCE.md)：等面积圆弧段、等效穿透、Hertz 解、椭圆积分、阻尼与纵向基线。
- [蠕滑率与接触坐标系](wheel_rail_contact/CREEPAGE_AND_CONTACT_FRAME.md)：接触坐标系、参考速度、三个蠕滑率、法向接近速度与滚动半径约定。
- [Kalker 线性蠕滑系数](wheel_rail_contact/KALKER_COEFFICIENTS.md)：有限系数表、泊松轴插值、半轴比插值与细长椭圆渐近式。
- [切向接触力：FASTSIM 条带推进](wheel_rail_contact/TANGENTIAL_CONTACT_FASTSIM.md)：条带推进、应力积累、压力分布、黏滑边界、自旋加密与下降摩擦律。
- [单轮接触模型组装与成对扳手](wheel_rail_contact/CONTACT_MODEL_ASSEMBLY_AND_WRENCH.md)：接触链的物理组装、材料参考点、轮侧作用点、坐标变换与成对扳手。

### 力元

- [力元连接运动学与空间扳手](force_elements/FORCE_ELEMENT_KINEMATICS_AND_WRENCHES.md)：两端相对位置、相对姿态、含运输项的相对速度与相对角速度，三种成对扳手施加方式、换点规则、功率恒等式与组织原则。
- [三向平动弹簧—阻尼力元](force_elements/TRANSLATIONAL_SPRING_DAMPER.md)：参考端三轴并联弹簧—阻尼与名义力、端点力对加支承矩、储能与耗散、对角本构在空间表达中的姿态依赖。
- [侧滚弹簧—阻尼力偶](force_elements/ROLL_SPRING_DAMPER_COUPLE.md)：矩阵元滚转度量、相对角速度分量、纯力偶对、纯滚转下的储能与耦合转动下的性质。
- [串联弹簧—黏性阻尼力元](force_elements/SERIES_SPRING_VISCOUS_DAMPER.md)：Maxwell 内力方程、松弛时间、解析响应、力状态与系统连续状态的衔接。
- [奇对称饱和分段线性阻尼力元](force_elements/SATURATED_PIECEWISE_LINEAR_DAMPER.md)：非负半轴折线力曲线、末节点外恒值延拓、奇延拓、定义域三条要求的后果与耗散势。
- [半角中点 RPY 衬套](force_elements/HALF_ANGLE_MIDPOINT_RPY_BUSHING.md)：半角中间系、中点材料相对速度、space-XYZ 角提取与速率映射、功率共轭的物理力矩与中点扳手对。

### 数值方法

- [BDF、Radau5、Newmark 与 Zhai 时间积分方法](numerical_methods/TIME_INTEGRATION_METHODS.md)：离散公式、单步与多步推进算法、误差和稳定性，以及不同方法对 ORVD 状态结构的适用条件。
