# 对 vendored Drake 源码的修改

本文件记录 `drake/` 下的源码相对上游的**每一处**改动,以及替代上游实现的 ORVD
第一方源码。上游修订与逐文件处置见
[`SOURCE_DISPOSITION.txt`](SOURCE_DISPOSITION.txt),此处不复述。

不记行号:行号随任何一次上游同步而漂移,漂移后的记录比没有记录更坏。每条记的是
**改了哪个符号、改成什么、为什么**,这些在源码里可以直接搜到。

未列入本文件的差异即为缺陷。G18 的分发义务以本文件为准。

## G13 — vendor 刚性 topology 源码

### `multibody/topology/link_joint_graph.h`

**删除** `LinkJointGraph::GenerateGraphvizString()` 与 `LinkJointGraph::MakeGraphvizFiles()`
的完整声明及其说明块,连同随之失去对象的调试 API 注释,以及只为 `MakeGraphvizFiles`
存在的 `#include <filesystem>`。

**原因**:这两个函数的唯一实现在 `multibody/topology/link_joint_graph_debug.cc`,该文件
已裁为 `discard`——它把图写成 `.dot` 并调用 `system()` 去跑 Graphviz 的 `dot`,与刚体
动力学求解无关。留下声明而不留实现,会造出一个"能编译、一调用就缺符号"的半支持 API:
调用方看得见它、写得出调用、直到链接才发现它不存在。删实现就同步删声明。

**未改动**:`SpanningForest::GenerateGraphvizString()` 保留。它是同名但不同类的另一个
函数,其实现在**准入**的 `spanning_forest_debug.cc` 内,该文件因 `spanning_forest.cc`
经 `DRAKE_ASSERT_VOID` 调用 `SanityCheckForest()` 而必须准入。

### `multibody/topology/spanning_forest.h`

**删除**指向已删除的 `LinkJointGraph::GenerateGraphvizString()` 的 `@see` 引用。

**原因**:上一处删除使该引用悬空。文档里指向不存在的东西,读者会去找,找不到,然后
怀疑是自己漏了。

## G15 — vendor 剩余刚性 tree 源码

### `orvd_implementations/double_rotation_and_rigid_transform_composition.cc`(新增,**非上游文件**)

**新增** ORVD 自研的 `ComposeRR` / `ComposeRinvR` / `ComposeXX` / `ComposeXinvX`,满足上游
`math/fast_pose_composition_functions.h` 的声明。上游同名 `.cc` 已裁 `first_party`,**不复制**。

**原因**:上游实现无条件 `#include <hwy/foreach_target.h>` 与 `<hwy/highway.h>`,经 Google
Highway 分派。那是额外的第三方依赖,产品不准入(G12)。抽取上游的 portable 分支再称作
第一方实现,等于复制上游代码却改署名,已明令禁止。因此按数学定义重写:

```
ComposeRR:    R_AC = R_AB R_BC
ComposeRinvR: R_AC = R_BAᵀ R_BC
ComposeXX:    R_AC = R_AB R_BC,  p_AC = p_AB + R_AB p_BC
ComposeXinvX: R_AC = R_BAᵀ R_BC, p_AC = R_BAᵀ (p_BC - p_BA)
```

`ComposeXX` 刻意不做 4×4 齐次矩阵乘:刚体变换的末行在结构上是常量,乘出来是花算力
重算一个已知的行。取转置而非通用求逆,是因为声明的前置条件就是输入为合法旋转,正交矩阵
的转置即其逆。

声明允许输出与任一或两个输入重叠,因此每个结果先算进定长局部量,读完全部输入后**一次**
写回;不依赖 `noalias()`——那是对调用方明确允许的行为作出的保证。不重复校验、不重新
正交化、不做空指针检查:内部调用方在进入热路径前保证旋转合法且输出非空。

### `math/fast_pose_composition_functions.h`

**删除**「以平台 SIMD 指令实现以求速度」「as quickly as possible」等描述,改为说明本实现
由 ORVD 按数学定义提供、不含 SIMD 分派、且不作性能承诺。

**原因**:上游那些描述针对的是被我们拒绝的 Highway 实现。留着它们,读者会以为这里有向量化
或有过性能测量,两者都没有。文件名沿用上游。

### `math/rotation_matrix.h` 与 `math/rigid_transform.h`

**删除或改写**只适用于上游 Highway 实现的 SIMD 与速度描述。保留可由公式直接说明的事实:
轴向旋转专门路径运算更少,逆组合使用旋转转置与刚体变换结构。

**原因**:当前实现没有 SIMD 分派,也没有在产品构建中完成性能测量;不把未经测量的速度承诺
留在公共 API 文档中。

### `multibody/tree/element_collection.cc`

**删除** `#include "drake/multibody/tree/deformable_body.h"` 与三组
`ElementCollection<*, DeformableBody, DeformableBodyIndex>` 显式实例化。

**原因**:`deformable_body.h` 是禁入类别(FEM + geometry + MultibodyPlant 三条边同在一个
头文件里)。这是准入集合里唯一一条禁入边,删除它使产品边界干净——`forbidden_prefix` 与
逐文件禁入的裁决在此兑现,而不是停留在清单上。

### `multibody/tree/multibody_tree_indexes.h`

**删除** `MultibodyConstraintId`、`DeformableBodyId`、`DeformableBodyIndex` 三个别名及其
说明,连同随之无用的 `#include "drake/common/identifier.h"`。

**原因**:三者都在刚体边界之外——前者属 MultibodyPlant 的约束,后两者属 deformable。它们是
`common/identifier.h` 进入闭包的唯一原因;删掉后该文件改判 `discard` 并从落位树移除。

### `multibody/topology/link_joint_graph_loop_constraint.h`

**删除**引用已删除的 `MultibodyConstraintId` 与 MultibodyPlant 的 TODO 注释。

**原因**:上一处删除使它悬空,且它指向的是产品不准入的 plant 概念。

### `multibody/tree/element_collection.h`、`multibody/tree/multibody_element.h`
与 `multibody/tree/joint_actuator.h`

**删除**文档中对 `DeformableBody` 的引用:前者把它列在 `ElementCollection` 的实例化类型
表里,后者两处以 `DeformableBody::DoDeclareDiscreteStates()` /
`DoDeclareCacheEntries()` 作为用法示例。另从 `MultibodyElement` 删除
`DeformableModel` 的前置声明与 friend,并删除只有禁入 plant 翻译单元提供实现的
`GetParentPlant()` 声明；`JointActuator` 的相关说明改为它实际使用的 parent tree。

**原因**:deformable 与 plant 都不在准入边界内,这些引用与 API 因此悬空。第一条还是
**事实错误**——对应的显式实例化已在 `element_collection.cc` 删除,类型表若不同步就会
说谎。保留 `GetParentPlant()` 会暴露一个可调用声明,但产品中没有也不会提供它的定义。

### `multibody/tree/unit_inertia.cc`

**删除**未被使用的 `#include "drake/common/text_logging.h"`。

**原因**:全文没有一处 `log()` 或 `DRAKE_LOGGER_*` 调用。留着它会把 spdlog 拖进闭包,
而拖进来的理由是一行谁都没在用的 include。

### `multibody/tree/frame.cc`

**删除**未被使用的 `#include "drake/common/identifier.h"`。

**原因**:同上,该文件不构造任何 `Identifier`。

## 未作的修改

- **没有改写任何 `#include "drake/..."` 前缀。** 源码按上游相对路径落位在 `drake/` 下,
  include 行一字未动。改前缀是对每个文件的机械编辑,且每次上游同步都要重做一遍。
- **没有添加逐文件版权头。** Drake 的 tree/topology 候选文件本就没有;人工发明是
  误述来源,不是保全来源。支撑源码原有的 Stanford/Apache-2.0 声明原样保留。
- **没有改写 vendored Drake 的刚体算法主体。** 边界/API/include/说明的裁剪均逐项列在
  上文；四个位姿组合函数是明确分列的 ORVD 第一方实现。
