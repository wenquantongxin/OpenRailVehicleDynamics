# 对 vendored Drake 源码的修改

本文件记录 `drake/` 下的源码相对上游的**每一处**改动。上游修订与逐文件处置见
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

## 未作的修改

- **没有改写任何 `#include "drake/..."` 前缀。** 源码按上游相对路径落位在 `drake/` 下,
  include 行一字未动。改前缀是对每个文件的机械编辑,且每次上游同步都要重做一遍。
- **没有添加逐文件版权头。** Drake 的 tree/topology 候选文件本就没有;人工发明是
  误述来源,不是保全来源。支撑源码原有的 Stanford/Apache-2.0 声明原样保留。
- **没有修改任何算法语义。** 本轮的全部改动就是上面两处文档与声明清理。
