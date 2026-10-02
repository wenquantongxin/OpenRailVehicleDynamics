# libs/integrators

时间积分与系统接受态事务，见 [ADR-0003](../../docs/adr/0003-abstract-advancer-cvode-first.md)。
安装目标为 `ORVD::integrators`。

## 公共调用

`SystemContinuousStateAdvancer` 只有一个构造入口，方法通过强类型配置显式选择：

```cpp
#include <orvd/integrators/system_continuous_state_advancer.h>

orvd::integrators::SystemContinuousStateAdvancer advancer(
    system, plan, accepted_context,
    orvd::integrators::SystemIntegrationConfiguration{
        orvd::integrators::CvodeBdf2Configuration{tolerances}},
    orvd::integrators::NoCallTimeAppliedForces{});
advancer.AdvanceTo(target_time_seconds);
```

`SystemIntegrationConfiguration::method` 包含 `CvodeBdf2Configuration`、
`CvodeBdf5Configuration`、`Radau5Configuration`、`NewmarkConfiguration` 和
`ZhaiConfiguration`。前三者持有物理状态的误差容差；Newmark 持有名义步长和 Newton
量纲尺度；Zhai 只持有等步步长。公共时间统一为 `double` 秒，正步长必须有限。
方法没有隐式默认值；应用层将场景默认配方解析为明确配置。

`method_identifier()` 从真实 runtime 返回 `cvode_bdf2`、`cvode_bdf5`、`radau5`、
`newmark` 或 `zhai`。没有独立的可写方法标签、旧容差构造、私有 Access 工厂或兼容入口。
具体 CVODE 类、BDF 阶次及适配器均在源码私有区域。

共同的 `maximum_internal_steps_per_advance` 限制一次公开推进的成功内步数，默认取公共
`kDefaultMaximumInternalStepsPerAdvance`（1,000,000）。预算耗尽属于工作预算失败。
配置按值拥有；系统、计划及接受 context 借用且必须比推进器寿命长，临时对象借用被禁止。

## Newmark 的尺度

公共配置不暴露内部坐标数组。`nonlinear_solver` 的四族全部要求正且有限；未使用的族通过
校验后不参与展开。

| 族 | 坐标 | 位置、速度、加速度尺度单位 |
| --- | --- | --- |
| `translation` | 自由体平移、移动副 | m、m/s、m/s² |
| `angle` | 转动副、Ball-RPY | rad、rad/s、rad/s² |
| `quaternion` | 四元数存储分量 | 存储分量、存储分量/s、存储分量/s² |
| `force` | 串联弹簧阻尼器内力 | N |

坐标族分别声明 `position_correction`、`velocity_correction`、`acceleration_residual`、
`acceleration_reference`，力族声明 `correction`、`residual`、`reference`。
四元数四项均乘以各块最近成功初始化时的参考范数；初始化包括构造以及成功的显式同步。
`maximum_iterations` 默认 12。
这些量用于非线性求解和数值差分，并非全局积分误差保证，也不从 ODE 容差或采样周期推导。

库内按 `GetJointType()` 与自由体范围建立一次不可变坐标布局，检查位置范围无遗漏、无重叠。
六组展开数组仅由数值核消费。显式同步重新准备参考及尺度，初始化全部成功后共同提交；
失败保留旧参考和尺度。每个成功初始化段内，尺度在非线性求解和普通推进中保持固定；
站位提示刷新也不重新展开。同步按传入状态重新计算浮点范数；即使只更新保持输入，参考与
展开尺度也可能因舍入改变浮点末位。这一约定由库定义，调用方无需另外提供约定标记。

## 数值方法与状态边界

物理状态为 `(q,v,z)`，机械核的私有坐标状态为 `(q,s,z)`，其中
`s=N(q)v=qdot`。`SystemCoordinateProblem` 从 `s` 恢复 `v=Nplus(q)s`，调用一次既有完整
RHS，再计算 `b=N(q)vdot+Ndot(q,v)v=qddot`。力元、轮轨与主动转矩仍走同一装配路径。
几何转换使用独立 context，不改变物理试算 context。

Newmark 固定平均加速度参数，内变量用梯形离散，未知量为完整 `(b1,z1)`；每轮重建完整
残差的前向差分 Jacobian，以缩放稠密 LU 做全 Newton。Zhai 固定基本两步递推，内变量用
AB2，首次或实际步长变化时采用原始启动式。当前实现不自动缩步；本次公共入口重构不改变
积分公式、Newton 策略或启动规则。

四元数按完整存储分量积分，端点投影同时恢复参考范数和切向坐标速度，保持物理角速度。
初始化不修复非法状态。非切向 Newton 试算使用明确的 `Nplus` 延拓，不把任意坐标速度
误认为物理 RHS 的 `N Nplus s`。

## 推进、同步与资源

系统推进层拥有私有 candidate context；每个 runtime 独占其 RHS/坐标桥及实际需要的求解
资源，借用者先析构。只有整个公开推进成功才一次提交接受态。中途失败保留外部接受态，
已发生工作仍可查询，继续推进前须显式 `SynchronizeAfterAcceptedContextChange()`。
初始化、推进及同步中的数值异常统一映射为 `ContinuousStateNumericalFailure`；物理异常
保留原类型和原因。参数及模型绑定检查在后端、Jacobian worker 和首次 RHS 之前完成。

每个成功内步安装物理端点并更新轮轨站位提示；同端点提示更新不会清空 Zhai 历史或补做
机械 RHS。保持输入、物理状态或参数改变仍走显式同步，重建方法历史。RHS 从不提交接受态。

机械适配器通过网格起点及整数步号计算时刻；系数仍使用原步长。真实停止边界使用短步。
采样不增加停靠点：普通物理分量使用端点线性插值，四元数采用同半球归一化线性插值并恢复
参考范数；区间端点逐值复制。插值不调用动力学，不改变历史或统计。

`integration_statistics()` 公开普通及差分求值、迭代、线性求解设置等工作量，成功同步后
重新计数，初始化求值计入新段。启动、投影等细节仅留在私有核与适配器测试。

轮轨求值的并行能力由各后端共用；CVODE 在适用条件下另有私有并行差分 Jacobian 提供器。
线程策略、SUNDIALS 类型和内部数值工作区不进入公共配置。
