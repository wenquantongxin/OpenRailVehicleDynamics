# scene_stream

`ORVD::scene_stream` 提供可移植的 OSTS v1 编解码、报头解析和分包，默认构建并安装。
公共接口见 [`scene_stream.h`](include/orvd/scene_stream/scene_stream.h)。

```cmake
find_package(OpenRailVehicleDynamics CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE ORVD::scene_stream)
```

`EncodeState` / `DecodeState` 处理状态正文；`Packetize` 生成报文；`ParseHeader`
校验单个报文。`EncodeDescription` / `DecodeDescription` 复用
`scene_observation::SceneDescription` 的描述编解码。

## OSTS v1 报文

所有整数均为无符号小端序，实数为 IEEE 754 binary64 小端序，无结构体填充。
每个 UDP 报文最多 1200 字节，前 40 字节为报头，后面最多 1160 字节为正文分片。

| 字节偏移 | 字段 | 类型与含义 |
|---:|---|---|
| 0 | magic | 4 字节 ASCII `OSTS` |
| 4 | version | uint16，固定为 `1` |
| 6 | kind | uint16，`1` 描述、`2` 状态 |
| 8 | run_id | uint64，非零运行身份 |
| 16 | sequence | uint64，生产者提供的消息序号 |
| 24 | simulation_time_seconds | binary64，有限仿真秒数 |
| 32 | fragment_index | uint16，从零起的分片序号 |
| 34 | fragment_count | uint16，至少为 1 |
| 36 | payload_bytes | uint32，完整正文的字节数 |

分片数为 `max(1, ceil(payload_bytes / 1160))`，不超过 65535。
除最后一片外，每片正文均为 1160 字节；空正文仍有一个仅含报头的报文。
同一消息的各片拥有相同的种类、运行身份、序号、时间、分片总数和正文长度。
重组键为 `(run_id, kind, sequence)`。

`sequence` 用于同一运行、同一种类的消息排序，仿真时间取报头中的 `simulation_time_seconds`。

`run_id` 与 `sequence` 必须保留完整 64 位。JavaScript 使用 `BigInt`；进入 JSON
时使用十进制字符串，不转换为 `Number`。

## 状态正文（kind 2）

按以下顺序连续存储：

1. uint32 刚体数。
2. 每个刚体 13 个 binary64：位置 `x,y,z`，姿态 `w,x,y,z`，线速度 `x,y,z`，角速度 `x,y,z`。
3. uint32 标量数。
4. 所有标量值，连续 binary64。
5. 所有标量状态，连续 uint8：`0` 未就绪、`1` 有效、`2` 占位。

正文长度为 `8 + 104 × 刚体数 + 9 × 标量数`。全部实数必须有限；非有效标量的
数值必须为零。编码保留负零。刚体和标量的槽位顺序来自公共描述。

位置和线速度对应刚体坐标系原点，所有向量均表达于描述的世界坐标系；四元数
将刚体坐标映射到世界坐标。车轮姿态包含自转；可选连续轮角标量用于选择姿态插值的旋转分支。

## 描述正文（kind 1）

正文为 UTF-8 JSON：

```json
{
  "schema": "orvd.scene_description",
  "description": {},
  "resource_bundle": null
}
```

`description` 填入公共场景描述对象，包含世界系、单位、四元数约定、刚体、车轮
放置、标量定义、状态码与阶段码、可选名义采样周期。结构与场景记录中的
`description` 相同。`resource_bundle` 为 `null` 或非空资源名称，由消费者映射到
明确配置的资源目录。线路表与视觉定义保存在资源包中，不内联到描述消息。

同一运行身份的描述保持不变；描述变化使用新的运行身份。生产者可以周期重发
描述，便于中途加入。描述尚未到达时，状态正文仍可解码，但不绑定名称。

## WebSocket 完整消息

一个二进制 WebSocket 消息保存原 UDP 首片的 40 字节报头，随后是完整重组正文。
首片序号保持为零，分片总数和正文长度保持原值；消息总长度为 `40 + payload_bytes`。
接收端按完整消息长度校验，运行身份、序号和仿真时间沿用原报头。

网页接收服务按首次完整有效消息的到达顺序登记运行；新运行的描述与匹配状态就绪后，
接管显示并退役较早的缓存运行。

## 接收示例

标准库 Python 示例 [`receive_state_stream.py`](receive_state_stream.py)，与本说明
一起安装到 `share/OpenRailVehicleDynamics/scene_stream`：

```bash
python3 receive_state_stream.py --bind 127.0.0.1 --port 10099 \
  --output-directory received --capture-states
```

仅检查字段与节奏时可输出报告：

```bash
python3 receive_state_stream.py --report stream-report.json --duration-seconds 30
```

`--report` 拒绝覆盖已有文件，可替代输出目录；`--capture-states` 仍需输出目录。
`--duration-seconds` 从监听成功起计时，`--max-states` 限制完整解码的状态数；两者与
空闲退出条件先到者结束，中断也写报告。报告按运行列出字段、轮角绑定、有效性计数、
序号缺口、仿真时间与到达间隔，以及保留和淘汰的运行数。序号缺口不等同于丢包率。
描述到达前的状态单独计数，不回填具名字段的有效性统计。

示例的内存限额为每条消息 1 MiB、八条待组消息、十六个运行记录。
长度检查先于分片保存；正文成功解码后才更新接受
序号，随即清理同类旧分片。运行淘汰同时清理其描述、序号和分片。

UDP 发送组件为可选的 `ORVD::state_stream`，发送等待预算与统计由该组件负责。
