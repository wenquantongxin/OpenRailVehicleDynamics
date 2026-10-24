# state_stream

可选的 Linux UDP 发送组件，默认不构建、不安装。

## 构建与消费

构建时设置 `-DORVD_BUILD_STATE_STREAM=ON`。发送端支持 Linux IPv4 单播。
安装消费者通过以下方式请求并链接组件：

```cmake
find_package(OpenRailVehicleDynamics CONFIG REQUIRED COMPONENTS state_stream)
target_link_libraries(your_target PRIVATE ORVD::state_stream)
```

公开接口见 [`state_stream.h`](include/orvd/state_stream/state_stream.h)。
刚体状态值类型、采样器与标量定义来自公共库 `ORVD::scene_observation`
（`orvd/scene_observation/body_state.h`、`scalar_definition.h`）；本组件调用默认产品库
`ORVD::scene_stream` 完成编码与分包，再发送 UDP 报文。
协议见 [OSTS v1 规格](../../libs/scene_stream/README.md)。
描述由 `scene_stream::EncodeDescription` 生成，再交给 `PublishDescription`。
调用方指定采样时机、输出内容与目标地址。

## 收发边界

发送在调用线程中使用非阻塞 socket。UDP 尽力发送，网络错误计入统计；
无效输入与初始化错误直接报告。

构造器第三参数 `UdpSendOptions` 可设置：

- `socket_send_buffer_bytes`：正整数 `SO_SNDBUF` 请求；不指定时使用操作系统默认值。
  `StreamStatistics` 记录请求值和 `getsockopt` 返回的有效容量。
- `retry_budget_seconds`：有限非负数，默认 `0`。大于零时，仅在 `EAGAIN` /
  `EWOULDBLOCK` 后用 `poll` 等待本地可写，再重试同一个尚未成功发送的分包。
  每次 `PublishDescription` 或 `PublishState` 的全部分包与目的地共用一次预算，
  编码及分片完成后开始计时；成功分包不重复发送。等待发生在调用线程中，
  `poll` 的毫秒精度和系统调度可能使实际等待时间超过预算。

各目的地分别统计容量不足次数、额外发送尝试次数、最终重试成功的报文数及等待
时间。只有最终未发出的报文计入 `failed_datagrams`；其他网络错误不重试。
网络丢包不检测、不补发。

独立接收示例 [`receive_state_stream.py`](../../libs/scene_stream/receive_state_stream.py) 仅依赖 Python 标准库，
随 `scene_stream` 默认安装；使用 `--help` 查看参数。

## 记录回放发送

开启本组件时同时构建不安装的 `orvd_replay_scene_record`：

```bash
orvd_replay_scene_record path/to/record \
  --udp-target 127.0.0.1:10099 --resource-bundle route
```

工具按记录时间差以 1× 发送，`--udp-target` 可重复指定。资源名称由接收服务映射到
记录或资源包目录；省略 `--resource-bundle` 时资源引用为 `null`。描述首发后每秒重发，
各类消息使用独立序号。记录的专用轮角列转换为连续弧度标量并声明绑定，
原有刚体状态与标量保持原值。
