# state_stream

面向通用多体模型的可选接受态采样与UDP输出组件，默认不构建、不安装。

## 构建与消费

构建时设置 `-DORVD_BUILD_STATE_STREAM=ON`。当前发送端支持Linux IPv4单播。
安装消费者通过以下方式请求并链接组件：

```cmake
find_package(OpenRailVehicleDynamics CONFIG REQUIRED COMPONENTS state_stream)
target_link_libraries(your_target PRIVATE ORVD::state_stream)
```

公开接口见 [`state_stream.h`](include/orvd/state_stream/state_stream.h)。
刚体状态值类型、采样器与标量定义来自公共库 `ORVD::scene_observation`
（`orvd/scene_observation/body_state.h`、`scalar_definition.h`）；本组件只承担编码、分片与 UDP 发送。
调用方负责采样时机、输出内容与目标地址；组件不介入控制或积分时序。

## 收发边界

发送使用非阻塞 socket，不开启后台线程、不等待接收端确认。默认不等待本地
发送容量；网络发送错误仅计数，无效输入与初始化错误明确报告。UDP 输出不保证
可靠归档。

构造器第三参数 `UdpSendOptions` 可设置：

- `socket_send_buffer_bytes`：正整数 `SO_SNDBUF` 请求；不指定时使用操作系统默认值。
  `StreamStatistics` 同时记录请求值和 `getsockopt` 返回的有效值。Linux 会调整并
  通常加倍该值，不能将请求值当作有效容量。
- `retry_budget_seconds`：有限非负数，默认 `0`。大于零时，仅在 `EAGAIN` /
  `EWOULDBLOCK` 后用 `poll` 等待本地可写，再重试同一个尚未成功发送的分包。
  每次 `PublishDescription` 或 `PublishState` 的全部分包与目的地共用一次预算，
  编码及分片完成后开始计时；成功分包不重复发送。等待发生在调用线程中，
  `poll` 的毫秒精度及系统调度可能使实际用时略超预算；它不是硬实时期限。

各目的地分别统计容量不足次数、额外发送尝试次数、最终重试成功的报文数及等待
时间。只有最终未发出的报文计入 `failed_datagrams`；其他网络错误不重试。
这只处理本地发送容量不足，不检测或补发已经发出后在网络中丢失的报文。

独立接收示例 [`receive_state_stream.py`](receive_state_stream.py) 仅依赖Python标准库，
可复制到接收设备运行；使用 `--help` 查看参数。
接收示例负责消息重组与接收统计，不承担显示或控制功能。
