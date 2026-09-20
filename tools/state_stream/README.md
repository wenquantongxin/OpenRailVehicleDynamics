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
调用方负责采样时机、输出内容与目标地址；组件不介入控制或积分时序。

## 收发边界

发送非阻塞，不开启后台线程、不等待接收端、不重传。网络发送错误仅计数；
无效输入与初始化错误明确报告。UDP输出不保证可靠归档。

独立接收示例 [`receive_state_stream.py`](receive_state_stream.py) 仅依赖Python标准库，
可复制到接收设备运行；使用 `--help` 查看参数。
接收示例负责消息重组与接收统计，不承担显示或控制功能。
