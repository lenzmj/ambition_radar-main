# Serial（实现功能）

## SerialDriver.cpp

- 打开串口为 raw 模式，波特率 **921600**。
- `send_packet`：拷贝报文、填 CRC16 后写出。
- `receive_packet`：找帧头 `0x5A`、读满包、CRC16 校验；失败返回 `false`。
- `flush_input`：丢弃输入缓冲中未读数据。

## crc.cpp

实现 CRC8/CRC16 查表计算与校验，供串口协议帧使用。
