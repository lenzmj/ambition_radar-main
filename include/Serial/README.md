# Serial（头文件定义）

串口通信与协议相关类型、接口声明。

## Protocol.h

- `SendPacket`：上位机 → 下位机。帧头 `0xA5`，含 `mode`、`pitch`、`yaw`、`distance`、`crc16`；1 字节对齐。
- `ReceivePacket`：下位机 → 上位机。帧头 `0x5A`，含当前 `yaw`/`pitch`/`roll`、`reserved`、`crc16`。

## SerialDriver.h

- `SerialDriver`：串口读写封装。
  - 构造：打开端口；析构：关闭。
  - `send_packet` / `receive_packet`：按协议收发。
  - `flush_input`：清空输入缓冲。

## crc.hpp

`tools` 命名空间下的 CRC 接口：

- `get_crc8` / `check_crc8`
- `get_crc16` / `check_crc16`（串口收发包主要使用 CRC16）
