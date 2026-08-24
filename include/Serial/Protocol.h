#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>
#include <cstdint>

// 强制 1 字节对齐
#pragma pack(push, 1) 

struct SendPacket {
    uint8_t header = 0xA5; // 帧头
    uint8_t mode = 0;
    float pitch = 0;
    float yaw = 0;
    float distance = 0 ;
    uint16_t crc16; 
};

struct ReceivePacket {
    uint8_t header = 0x5A; // 接收帧头
    float current_yaw;
    float current_pitch;
    float current_roll;
    uint8_t reserved = 0; // 补齐 16 字节
    uint16_t crc16; 
};


#pragma pack(pop) 

#endif // PROTOCOL_H
