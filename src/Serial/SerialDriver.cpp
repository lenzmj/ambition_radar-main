#include "Serial/SerialDriver.h"
#include "Serial/crc.hpp"
#include <iostream>
#include <cerrno>


SerialDriver::SerialDriver(const char* port) {
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "[Serial] 无法打开 " << port << "（errno=" << errno << "）。"
                  << "手眼采集仍可运行，但 yaw/pitch/roll 会保持 0。\n";
        return;
    }

    struct termios options {};
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "[Serial] tcgetattr 失败\n";
        close(fd);
        fd = -1;
        return;
    }

    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);

    // 确保非阻塞（部分驱动在 tcsetattr 后行为不一致）
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

SerialDriver::~SerialDriver() {
    if (fd != -1) close(fd);
}

void SerialDriver::send_packet(const SendPacket& pkt) {
    if (fd != -1) {
        SendPacket tx_pkt = pkt;
        tx_pkt.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&tx_pkt),
                                        sizeof(SendPacket) - 2);
        write(fd, &tx_pkt, sizeof(SendPacket));
    }
}

bool SerialDriver::receive_packet(ReceivePacket& in_pkt) {
    if (fd < 0) return false;

    uint8_t buffer[sizeof(ReceivePacket)];
    uint8_t byte;

    // 非阻塞扫到帧头；扫不到立即返回
    while (true) {
        ssize_t n = read(fd, &byte, 1);
        if (n <= 0) return false;
        if (byte == 0x5A) break;
    }

    buffer[0] = 0x5A;
    int total_read = 1;
    // 剩余字节：短超时等待，避免半包时永久卡住
    while (total_read < static_cast<int>(sizeof(ReceivePacket))) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 5000;  // 5 ms
        const int sel = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) return false;

        const ssize_t n =
            read(fd, buffer + total_read, sizeof(ReceivePacket) - static_cast<size_t>(total_read));
        if (n <= 0) return false;
        total_read += static_cast<int>(n);
    }

    if (!tools::check_crc16(buffer, sizeof(ReceivePacket))) {
        return false;
    }

    memcpy(&in_pkt, buffer, sizeof(ReceivePacket));
    return true;
}

void SerialDriver::flush_input() {
    if (fd != -1) {
        tcflush(fd, TCIFLUSH);
    }
}
