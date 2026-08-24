#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h> // 必须包含
#include <cstring>
#include "Serial/Protocol.h"

class SerialDriver {
public:
    SerialDriver(const char* port);
    ~SerialDriver();
    void send_packet(const SendPacket& pkt); 
    bool receive_packet(ReceivePacket& in_pkt);
    void flush_input(); //清空输入缓冲区

private:
    int fd;
};