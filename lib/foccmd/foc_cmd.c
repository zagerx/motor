#include "stdint.h"
struct foc_cmd {
    uint8_t cmd;  // 命令类型
    uint8_t len;  // 数据长度
    union {
        float  f_buf[4];  // 浮点数据缓冲区
        uint8_t b_buf[16]; // 字节数据缓冲区
    } payload;  // 联合体需要命名
};



