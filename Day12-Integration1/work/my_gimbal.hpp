/**
 * Day12-Integration1/work/my_gimbal.hpp —— 串口通信封装（基于 T-26-5）
 *
 * #### Task 12-4b: 封装串口通信为云台控制接口 #####################
 *
 * 你在 Training-26-5 中已经编写了串口通信程序（8字节包、union浮点数拆分、
 * 位运算字节）。现在将其封装为 my_gimbal.hpp，提供简洁的云台控制接口。
 *
 * 目标接口：
 *   send(yaw, pitch, fire)  —— 向电控发送云台角度和开火指令
 *   receive() → GimbalState  —— 接收电控发来的云台状态（角度、弹速等）
 *
 * 8字节数据包格式（参考 26_SP io/gimbal/，★ 简化教学版本）:
 *   Byte 0: 包头 (如 0xA5)
 *   Byte 1: 无符号整数 (0-255)，如模式标志
 *   Byte 2: 位运算字节（每位代表一个布尔标志，如开火、陀螺等）
 *   Byte 3-6: 浮点数（如 yaw 角，IEEE 754 单精度）
 *   Byte 7: 包尾 (如 0x5A)
 *
 * ⚠ SPSREMU 兼容性警告:
 *   以上为简化教学格式（8字节，仅含yaw），与 26_SP / SPSREMU_V10.py
 *   的真实协议不同。26_SP 实际格式（以普通模式为例）:
 *     head(0xCD) + pitch(float4B) + yaw(float4B) + mode(1B)
 *     + bullet_speed(1B) + tail(0xDC) = 11字节
 *   使用 SPSREMU_V10.py 联调时，需将封包格式适配为 26_SP 协议。
 *   详见 Day13-Integration2/work/README.md 的协议兼容性说明。
 *
 * 参考：
 *   - 26_SP io/gimbal/gimbal.cpp / gimbal.hpp
 *   - 26_SP io/serial/serial.cpp (底层串口读写)
 *   - Training-26-5 你自己编写的串口程序
 *   - SPSREMU_V10.py (串口模拟器，用于无硬件调试)
 */

#pragma once

#include <string>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>

namespace my_auto_aim {

// ================================================================
// 云台状态（从电控接收）
// ================================================================
struct GimbalState {
    double current_yaw{0.0};      // 当前云台 yaw 角 (rad)
    double current_pitch{0.0};    // 当前云台 pitch 角 (rad)
    double bullet_speed{0.0};     // 当前弹速 (m/s)
    bool auto_mode{false};        // 是否处于自动模式
};

// ================================================================
// #### Task 12-4b: 实现 MyGimbal 类 #############################
//
// 串口通信的核心步骤：
//   1. 打开串口: open(device, O_RDWR | O_NOCTTY)
//   2. 配置串口: tcgetattr + 设置波特率/数据位/停止位/校验位
//   3. 读取数据: read(fd, buffer, size)
//   4. 写入数据: write(fd, buffer, size)
//   5. 关闭串口: close(fd)
//
// 实现策略：
//   - 如果已有 T-26-5 的串口代码，直接复用
//   - 如果硬件不在手边，使用 MockGimbal（终端打印）进行逻辑验证
//   - Day13 使用 SPSREMU_V10.py 进行模拟联调
// ================================================================
class MyGimbal {
public:
    /**
     * 构造函数：打开串口设备
     * @param device  串口设备路径，如 "/dev/ttyUSB0"
     * @param baud_rate  波特率，默认 115200
     */
    MyGimbal(const std::string& device, int baud_rate = 115200)
        : device_(device), baud_rate_(baud_rate), is_mock_(true)
    {
        // TODO: 尝试打开真实串口，失败则回退到 Mock 模式
        // fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        // if (fd_ < 0) { is_mock_ = true; return; }
        // configure_serial(fd_, baud_rate);
        // is_mock_ = false;
        
        std::cout << "[MyGimbal] Mock 模式已启用" << std::endl;
        std::cout << "  串口设备: " << device_ << std::endl;
        std::cout << "  波特率:   " << baud_rate_ << std::endl;
        std::cout << "  （接入真实串口后自动切换为真实模式）" << std::endl;
    }

    ~MyGimbal() {
        if (!is_mock_ && fd_ >= 0) {
            close(fd_);
        }
    }

    // ============================================================
    // ★ 发送接口: send(yaw, pitch, fire) ★
    //
    // 将角度和开火指令打包为 8 字节数据包发送。
    //
    // #### Task 12-4b-1: 实现 8 字节封包 ########################
    //
    // 封包步骤:
    //   1. 将 yaw (rad) 转为单精度浮点数
    //   2. 使用 union 将浮点数拆分为 4 个字节
    //      union { float f; uint8_t bytes[4]; } converter;
    //   3. 按协议组装 8 字节包
    //
    // 提示: 注意字节序（大端/小端），RM 通常使用小端序。
    // ============================================================
    void send(double yaw, double pitch, bool fire) {
        // === 你的代码开始 ===
        
        if (is_mock_) {
            // Mock 模式：打印到终端
            std::cout << "  [GIMBAL→电控] yaw=" << yaw * 180.0 / M_PI << "°"
                      << " pitch=" << pitch * 180.0 / M_PI << "°"
                      << " fire=" << (fire ? "YES" : "NO") << std::endl;
            return;
        }
        
        // 真实串口模式：
        // uint8_t packet[8];
        // packet[0] = 0xA5;  // 包头
        // packet[7] = 0x5A;  // 包尾
        // 
        // // Byte 1: 模式标志
        // packet[1] = fire ? 0x01 : 0x00;
        // 
        // // Byte 2: 位运算标志
        // packet[2] = 0x00;
        // if (fire) packet[2] |= 0x01;
        // 
        // // Byte 3-6: yaw 浮点数（小端序）
        // union { float f; uint8_t b[4]; } yaw_u;
        // yaw_u.f = static_cast<float>(yaw);
        // packet[3] = yaw_u.b[0];
        // packet[4] = yaw_u.b[1];
        // packet[5] = yaw_u.b[2];
        // packet[6] = yaw_u.b[3];
        // 
        // write(fd_, packet, 8);
        
        // === 你的代码结束 ===
    }

    // ============================================================
    // 接收接口：读取电控发来的云台状态
    // ============================================================
    GimbalState receive() {
        GimbalState state;
        
        // TODO: 从串口读取并解析 8 字节包
        // === 你的代码开始 ===
        
        if (is_mock_) {
            // Mock 模式：返回模拟数据
            state.current_yaw = 0.0;
            state.current_pitch = 0.0;
            state.bullet_speed = 28.0;
            state.auto_mode = true;
            return state;
        }
        
        // 真实串口模式：
        // uint8_t buffer[8];
        // int n = read(fd_, buffer, 8);
        // if (n == 8 && buffer[0] == 0xA5 && buffer[7] == 0x5A) {
        //     // 解析 buffer[1-6]
        // }
        
        return state;
        
        // === 你的代码结束 ===
    }

    /**
     * 检查是否为 Mock 模式（无真实硬件时使用）
     */
    bool is_mock_mode() const { return is_mock_; }

private:
    std::string device_;
    int baud_rate_;
    int fd_{-1};
    bool is_mock_{true};

    // ============================================================
    // #### Task 12-4b-2: 配置串口参数 ###########################
    // ============================================================
    static bool configure_serial(int fd, int baud_rate) {
        // TODO: 使用 tcgetattr/tcsetattr 配置串口
        // 参考 Training-26-5 或 26_SP io/serial/serial.cpp
        //
        // struct termios options;
        // tcgetattr(fd, &options);
        //
        // // 根据 baud_rate 参数选择波特率常量:
        // speed_t speed = B115200;  // 默认
        // switch (baud_rate) {
        //   case 9600:   speed = B9600;   break;
        //   case 115200: speed = B115200; break;
        //   case 921600: speed = B921600; break;
        //   // ... 其他波特率
        // }
        // cfsetispeed(&options, speed);
        // cfsetospeed(&options, speed);
        // options.c_cflag |= (CLOCAL | CREAD);
        // options.c_cflag &= ~PARENB;  // 无校验
        // options.c_cflag &= ~CSTOPB;  // 1 停止位
        // options.c_cflag &= ~CSIZE;
        // options.c_cflag |= CS8;      // 8 数据位
        // tcsetattr(fd, TCSANOW, &options);
        
        return true;
    }
};

} // namespace my_auto_aim
