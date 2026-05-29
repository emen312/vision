#include <iostream>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define LIBXR_SYSTEM_POSIX_HOST
#include "libxr_system.hpp"
#include "linux_shared_topic.hpp"

using namespace LibXR;

// ======================================================
// 串口配置：树莓派 -> ESP32
// ======================================================

static constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
// 如果你用树莓派 GPIO 串口，可以改成：
// static constexpr const char* SERIAL_PORT = "/dev/serial0";

static constexpr int SERIAL_BAUDRATE = 115200;

// ======================================================
// 协议配置：必须和 ESP32 main.cpp 一致
// ======================================================

static constexpr uint8_t FRAME_HEADER_1 = 0x55;
static constexpr uint8_t FRAME_HEADER_2 = 0xAA;

static constexpr uint8_t PROTOCOL_VERSION = 0x01;

static constexpr uint8_t MSG_HEARTBEAT = 0x00;
static constexpr uint8_t MSG_TARGET_INFO = 0x01;

// ======================================================
// 必须和 writer.cpp / ESP32 target_packet_t 完全一致
// ======================================================

#pragma pack(push, 1)

struct VisionData
{
    uint32_t timestamp_ms;

    uint16_t frame_id;

    uint16_t image_width;
    uint16_t image_height;

    uint16_t target_x;
    uint16_t target_y;

    uint16_t target_width;
    uint16_t target_height;

    uint32_t target_area;

    int16_t yaw_error;
    int16_t pitch_error;

    uint16_t confidence;

    uint8_t found;
    uint8_t locked;

    uint8_t target_id;
    uint8_t reserved;
};

#pragma pack(pop)

static_assert(sizeof(VisionData) == 32, "VisionData size must be 32 bytes");

// ======================================================
// CRC16-Modbus：必须和 ESP32 main.cpp 一致
// ======================================================

static uint16_t crc16_modbus(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

// ======================================================
// 打开串口
// ======================================================

static speed_t baudrate_to_flag(int baudrate)
{
    switch (baudrate)
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            return B115200;
    }
}

static int open_serial_port(const char* device, int baudrate)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0)
    {
        std::cerr
            << "[ERROR] open serial failed: "
            << device
            << " errno="
            << errno
            << std::endl;

        return -1;
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cerr << "[ERROR] tcgetattr failed" << std::endl;
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    speed_t speed = baudrate_to_flag(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // 不使用硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 启用接收，本地模式
    tty.c_cflag |= CLOCAL | CREAD;

    // 非阻塞 read 行为
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cerr << "[ERROR] tcsetattr failed" << std::endl;
        close(fd);
        return -1;
    }

    std::cout
        << "[INFO] Serial opened: "
        << device
        << " baud="
        << baudrate
        << std::endl;

    return fd;
}

// ======================================================
// 确保完整写入串口
// ======================================================

static bool write_all(int fd, const uint8_t* data, size_t len)
{
    size_t sent = 0;

    while (sent < len)
    {
        ssize_t ret = write(fd, data + sent, len - sent);

        if (ret > 0)
        {
            sent += static_cast<size_t>(ret);
            continue;
        }

        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        std::cerr
            << "[WARN] serial write failed, errno="
            << errno
            << std::endl;

        return false;
    }

    return true;
}

// ======================================================
// 发送一帧协议到 ESP32
//
// 帧格式：
// 55 AA version type seq len payload crc_low crc_high
//
// CRC 范围：
// version type seq len payload
// ======================================================

static bool send_protocol_frame(
    int serial_fd,
    uint8_t msg_type,
    uint8_t seq,
    const uint8_t* payload,
    uint8_t payload_len
)
{
    uint8_t frame[2 + 4 + 64 + 2];

    size_t index = 0;

    frame[index++] = FRAME_HEADER_1;
    frame[index++] = FRAME_HEADER_2;

    frame[index++] = PROTOCOL_VERSION;
    frame[index++] = msg_type;
    frame[index++] = seq;
    frame[index++] = payload_len;

    if (payload_len > 0)
    {
        std::memcpy(&frame[index], payload, payload_len);
        index += payload_len;
    }

    // CRC 计算范围：version type seq len payload
    uint16_t crc = crc16_modbus(&frame[2], 4 + payload_len);

    frame[index++] = static_cast<uint8_t>(crc & 0xFF);
    frame[index++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return write_all(serial_fd, frame, index);
}

static bool send_target_to_esp32(
    int serial_fd,
    uint8_t seq,
    const VisionData& data
)
{
    return send_protocol_frame(
        serial_fd,
        MSG_TARGET_INFO,
        seq,
        reinterpret_cast<const uint8_t*>(&data),
        sizeof(VisionData)
    );
}

// ======================================================
// 主函数
// ======================================================

int main()
{
    PlatformInit();

    int serial_fd = open_serial_port(
        SERIAL_PORT,
        SERIAL_BAUDRATE
    );

    if (serial_fd < 0)
    {
        return -1;
    }

    using VisionTopic = LinuxSharedTopic<VisionData>;

    VisionTopic::Subscriber subscriber;

    while (!subscriber.Valid())
    {
        subscriber = VisionTopic::Subscriber(
            "/vision_topic",
            LinuxSharedSubscriberMode::BROADCAST_DROP_OLD
        );

        if (!subscriber.Valid())
        {
            std::cerr << "[WARN] Waiting for /vision_topic..." << std::endl;

            std::this_thread::sleep_for(
                std::chrono::milliseconds(500)
            );
        }
    }

    std::cout << "[INFO] Vision topic reader started" << std::endl;
    std::cout << "[INFO] Forwarding /vision_topic to ESP32 UART" << std::endl;

    uint8_t seq = 0;

    while (true)
    {
        ErrorCode ret = subscriber.Wait(20);

        if (ret == ErrorCode::OK)
        {
            VisionData* ptr = subscriber.GetData();

            if (ptr != nullptr)
            {
                VisionData data {};
                std::memcpy(&data, ptr, sizeof(VisionData));

                bool ok = send_target_to_esp32(
                    serial_fd,
                    seq++,
                    data
                );

                if (!ok)
                {
                    std::cerr << "[WARN] send target to ESP32 failed" << std::endl;
                }

                std::cout
                    << "UART Send: "
                    << "found=" << static_cast<int>(data.found)
                    << " locked=" << static_cast<int>(data.locked)
                    << " x=" << data.target_x
                    << " y=" << data.target_y
                    << " w=" << data.target_width
                    << " h=" << data.target_height
                    << " area=" << data.target_area
                    << " yaw=" << data.yaw_error
                    << " pitch=" << data.pitch_error
                    << " conf=" << data.confidence
                    << std::endl;
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(2)
        );
    }

    close(serial_fd);

    return 0;
}