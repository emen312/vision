#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdint>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#define LIBXR_SYSTEM_POSIX_HOST
#include "libxr_system.hpp"
#include "linux_shared_topic.hpp"


using namespace LibXR;

// ======================================================
// LibXR Topic 数据结构
// 和 ESP32 下位机 target_packet_t 保持一致
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
// 协议配置
// ======================================================

static constexpr uint8_t FRAME_HEADER_1 = 0x55;
static constexpr uint8_t FRAME_HEADER_2 = 0xAA;

static constexpr uint8_t PROTOCOL_VERSION = 0x01;

static constexpr uint8_t MSG_HEARTBEAT = 0x00;
static constexpr uint8_t MSG_TARGET_INFO = 0x01;

static constexpr int UDP_LISTEN_PORT = 5005;

// ======================================================
// CRC16-Modbus
// ======================================================

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
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
// 创建 UDP 接收 socket
// ======================================================

static int create_udp_socket()
{
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock_fd < 0)
    {
        std::cerr << "[ERROR] socket create failed" << std::endl;
        return -1;
    }

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_LISTEN_PORT);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (bind(sock_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        std::cerr << "[ERROR] bind 127.0.0.1:" << UDP_LISTEN_PORT << " failed" << std::endl;
        close(sock_fd);
        return -1;
    }

    std::cout << "[INFO] UDP listening on 127.0.0.1:" << UDP_LISTEN_PORT << std::endl;

    return sock_fd;
}

// ======================================================
// 解析一帧协议
//
// 帧格式：
// 0x55 0xAA version type seq len payload crc_low crc_high
//
// CRC 范围：
// version type seq len payload
// ======================================================

static bool parse_target_frame(const uint8_t *buffer, int len, VisionData &out)
{
    // 最短帧：55 AA version type seq len crc_low crc_high
    if (len < 8)
    {
        return false;
    }

    if (buffer[0] != FRAME_HEADER_1 || buffer[1] != FRAME_HEADER_2)
    {
        return false;
    }

    uint8_t version = buffer[2];
    uint8_t msg_type = buffer[3];
    uint8_t seq = buffer[4];
    uint8_t payload_len = buffer[5];

    (void)seq;

    if (version != PROTOCOL_VERSION)
    {
        return false;
    }

    int expected_len = 2 + 4 + payload_len + 2;

    if (len != expected_len)
    {
        return false;
    }

    const uint8_t *payload = &buffer[6];

    uint16_t recv_crc =
        static_cast<uint16_t>(buffer[6 + payload_len])
        | (static_cast<uint16_t>(buffer[6 + payload_len + 1]) << 8);

    uint16_t calc_crc = crc16_modbus(
        &buffer[2],
        4 + payload_len
    );

    if (recv_crc != calc_crc)
    {
        std::cerr << "[WARN] CRC error" << std::endl;
        return false;
    }

    if (msg_type == MSG_HEARTBEAT)
    {
        return false;
    }

    if (msg_type != MSG_TARGET_INFO)
    {
        return false;
    }

    if (payload_len != sizeof(VisionData))
    {
        std::cerr << "[WARN] payload size error: " << static_cast<int>(payload_len) << std::endl;
        return false;
    }

    std::memcpy(&out, payload, sizeof(VisionData));

    return true;
}

// ======================================================
// 主函数
// ======================================================

int main()
{
    PlatformInit();

    LinuxSharedTopicConfig config {};
    config.slot_num = 8;
    config.subscriber_num = 2;
    config.queue_num = 8;

    LinuxSharedTopic<VisionData> topic(
        "/vision_topic",
        config
    );

    std::cout
        << "[DEBUG] topic.Valid()="
        << topic.Valid()
        << std::endl;

    int sock_fd = create_udp_socket();

    if (sock_fd < 0)
    {
        return -1;
    }

    std::cout << "[INFO] Vision IPC writer started" << std::endl;

    while (true)
    {
        uint8_t buffer[128];

        ssize_t recv_len = recv(
            sock_fd,
            buffer,
            sizeof(buffer),
            0
        );

        if (recv_len <= 0)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1)
            );

            continue;
        }

        std::cout << "[DEBUG] UDP recv len=" << recv_len << std::endl;

        VisionData data {};

        if (parse_target_frame(buffer, static_cast<int>(recv_len), data))
        {
            ErrorCode ret = topic.Publish(data);

            if (ret != ErrorCode::OK)
            {
                std::cerr
                    << "[WARN] topic publish failed, error="
                    << static_cast<int>(ret)
                    << std::endl;

                continue;
            }

            std::cout
                << "Publish: "
                << "found=" << static_cast<int>(data.found)
                << " locked=" << static_cast<int>(data.locked)
                << " x=" << data.target_x
                << " y=" << data.target_y
                << " w=" << data.target_width
                << " h=" << data.target_height
                << " yaw=" << data.yaw_error
                << " pitch=" << data.pitch_error
                << " conf=" << data.confidence
                << std::endl;
        }
    }

    close(sock_fd);

    return 0;
}