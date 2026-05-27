/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. *****
 * Send MAVLink RC message over UDP (web-sim / test).
 *****/
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "mavlink.h"

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW (35U)
#define MAVLINK_MSG_ID_MANUAL_CONTROL (69U)

static void print_usage(const char * prog)
{
    std::fprintf(
        stderr,
        "Usage: %s <type> <host> <port> <mode> <ch0> <ch1> <ch2> <ch3>\n"
        "  type: manual_control | rc_channels_raw | sealien_rc\n",
        prog);
}

static uint16_t float_to_pwm(float norm)
{
    if (norm < -1.0f)
    {
        norm = -1.0f;
    }
    if (norm > 1.0f)
    {
        norm = 1.0f;
    }
    return static_cast<uint16_t>(1500.0f + norm * 500.0f);
}

static bool pack_manual_control(
    float ch0,
    float ch1,
    float ch2,
    float ch3,
    mavlink_message_t * msg)
{
    int16_t payload[5] = {0};
    payload[0] = static_cast<int16_t>(ch0 * 1000.0f);
    payload[1] = static_cast<int16_t>(ch1 * 1000.0f);
    payload[2] = static_cast<int16_t>(ch2 * 1000.0f);
    payload[3] = static_cast<int16_t>(ch3 * 1000.0f);

    std::memset(msg, 0, sizeof(*msg));
    std::memcpy(_MAV_PAYLOAD_NON_CONST(msg), payload, 10);
    msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
    mavlink_finalize_message(msg, 255, 1, 10, 10, 243);
    return true;
}

static bool pack_rc_channels_raw(
    float ch0,
    float ch1,
    float ch2,
    float ch3,
    mavlink_message_t * msg)
{
    uint8_t payload[22] = {0};
    const uint16_t c0 = float_to_pwm(ch0);
    const uint16_t c1 = float_to_pwm(ch1);
    const uint16_t c2 = float_to_pwm(ch2);
    const uint16_t c3 = float_to_pwm(ch3);

    std::memcpy(payload + 4, &c0, 2);
    std::memcpy(payload + 6, &c1, 2);
    std::memcpy(payload + 8, &c2, 2);
    std::memcpy(payload + 10, &c3, 2);
    payload[20] = 255U;
    payload[21] = 8U;

    std::memset(msg, 0, sizeof(*msg));
    std::memcpy(_MAV_PAYLOAD_NON_CONST(msg), payload, 22);
    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
    mavlink_finalize_message(msg, 255, 1, 22, 22, 244);
    return true;
}

int main(int argc, char * argv[])
{
    if (argc < 9)
    {
        print_usage(argv[0]);
        return 1;
    }

    const char * type = argv[1];
    const char * host = argv[2];
    const int port = std::atoi(argv[3]);
    const int mode = std::atoi(argv[4]);
    float ch[4] = {0.0f};
    for (int i = 0; i < 4; i++)
    {
        ch[i] = static_cast<float>(std::atof(argv[5 + i]));
    }

    mavlink_message_t msg {};
    bool packed = false;

    if (std::strcmp(type, "manual_control") == 0)
    {
        packed = pack_manual_control(ch[0], ch[1], ch[2], ch[3], &msg);
    }
    else if (std::strcmp(type, "rc_channels_raw") == 0)
    {
        packed = pack_rc_channels_raw(ch[0], ch[1], ch[2], ch[3], &msg);
    }
    else if (std::strcmp(type, "sealien_rc") == 0)
    {
        float ch16[16] = {0.0f};
        for (int i = 0; i < 4; i++)
        {
            ch16[i] = ch[i];
        }
        mavlink_msg_sealien_rc_input_pack(
            255,
            1,
            &msg,
            1U,
            1000U,
            ch16,
            static_cast<int8_t>(mode),
            0x0FU,
            0U);
        packed = true;
    }
    else
    {
        std::fprintf(stderr, "unknown type: %s\n", type);
        return 1;
    }

    if (!packed)
    {
        std::fprintf(stderr, "pack failed\n");
        return 1;
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::perror("socket");
        return 1;
    }

    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0)
    {
        std::fprintf(stderr, "invalid host: %s\n", host);
        close(fd);
        return 1;
    }

    const ssize_t sent = sendto(
        fd,
        buf,
        len,
        0,
        reinterpret_cast<struct sockaddr *>(&addr),
        sizeof(addr));
    close(fd);

    if (sent < 0)
    {
        std::perror("sendto");
        return 1;
    }

    std::printf(
        "sent %zd bytes type=%s msgid=%u to %s:%d mode=%d ch=[%.3f,%.3f,%.3f,%.3f]\n",
        sent,
        type,
        msg.msgid,
        host,
        port,
        mode,
        ch[0],
        ch[1],
        ch[2],
        ch[3]);
    return 0;
}
