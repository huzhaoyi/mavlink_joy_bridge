/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. *****
 * Send SEALIEN_IO_INPUT (24 AI + DI bitmask) over UDP.
 *****/
#include <arpa/inet.h>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "mavlink.h"

#define IO_SEND_AI_COUNT (24U)

static void print_usage(const char * prog)
{
    std::fprintf(
        stderr,
        "Usage: %s <host> <port> <di_mask> <ai0> <ai1> ... <ai23>\n"
        "  di_mask: uint32, bit i = DI i (0/1)\n"
        "  ai*: float in [-1, 1]\n",
        prog);
}

int main(int argc, char * argv[])
{
    if (argc < 4 + static_cast<int>(IO_SEND_AI_COUNT))
    {
        print_usage(argv[0]);
        return 1;
    }

    const char * host = argv[1];
    const int port = std::atoi(argv[2]);
    const uint32_t di = static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 0));

    float ai[IO_SEND_AI_COUNT] = {0.0f};
    for (uint8_t i = 0U; i < IO_SEND_AI_COUNT; i++)
    {
        ai[i] = static_cast<float>(std::atof(argv[4 + i]));
        if (ai[i] < -1.0f)
        {
            ai[i] = -1.0f;
        }
        if (ai[i] > 1.0f)
        {
            ai[i] = 1.0f;
        }
    }

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        std::perror("socket");
        return 1;
    }

    struct sockaddr_in dest {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &dest.sin_addr) <= 0)
    {
        std::fprintf(stderr, "invalid host\n");
        close(sock_fd);
        return 1;
    }

    mavlink_message_t msg {};
    const uint32_t seq = static_cast<uint32_t>(std::time(nullptr));
    mavlink_msg_sealien_io_input_pack(
        255,
        1,
        &msg,
        seq,
        seq,
        ai,
        di & 0x00FFFFFFU);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    const ssize_t sent = sendto(
        sock_fd,
        buf,
        len,
        0,
        reinterpret_cast<struct sockaddr *>(&dest),
        sizeof(dest));
    close(sock_fd);

    if (sent < 0)
    {
        std::perror("sendto");
        return 1;
    }

    std::printf(
        "sent %zd bytes SEALIEN_IO_INPUT to %s:%d di=0x%06x\n",
        sent,
        host,
        port,
        di & 0x00FFFFFFU);
    return 0;
}
