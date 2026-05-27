/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. *****
 * Send MIXED_IO_DATA (24 ADC V + GPIO input bitmask) over UDP.
 *****/
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>

#include "sealien_mavlink/mavlink.h"

#define MIXED_IO_SEND_ADC_COUNT (24U)

static void print_usage(const char * prog)
{
    std::fprintf(
        stderr,
        "Usage: %s <host> <port> <gpio_lo> <gpio_hi> <adc0_V> ... <adc23_V>\n"
        "  gpio_lo: uint32, GPIO 1-32 input bitmask\n"
        "  gpio_hi: uint32, GPIO 33-64 input bitmask\n"
        "  adc*: float voltage in volts\n",
        prog);
}

int main(int argc, char * argv[])
{
    if (argc < 5 + static_cast<int>(MIXED_IO_SEND_ADC_COUNT))
    {
        print_usage(argv[0]);
        return 1;
    }

    const char * host = argv[1];
    const int port = std::atoi(argv[2]);
    const uint32_t gpio_lo = static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 0));
    const uint32_t gpio_hi = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 0));

    float adc_v[MIXED_IO_SEND_ADC_COUNT] = {0.0f};
    for (uint8_t i = 0U; i < MIXED_IO_SEND_ADC_COUNT; i++)
    {
        adc_v[i] = static_cast<float>(std::atof(argv[5 + i]));
    }

    float dac_zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

    const uint32_t timestamp_ms = static_cast<uint32_t>(std::time(nullptr) * 1000U);

    mavlink_message_t msg {};
    mavlink_msg_mixed_io_data_pack(
        255,
        1,
        &msg,
        timestamp_ms,
        0U,
        0U,
        gpio_lo,
        gpio_hi,
        dac_zero,
        dac_zero,
        dac_zero,
        adc_v,
        adc_v + 8,
        adc_v + 16);

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

    return 0;
}
