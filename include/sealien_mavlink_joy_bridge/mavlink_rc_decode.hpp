#pragma once

#include <cstdint>
#include <cstring>

#include "mavlink.h"

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW (35U)
#define MAVLINK_MSG_ID_RC_CHANNELS (65U)
#define MAVLINK_MSG_ID_MANUAL_CONTROL (69U)

#define ADC_SRC_PWM_US (1U)
#define ADC_SRC_MANUAL_CONTROL (2U)
#define ADC_SRC_SEALIEN_RC (3U)

#define ADC_RAW_MAX_CHANNELS (18U)

static inline int16_t mavlink_read_i16(const uint8_t * buf, size_t offset)
{
    int16_t val = 0;
    std::memcpy(&val, buf + offset, sizeof(val));
    return val;
}

static inline uint16_t mavlink_read_u16(const uint8_t * buf, size_t offset)
{
    uint16_t val = 0U;
    std::memcpy(&val, buf + offset, sizeof(val));
    return val;
}

static inline uint16_t manual_to_pwm_us(int16_t v)
{
    const float norm = static_cast<float>(v) / 1000.0f;
    const float clamped = (norm < -1.0f) ? -1.0f : ((norm > 1.0f) ? 1.0f : norm);
    return static_cast<uint16_t>(1500.0f + clamped * 500.0f);
}

static inline bool mavlink_decode_manual_control(
    const mavlink_message_t * msg,
    uint16_t * channels,
    uint8_t * channel_count)
{
    if (msg == nullptr || channels == nullptr || channel_count == nullptr)
    {
        return false;
    }
    if (msg->len < 10U)
    {
        return false;
    }

    const uint8_t * p = reinterpret_cast<const uint8_t *>(msg->payload64);
    channels[0] = manual_to_pwm_us(mavlink_read_i16(p, 0));
    channels[1] = manual_to_pwm_us(mavlink_read_i16(p, 2));
    channels[2] = manual_to_pwm_us(mavlink_read_i16(p, 4));
    channels[3] = manual_to_pwm_us(mavlink_read_i16(p, 6));
    *channel_count = 4U;
    return true;
}

static inline bool mavlink_decode_rc_channels_raw(
    const mavlink_message_t * msg,
    uint16_t * channels,
    uint8_t * channel_count)
{
    if (msg == nullptr || channels == nullptr || channel_count == nullptr)
    {
        return false;
    }
    if (msg->len < 20U)
    {
        return false;
    }

    const uint8_t * p = reinterpret_cast<const uint8_t *>(msg->payload64);
    uint8_t count = 8U;
    if (msg->len >= 22U)
    {
        count = p[21];
        if (count > 8U)
        {
            count = 8U;
        }
        if (count == 0U)
        {
            count = 8U;
        }
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        channels[i] = mavlink_read_u16(p, 4U + static_cast<size_t>(i) * 2U);
    }
    *channel_count = count;
    return true;
}

static inline bool mavlink_decode_rc_channels(
    const mavlink_message_t * msg,
    uint16_t * channels,
    uint8_t * channel_count)
{
    if (msg == nullptr || channels == nullptr || channel_count == nullptr)
    {
        return false;
    }
    if (msg->len < 36U)
    {
        return false;
    }

    const uint8_t * p = reinterpret_cast<const uint8_t *>(msg->payload64);
    uint8_t count = 8U;
    if (msg->len >= 42U)
    {
        count = p[41];
        if (count > 18U)
        {
            count = 18U;
        }
        if (count == 0U)
        {
            count = 8U;
        }
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        channels[i] = mavlink_read_u16(p, 4U + static_cast<size_t>(i) * 2U);
    }
    *channel_count = count;
    return true;
}

static inline bool mavlink_decode_sealien_rc_input(
    const mavlink_message_t * msg,
    uint16_t * channels,
    uint8_t * channel_count)
{
    if (msg == nullptr || channels == nullptr || channel_count == nullptr)
    {
        return false;
    }

    mavlink_sealien_rc_input_t rc_input {};
    mavlink_msg_sealien_rc_input_decode(msg, &rc_input);

    const uint8_t count = 8U;
    for (uint8_t i = 0U; i < count; i++)
    {
        const float norm = rc_input.ch[i];
        const float clamped = (norm < -1.0f) ? -1.0f : ((norm > 1.0f) ? 1.0f : norm);
        channels[i] = static_cast<uint16_t>(1500.0f + clamped * 500.0f);
    }
    *channel_count = count;
    return true;
}
