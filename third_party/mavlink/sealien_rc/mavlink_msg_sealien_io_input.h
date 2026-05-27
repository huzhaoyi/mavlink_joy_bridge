#pragma once
// MESSAGE SEALIEN_IO_INPUT PACKING

#define MAVLINK_MSG_ID_SEALIEN_IO_INPUT 203


typedef struct __mavlink_sealien_io_input_t {
 uint32_t seq; /*<  sequence*/
 uint32_t timestamp_ms; /*<  timestamp ms*/
 float ai[24]; /*<  analog -1..1*/
 uint32_t di; /*<  digital inputs bitmask*/
} mavlink_sealien_io_input_t;

#define MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN 108
#define MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN 108
#define MAVLINK_MSG_ID_203_LEN 108
#define MAVLINK_MSG_ID_203_MIN_LEN 108

#define MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC 182
#define MAVLINK_MSG_ID_203_CRC 182

#define MAVLINK_MSG_SEALIEN_IO_INPUT_FIELD_AI_LEN 24

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SEALIEN_IO_INPUT { \
    203, \
    "SEALIEN_IO_INPUT", \
    4, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sealien_io_input_t, seq) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sealien_io_input_t, timestamp_ms) }, \
         { "ai", NULL, MAVLINK_TYPE_FLOAT, 24, 8, offsetof(mavlink_sealien_io_input_t, ai) }, \
         { "di", NULL, MAVLINK_TYPE_UINT32_T, 0, 104, offsetof(mavlink_sealien_io_input_t, di) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SEALIEN_IO_INPUT { \
    "SEALIEN_IO_INPUT", \
    4, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sealien_io_input_t, seq) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sealien_io_input_t, timestamp_ms) }, \
         { "ai", NULL, MAVLINK_TYPE_FLOAT, 24, 8, offsetof(mavlink_sealien_io_input_t, ai) }, \
         { "di", NULL, MAVLINK_TYPE_UINT32_T, 0, 104, offsetof(mavlink_sealien_io_input_t, di) }, \
         } \
}
#endif

/**
 * @brief Pack a sealien_io_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ai  analog -1..1
 * @param di  digital inputs bitmask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_io_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t seq, uint32_t timestamp_ms, const float *ai, uint32_t di)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_uint32_t(buf, 104, di);
    _mav_put_float_array(buf, 8, ai, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#else
    mavlink_sealien_io_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.di = di;
    mav_array_assign_float(packet.ai, ai, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_IO_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
}

/**
 * @brief Pack a sealien_io_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ai  analog -1..1
 * @param di  digital inputs bitmask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_io_input_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t seq, uint32_t timestamp_ms, const float *ai, uint32_t di)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_uint32_t(buf, 104, di);
    _mav_put_float_array(buf, 8, ai, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#else
    mavlink_sealien_io_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.di = di;
    mav_array_memcpy(packet.ai, ai, sizeof(float)*24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_IO_INPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#endif
}

/**
 * @brief Pack a sealien_io_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ai  analog -1..1
 * @param di  digital inputs bitmask
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_io_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t seq,uint32_t timestamp_ms,const float *ai,uint32_t di)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_uint32_t(buf, 104, di);
    _mav_put_float_array(buf, 8, ai, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#else
    mavlink_sealien_io_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.di = di;
    mav_array_assign_float(packet.ai, ai, 24);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_IO_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
}

/**
 * @brief Encode a sealien_io_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sealien_io_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_io_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sealien_io_input_t* sealien_io_input)
{
    return mavlink_msg_sealien_io_input_pack(system_id, component_id, msg, sealien_io_input->seq, sealien_io_input->timestamp_ms, sealien_io_input->ai, sealien_io_input->di);
}

/**
 * @brief Encode a sealien_io_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sealien_io_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_io_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sealien_io_input_t* sealien_io_input)
{
    return mavlink_msg_sealien_io_input_pack_chan(system_id, component_id, chan, msg, sealien_io_input->seq, sealien_io_input->timestamp_ms, sealien_io_input->ai, sealien_io_input->di);
}

/**
 * @brief Encode a sealien_io_input struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sealien_io_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_io_input_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sealien_io_input_t* sealien_io_input)
{
    return mavlink_msg_sealien_io_input_pack_status(system_id, component_id, _status, msg,  sealien_io_input->seq, sealien_io_input->timestamp_ms, sealien_io_input->ai, sealien_io_input->di);
}

/**
 * @brief Send a sealien_io_input message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ai  analog -1..1
 * @param di  digital inputs bitmask
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sealien_io_input_send(mavlink_channel_t chan, uint32_t seq, uint32_t timestamp_ms, const float *ai, uint32_t di)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_uint32_t(buf, 104, di);
    _mav_put_float_array(buf, 8, ai, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT, buf, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#else
    mavlink_sealien_io_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.di = di;
    mav_array_assign_float(packet.ai, ai, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT, (const char *)&packet, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#endif
}

/**
 * @brief Send a sealien_io_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sealien_io_input_send_struct(mavlink_channel_t chan, const mavlink_sealien_io_input_t* sealien_io_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sealien_io_input_send(chan, sealien_io_input->seq, sealien_io_input->timestamp_ms, sealien_io_input->ai, sealien_io_input->di);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT, (const char *)sealien_io_input, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sealien_io_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t seq, uint32_t timestamp_ms, const float *ai, uint32_t di)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_uint32_t(buf, 104, di);
    _mav_put_float_array(buf, 8, ai, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT, buf, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#else
    mavlink_sealien_io_input_t *packet = (mavlink_sealien_io_input_t *)msgbuf;
    packet->seq = seq;
    packet->timestamp_ms = timestamp_ms;
    packet->di = di;
    mav_array_assign_float(packet->ai, ai, 24);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_IO_INPUT, (const char *)packet, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE SEALIEN_IO_INPUT UNPACKING


/**
 * @brief Get field seq from sealien_io_input message
 *
 * @return  sequence
 */
static inline uint32_t mavlink_msg_sealien_io_input_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field timestamp_ms from sealien_io_input message
 *
 * @return  timestamp ms
 */
static inline uint32_t mavlink_msg_sealien_io_input_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field ai from sealien_io_input message
 *
 * @return  analog -1..1
 */
static inline uint16_t mavlink_msg_sealien_io_input_get_ai(const mavlink_message_t* msg, float *ai)
{
    return _MAV_RETURN_float_array(msg, ai, 24,  8);
}

/**
 * @brief Get field di from sealien_io_input message
 *
 * @return  digital inputs bitmask
 */
static inline uint32_t mavlink_msg_sealien_io_input_get_di(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  104);
}

/**
 * @brief Decode a sealien_io_input message into a struct
 *
 * @param msg The message to decode
 * @param sealien_io_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_sealien_io_input_decode(const mavlink_message_t* msg, mavlink_sealien_io_input_t* sealien_io_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sealien_io_input->seq = mavlink_msg_sealien_io_input_get_seq(msg);
    sealien_io_input->timestamp_ms = mavlink_msg_sealien_io_input_get_timestamp_ms(msg);
    mavlink_msg_sealien_io_input_get_ai(msg, sealien_io_input->ai);
    sealien_io_input->di = mavlink_msg_sealien_io_input_get_di(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN? msg->len : MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN;
        memset(sealien_io_input, 0, MAVLINK_MSG_ID_SEALIEN_IO_INPUT_LEN);
    memcpy(sealien_io_input, _MAV_PAYLOAD(msg), len);
#endif
}
