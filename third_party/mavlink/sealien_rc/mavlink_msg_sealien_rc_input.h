#pragma once
// MESSAGE SEALIEN_RC_INPUT PACKING

#define MAVLINK_MSG_ID_SEALIEN_RC_INPUT 202


typedef struct __mavlink_sealien_rc_input_t {
 uint32_t seq; /*<  sequence*/
 uint32_t timestamp_ms; /*<  timestamp ms*/
 float ch[16]; /*<  channels normalized*/
 int8_t mode; /*<  mode switch*/
 uint8_t valid_mask; /*<  valid channel bitmask*/
 uint8_t flags; /*<  status flags*/
} mavlink_sealien_rc_input_t;

#define MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN 75
#define MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN 75
#define MAVLINK_MSG_ID_202_LEN 75
#define MAVLINK_MSG_ID_202_MIN_LEN 75

#define MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC 147
#define MAVLINK_MSG_ID_202_CRC 147

#define MAVLINK_MSG_SEALIEN_RC_INPUT_FIELD_CH_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SEALIEN_RC_INPUT { \
    202, \
    "SEALIEN_RC_INPUT", \
    6, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sealien_rc_input_t, seq) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sealien_rc_input_t, timestamp_ms) }, \
         { "ch", NULL, MAVLINK_TYPE_FLOAT, 16, 8, offsetof(mavlink_sealien_rc_input_t, ch) }, \
         { "mode", NULL, MAVLINK_TYPE_INT8_T, 0, 72, offsetof(mavlink_sealien_rc_input_t, mode) }, \
         { "valid_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sealien_rc_input_t, valid_mask) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_sealien_rc_input_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SEALIEN_RC_INPUT { \
    "SEALIEN_RC_INPUT", \
    6, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sealien_rc_input_t, seq) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sealien_rc_input_t, timestamp_ms) }, \
         { "ch", NULL, MAVLINK_TYPE_FLOAT, 16, 8, offsetof(mavlink_sealien_rc_input_t, ch) }, \
         { "mode", NULL, MAVLINK_TYPE_INT8_T, 0, 72, offsetof(mavlink_sealien_rc_input_t, mode) }, \
         { "valid_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sealien_rc_input_t, valid_mask) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_sealien_rc_input_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a sealien_rc_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ch  channels normalized
 * @param mode  mode switch
 * @param valid_mask  valid channel bitmask
 * @param flags  status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_rc_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t seq, uint32_t timestamp_ms, const float *ch, int8_t mode, uint8_t valid_mask, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_int8_t(buf, 72, mode);
    _mav_put_uint8_t(buf, 73, valid_mask);
    _mav_put_uint8_t(buf, 74, flags);
    _mav_put_float_array(buf, 8, ch, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#else
    mavlink_sealien_rc_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.mode = mode;
    packet.valid_mask = valid_mask;
    packet.flags = flags;
    mav_array_assign_float(packet.ch, ch, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_RC_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
}

/**
 * @brief Pack a sealien_rc_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ch  channels normalized
 * @param mode  mode switch
 * @param valid_mask  valid channel bitmask
 * @param flags  status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_rc_input_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t seq, uint32_t timestamp_ms, const float *ch, int8_t mode, uint8_t valid_mask, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_int8_t(buf, 72, mode);
    _mav_put_uint8_t(buf, 73, valid_mask);
    _mav_put_uint8_t(buf, 74, flags);
    _mav_put_float_array(buf, 8, ch, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#else
    mavlink_sealien_rc_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.mode = mode;
    packet.valid_mask = valid_mask;
    packet.flags = flags;
    mav_array_memcpy(packet.ch, ch, sizeof(float)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_RC_INPUT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#endif
}

/**
 * @brief Pack a sealien_rc_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ch  channels normalized
 * @param mode  mode switch
 * @param valid_mask  valid channel bitmask
 * @param flags  status flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sealien_rc_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t seq,uint32_t timestamp_ms,const float *ch,int8_t mode,uint8_t valid_mask,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_int8_t(buf, 72, mode);
    _mav_put_uint8_t(buf, 73, valid_mask);
    _mav_put_uint8_t(buf, 74, flags);
    _mav_put_float_array(buf, 8, ch, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#else
    mavlink_sealien_rc_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.mode = mode;
    packet.valid_mask = valid_mask;
    packet.flags = flags;
    mav_array_assign_float(packet.ch, ch, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SEALIEN_RC_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
}

/**
 * @brief Encode a sealien_rc_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sealien_rc_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_rc_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sealien_rc_input_t* sealien_rc_input)
{
    return mavlink_msg_sealien_rc_input_pack(system_id, component_id, msg, sealien_rc_input->seq, sealien_rc_input->timestamp_ms, sealien_rc_input->ch, sealien_rc_input->mode, sealien_rc_input->valid_mask, sealien_rc_input->flags);
}

/**
 * @brief Encode a sealien_rc_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sealien_rc_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_rc_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sealien_rc_input_t* sealien_rc_input)
{
    return mavlink_msg_sealien_rc_input_pack_chan(system_id, component_id, chan, msg, sealien_rc_input->seq, sealien_rc_input->timestamp_ms, sealien_rc_input->ch, sealien_rc_input->mode, sealien_rc_input->valid_mask, sealien_rc_input->flags);
}

/**
 * @brief Encode a sealien_rc_input struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sealien_rc_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sealien_rc_input_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sealien_rc_input_t* sealien_rc_input)
{
    return mavlink_msg_sealien_rc_input_pack_status(system_id, component_id, _status, msg,  sealien_rc_input->seq, sealien_rc_input->timestamp_ms, sealien_rc_input->ch, sealien_rc_input->mode, sealien_rc_input->valid_mask, sealien_rc_input->flags);
}

/**
 * @brief Send a sealien_rc_input message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  sequence
 * @param timestamp_ms  timestamp ms
 * @param ch  channels normalized
 * @param mode  mode switch
 * @param valid_mask  valid channel bitmask
 * @param flags  status flags
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sealien_rc_input_send(mavlink_channel_t chan, uint32_t seq, uint32_t timestamp_ms, const float *ch, int8_t mode, uint8_t valid_mask, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN];
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_int8_t(buf, 72, mode);
    _mav_put_uint8_t(buf, 73, valid_mask);
    _mav_put_uint8_t(buf, 74, flags);
    _mav_put_float_array(buf, 8, ch, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT, buf, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#else
    mavlink_sealien_rc_input_t packet;
    packet.seq = seq;
    packet.timestamp_ms = timestamp_ms;
    packet.mode = mode;
    packet.valid_mask = valid_mask;
    packet.flags = flags;
    mav_array_assign_float(packet.ch, ch, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT, (const char *)&packet, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#endif
}

/**
 * @brief Send a sealien_rc_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sealien_rc_input_send_struct(mavlink_channel_t chan, const mavlink_sealien_rc_input_t* sealien_rc_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sealien_rc_input_send(chan, sealien_rc_input->seq, sealien_rc_input->timestamp_ms, sealien_rc_input->ch, sealien_rc_input->mode, sealien_rc_input->valid_mask, sealien_rc_input->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT, (const char *)sealien_rc_input, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sealien_rc_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t seq, uint32_t timestamp_ms, const float *ch, int8_t mode, uint8_t valid_mask, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, seq);
    _mav_put_uint32_t(buf, 4, timestamp_ms);
    _mav_put_int8_t(buf, 72, mode);
    _mav_put_uint8_t(buf, 73, valid_mask);
    _mav_put_uint8_t(buf, 74, flags);
    _mav_put_float_array(buf, 8, ch, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT, buf, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#else
    mavlink_sealien_rc_input_t *packet = (mavlink_sealien_rc_input_t *)msgbuf;
    packet->seq = seq;
    packet->timestamp_ms = timestamp_ms;
    packet->mode = mode;
    packet->valid_mask = valid_mask;
    packet->flags = flags;
    mav_array_assign_float(packet->ch, ch, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SEALIEN_RC_INPUT, (const char *)packet, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE SEALIEN_RC_INPUT UNPACKING


/**
 * @brief Get field seq from sealien_rc_input message
 *
 * @return  sequence
 */
static inline uint32_t mavlink_msg_sealien_rc_input_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field timestamp_ms from sealien_rc_input message
 *
 * @return  timestamp ms
 */
static inline uint32_t mavlink_msg_sealien_rc_input_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field ch from sealien_rc_input message
 *
 * @return  channels normalized
 */
static inline uint16_t mavlink_msg_sealien_rc_input_get_ch(const mavlink_message_t* msg, float *ch)
{
    return _MAV_RETURN_float_array(msg, ch, 16,  8);
}

/**
 * @brief Get field mode from sealien_rc_input message
 *
 * @return  mode switch
 */
static inline int8_t mavlink_msg_sealien_rc_input_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  72);
}

/**
 * @brief Get field valid_mask from sealien_rc_input message
 *
 * @return  valid channel bitmask
 */
static inline uint8_t mavlink_msg_sealien_rc_input_get_valid_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field flags from sealien_rc_input message
 *
 * @return  status flags
 */
static inline uint8_t mavlink_msg_sealien_rc_input_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Decode a sealien_rc_input message into a struct
 *
 * @param msg The message to decode
 * @param sealien_rc_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_sealien_rc_input_decode(const mavlink_message_t* msg, mavlink_sealien_rc_input_t* sealien_rc_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sealien_rc_input->seq = mavlink_msg_sealien_rc_input_get_seq(msg);
    sealien_rc_input->timestamp_ms = mavlink_msg_sealien_rc_input_get_timestamp_ms(msg);
    mavlink_msg_sealien_rc_input_get_ch(msg, sealien_rc_input->ch);
    sealien_rc_input->mode = mavlink_msg_sealien_rc_input_get_mode(msg);
    sealien_rc_input->valid_mask = mavlink_msg_sealien_rc_input_get_valid_mask(msg);
    sealien_rc_input->flags = mavlink_msg_sealien_rc_input_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN? msg->len : MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN;
        memset(sealien_rc_input, 0, MAVLINK_MSG_ID_SEALIEN_RC_INPUT_LEN);
    memcpy(sealien_rc_input, _MAV_PAYLOAD(msg), len);
#endif
}
