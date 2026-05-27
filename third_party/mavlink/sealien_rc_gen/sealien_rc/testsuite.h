/** @file
 *    @brief MAVLink comm protocol testsuite generated from sealien_rc.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef SEALIEN_RC_TESTSUITE_H
#define SEALIEN_RC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_sealien_rc(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_sealien_rc(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_sealien_rc_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SEALIEN_RC_INPUT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sealien_rc_input_t packet_in = {
        963497464,963497672,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0 },221,32,99
    };
    mavlink_sealien_rc_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.mode = packet_in.mode;
        packet1.valid_mask = packet_in.valid_mask;
        packet1.flags = packet_in.flags;
        
        mav_array_memcpy(packet1.ch, packet_in.ch, sizeof(float)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SEALIEN_RC_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_rc_input_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sealien_rc_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_rc_input_pack(system_id, component_id, &msg , packet1.seq , packet1.timestamp_ms , packet1.ch , packet1.mode , packet1.valid_mask , packet1.flags );
    mavlink_msg_sealien_rc_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_rc_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq , packet1.timestamp_ms , packet1.ch , packet1.mode , packet1.valid_mask , packet1.flags );
    mavlink_msg_sealien_rc_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sealien_rc_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_rc_input_send(MAVLINK_COMM_1 , packet1.seq , packet1.timestamp_ms , packet1.ch , packet1.mode , packet1.valid_mask , packet1.flags );
    mavlink_msg_sealien_rc_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SEALIEN_RC_INPUT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SEALIEN_RC_INPUT) != NULL);
#endif
}

static void mavlink_test_sealien_io_input(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SEALIEN_IO_INPUT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sealien_io_input_t packet_in = {
        963497464,963497672,{ 73.0, 74.0, 75.0, 76.0, 77.0, 78.0, 79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0, 92.0, 93.0, 94.0, 95.0, 96.0 },963502872
    };
    mavlink_sealien_io_input_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.seq = packet_in.seq;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.di = packet_in.di;
        
        mav_array_memcpy(packet1.ai, packet_in.ai, sizeof(float)*24);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SEALIEN_IO_INPUT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_io_input_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sealien_io_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_io_input_pack(system_id, component_id, &msg , packet1.seq , packet1.timestamp_ms , packet1.ai , packet1.di );
    mavlink_msg_sealien_io_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_io_input_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.seq , packet1.timestamp_ms , packet1.ai , packet1.di );
    mavlink_msg_sealien_io_input_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sealien_io_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sealien_io_input_send(MAVLINK_COMM_1 , packet1.seq , packet1.timestamp_ms , packet1.ai , packet1.di );
    mavlink_msg_sealien_io_input_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SEALIEN_IO_INPUT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SEALIEN_IO_INPUT) != NULL);
#endif
}

static void mavlink_test_sealien_rc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_sealien_rc_input(system_id, component_id, last_msg);
    mavlink_test_sealien_io_input(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SEALIEN_RC_TESTSUITE_H
