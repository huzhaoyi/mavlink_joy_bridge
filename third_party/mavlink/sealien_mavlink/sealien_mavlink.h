/** @file
 *  @brief MAVLink comm protocol generated from sealien_mavlink.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SEALIEN_MAVLINK_H
#define MAVLINK_SEALIEN_MAVLINK_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SEALIEN_MAVLINK.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_SEALIEN_MVALINK_XML_HASH 2439769977888350562

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 176, 7, 7, 0, 0, 0}, {1, 64, 98, 98, 0, 0, 0}, {2, 189, 149, 149, 0, 0, 0}, {3, 92, 20, 20, 0, 0, 0}, {4, 32, 14, 14, 0, 0, 0}, {5, 119, 16, 16, 0, 0, 0}, {6, 190, 64, 64, 0, 0, 0}, {7, 6, 36, 36, 0, 0, 0}, {8, 151, 16, 16, 0, 0, 0}, {9, 142, 12, 12, 0, 0, 0}, {10, 41, 24, 24, 0, 0, 0}, {11, 203, 1, 1, 0, 0, 0}, {12, 28, 20, 20, 0, 0, 0}, {13, 192, 1, 1, 0, 0, 0}, {14, 0, 2, 2, 0, 0, 0}, {15, 208, 5, 5, 0, 0, 0}, {16, 161, 24, 24, 0, 0, 0}, {17, 67, 2, 2, 0, 0, 0}, {18, 131, 244, 244, 0, 0, 0}, {19, 173, 56, 56, 0, 0, 0}, {20, 76, 164, 164, 0, 0, 0}, {126, 220, 79, 79, 0, 0, 0}, {127, 220, 25, 25, 0, 0, 0}, {128, 66, 54, 54, 0, 0, 0}, {129, 152, 33, 33, 3, 30, 31}}
#endif

#include "../protocol.h"


#define MAVLINK_ENABLED_SEALIEN_MAVLINK

// ENUM DEFINITIONS


/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
   MAV_CMD_DUMMY_ALL=393, /* Dummy/temporary MAV_CMD that causes all.xml to correctly import all commands from both ardupilotmega.xml and development.xml (otherwise only one is imported, for the reasons given in https://github.com/ArduPilot/pymavlink/pull/544#discussion_r2069976980).
          It not be used, and will be removed when the toolchain is fixed. |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
   MAV_CMD_ENUM_END=394, /*  | */
} MAV_CMD;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS


// base include
#include "mavlink_msg_bem280.h"
#include "mavlink_msg_depth_status.h"
#include "mavlink_msg_gs_cfg.h"
#include "mavlink_msg_gs_cmd.h"
#include "mavlink_msg_gs_status.h"
#include "mavlink_msg_heartbeat.h"
#include "mavlink_msg_height_status.h"
#include "mavlink_msg_imu_calib.h"
#include "mavlink_msg_imu_clear.h"
#include "mavlink_msg_imu_data.h"
#include "mavlink_msg_led_cmd.h"
#include "mavlink_msg_led_status.h"
#include "mavlink_msg_serial_control.h"
#include "mavlink_msg_switch_cmd.h"
#include "mavlink_msg_switch_status.h"
#include "mavlink_msg_thruster_cmd.h"
#include "mavlink_msg_thruster_lock.h"
#include "mavlink_msg_thruster_status.h"
#include "mavlink_msg_vcheck.h"
#include "mavlink_msg_param_value.h"
#include "mavlink_msg_statustext.h"
#include "mavlink_msg_command_long.h"
#include "mavlink_msg_dvl_data.h"
#include "mavlink_msg_mixed_io_cmd.h"
#include "mavlink_msg_mixed_io_data.h"


#if MAVLINK_SEALIEN_MVALINK_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_IMU_DATA, MAVLINK_MESSAGE_INFO_THRUSTER_STATUS, MAVLINK_MESSAGE_INFO_GS_STATUS, MAVLINK_MESSAGE_INFO_LED_STATUS, MAVLINK_MESSAGE_INFO_VCHECK, MAVLINK_MESSAGE_INFO_HEIGHT_STATUS, MAVLINK_MESSAGE_INFO_DEPTH_STATUS, MAVLINK_MESSAGE_INFO_BEM280, MAVLINK_MESSAGE_INFO_SWITCH_STATUS, MAVLINK_MESSAGE_INFO_THRUSTER_CMD, MAVLINK_MESSAGE_INFO_THRUSTER_LOCK, MAVLINK_MESSAGE_INFO_IMU_CALIB, MAVLINK_MESSAGE_INFO_IMU_CLEAR, MAVLINK_MESSAGE_INFO_LED_CMD, MAVLINK_MESSAGE_INFO_GS_CMD, MAVLINK_MESSAGE_INFO_GS_CFG, MAVLINK_MESSAGE_INFO_SWITCH_CMD, MAVLINK_MESSAGE_INFO_DVL_DATA, MAVLINK_MESSAGE_INFO_MIXED_IO_CMD, MAVLINK_MESSAGE_INFO_MIXED_IO_DATA, MAVLINK_MESSAGE_INFO_SERIAL_CONTROL, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_COMMAND_LONG}
# define MAVLINK_MESSAGE_NAMES {{ "BEM280", 8 }, { "COMMAND_LONG", 129 }, { "DEPTH_STATUS", 7 }, { "DVL_DATA", 18 }, { "GS_CFG", 16 }, { "GS_CMD", 15 }, { "GS_STATUS", 3 }, { "HEARTBEAT", 0 }, { "HEIGHT_STATUS", 6 }, { "IMU_CALIB", 12 }, { "IMU_CLEAR", 13 }, { "IMU_DATA", 1 }, { "LED_CMD", 14 }, { "LED_STATUS", 4 }, { "MIXED_IO_CMD", 19 }, { "MIXED_IO_DATA", 20 }, { "PARAM_VALUE", 127 }, { "SERIAL_CONTROL", 126 }, { "STATUSTEXT", 128 }, { "SWITCH_CMD", 17 }, { "SWITCH_STATUS", 9 }, { "THRUSTER_CMD", 10 }, { "THRUSTER_LOCK", 11 }, { "THRUSTER_STATUS", 2 }, { "VCHECK", 5 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SEALIEN_MAVLINK_H
