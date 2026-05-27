/** @file
 *  @brief MAVLink comm protocol generated from sealien_rc.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SEALIEN_RC_H
#define MAVLINK_SEALIEN_RC_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SEALIEN_RC.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_SEALIEN_RC_XML_HASH -7133031343591240604

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{202, 147, 75, 75, 0, 0, 0}, {203, 182, 108, 108, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SEALIEN_RC

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sealien_rc_input.h"
#include "./mavlink_msg_sealien_io_input.h"

// base include



#if MAVLINK_SEALIEN_RC_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_SEALIEN_RC_INPUT, MAVLINK_MESSAGE_INFO_SEALIEN_IO_INPUT}
# define MAVLINK_MESSAGE_NAMES {{ "SEALIEN_IO_INPUT", 203 }, { "SEALIEN_RC_INPUT", 202 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SEALIEN_RC_H
