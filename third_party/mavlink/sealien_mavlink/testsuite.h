/** @file
 *    @brief MAVLink comm protocol testsuite generated from sealien_mvalink.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef SEALIEN_MAVLINK_TESTSUITE_H
#define SEALIEN_MAVLINK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_Bem280(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_DepthStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_GsCfg(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_GsCmd(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_GsStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_Heartbeat(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_HeightStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_Imucalib(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ImuClear(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ImuData(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_LedCmd(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_LedStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_SerialControl(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_SwitchCmd(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_SwitchStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ThrusterCmd(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ThrusterLock(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ThrusterStatus(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_Vcheck(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ParamValue(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_Statustext(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_CommandLong(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_DVLData(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_MixedIOCmd(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_MixedIOData(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_sealien_mavlink(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_Bem280(system_id, component_id, last_msg);
    mavlink_test_DepthStatus(system_id, component_id, last_msg);
    mavlink_test_GsCfg(system_id, component_id, last_msg);
    mavlink_test_GsCmd(system_id, component_id, last_msg);
    mavlink_test_GsStatus(system_id, component_id, last_msg);
    mavlink_test_Heartbeat(system_id, component_id, last_msg);
    mavlink_test_HeightStatus(system_id, component_id, last_msg);
    mavlink_test_Imucalib(system_id, component_id, last_msg);
    mavlink_test_ImuClear(system_id, component_id, last_msg);
    mavlink_test_ImuData(system_id, component_id, last_msg);
    mavlink_test_LedCmd(system_id, component_id, last_msg);
    mavlink_test_LedStatus(system_id, component_id, last_msg);
    mavlink_test_SerialControl(system_id, component_id, last_msg);
    mavlink_test_SwitchCmd(system_id, component_id, last_msg);
    mavlink_test_SwitchStatus(system_id, component_id, last_msg);
    mavlink_test_ThrusterCmd(system_id, component_id, last_msg);
    mavlink_test_ThrusterLock(system_id, component_id, last_msg);
    mavlink_test_ThrusterStatus(system_id, component_id, last_msg);
    mavlink_test_Vcheck(system_id, component_id, last_msg);
    mavlink_test_ParamValue(system_id, component_id, last_msg);
    mavlink_test_Statustext(system_id, component_id, last_msg);
    mavlink_test_CommandLong(system_id, component_id, last_msg);
    mavlink_test_DVLData(system_id, component_id, last_msg);
    mavlink_test_MixedIOCmd(system_id, component_id, last_msg);
    mavlink_test_MixedIOData(system_id, component_id, last_msg);
    mavlink_test_sealien_mavlink(system_id, component_id, last_msg);
}
#endif

#include "../Bem280/testsuite.h"
#include "../DepthStatus/testsuite.h"
#include "../GsCfg/testsuite.h"
#include "../GsCmd/testsuite.h"
#include "../GsStatus/testsuite.h"
#include "../Heartbeat/testsuite.h"
#include "../HeightStatus/testsuite.h"
#include "../Imucalib/testsuite.h"
#include "../ImuClear/testsuite.h"
#include "../ImuData/testsuite.h"
#include "../LedCmd/testsuite.h"
#include "../LedStatus/testsuite.h"
#include "../SerialControl/testsuite.h"
#include "../SwitchCmd/testsuite.h"
#include "../SwitchStatus/testsuite.h"
#include "../ThrusterCmd/testsuite.h"
#include "../ThrusterLock/testsuite.h"
#include "../ThrusterStatus/testsuite.h"
#include "../Vcheck/testsuite.h"
#include "../ParamValue/testsuite.h"
#include "../Statustext/testsuite.h"
#include "../CommandLong/testsuite.h"
#include "../DVLData/testsuite.h"
#include "../MixedIOCmd/testsuite.h"
#include "../MixedIOData/testsuite.h"


static void mavlink_test_sealien_mavlink(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SEALIEN_MAVLINK_TESTSUITE_H
