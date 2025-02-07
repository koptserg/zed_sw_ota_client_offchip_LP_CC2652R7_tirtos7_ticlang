/**************************************************************************************************
  Filename:       zcl_samplesw.h
  Revised:        $Date: 2015-08-19 17:11:00 -0700 (Wed, 19 Aug 2015) $
  Revision:       $Revision: 44460 $


  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


  Copyright 2006-2013 Texas Instruments Incorporated.

  All rights reserved not granted herein.
  Limited License.

  Texas Instruments Incorporated grants a world-wide, royalty-free,
  non-exclusive license under copyrights and patents it now or hereafter
  owns or controls to make, have made, use, import, offer to sell and sell
  ("Utilize") this software subject to the terms herein. With respect to the
  foregoing patent license, such license is granted solely to the extent that
  any such patent is necessary to Utilize the software alone. The patent
  license shall not apply to any combinations which include this software,
  other than combinations with devices manufactured by or for TI ("TI
  Devices"). No hardware patent is licensed hereunder.

  Redistributions must preserve existing copyright notices and reproduce
  this license (including the above copyright notice and the disclaimer and
  (if applicable) source code license limitations below) in the documentation
  and/or other materials provided with the distribution.

  Redistribution and use in binary form, without modification, are permitted
  provided that the following conditions are met:

    * No reverse engineering, decompilation, or disassembly of this software
      is permitted with respect to any software provided in binary form.
    * Any redistribution and use are licensed by TI for use only with TI Devices.
    * Nothing shall obligate TI to provide you with source code for the software
      licensed and provided to you in object code.

  If software source code is provided to you, modification and redistribution
  of the source code are permitted provided that the following conditions are
  met:

    * Any redistribution and use of the source code, including any resulting
      derivative works, are licensed by TI for use only with TI Devices.
    * Any redistribution and use of any object code compiled from the source
      code and any resulting derivative works, are licensed by TI for use
      only with TI Devices.

  Neither the name of Texas Instruments Incorporated nor the names of its
  suppliers may be used to endorse or promote products derived from this
  software without specific prior written permission.

  DISCLAIMER.

  THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**************************************************************************************************/

#ifndef ZCL_SAMPLESW_H
#define ZCL_SAMPLESW_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "nvintf.h"

/*********************************************************************
 * CONSTANTS
 */
#define SAMPLESW_ENDPOINT               8
#define SAMPLESW_SENSORENDPOINT         9

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01
#define LIGHT_UNKNOWN                   0xFF

#define maxReportingInterval 1800
#define minReportingInterval 3

// Events for the sample app
#define SAMPLEAPP_END_DEVICE_REJOIN_EVT   0x0001
#define SAMPLEAPP_PROV_CONNECT_EVT        0x0002
#define SAMPLEAPP_PROV_DISCONNECT_EVT     0x0004
#define SAMPLEAPP_GET_NWK_INFO_EVT        0x0008
#define SAMPLEAPP_SYNC_ATTR_EVT           0x0010

#if defined (BH1750) || defined (BME280)
#define SAMPLEAPP_SENSOR_START_EVT          0x0020
#ifdef CUSTOM_TASK
#define SAMPLEAPP_SENSOR_START_EVT_1      0x0001
#endif //CUSTOM_TASK
#endif

#ifdef BH1750
#define SAMPLEAPP_BH1750_SEND_EVT         0x0040
#endif
#ifdef BME280
#define SAMPLEAPP_BME280_SEND_EVT         0x0080
#endif
#ifdef EPD1IN54V2
#define SAMPLEAPP_APP_EPD_PARTIAL_EVT       0x1000
#define SAMPLEAPP_APP_EPD_DELAY_EVT       0x2000
#endif
#ifdef CUI_DISABLE
#define SAMPLEAPP_KEY_EVT         0x4000
#endif

#if defined (Z_POWER_TEST)
#define SAMPLEAPP_POWER_TEST_START_EVT    0x1000
#if defined (POWER_TEST_DATA_ACK)
#define SAMPLEAPP_POWER_TEST_TOGGLE_EVT   0x2000
#endif
#endif // Z_POWER_TEST

#if defined(DMM_ZCSWITCH) && defined(NWK_TOPOLOGY_DISCOVERY)
#define SAMPLEAPP_NWK_DISC_EVT            0x0080
#endif

// Green Power Events
#define SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT              0x0100
#define SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT       0x0200
#define SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT            0x0400

#define SAMPLEAPP_POLICY_UPDATE_EVT       0x0800

#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY 1000
#define SAMPLEAPP_CONFIG_SYNC_TIMEOUT     500

#if defined (BDB_TL_TARGET) || defined (BDB_TL_INITIATOR)
#define TL_BDB_FB_EVT                     0x0100
#define TL_BDB_FB_DELAY                   16000
#endif // defined ( BDB_TL_TARGET ) || defined (BDB_TL_INITIATOR)

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleSw_SimpleDesc;
extern SimpleDescriptionFormat_t zclSampleSw_SensorSimpleDesc;

extern SimpleDescriptionFormat_t zclSampleSw9_SimpleDesc;

extern CONST zclAttrRec_t zclSampleSw_Attrs[];
extern CONST zclAttrRec_t zclSampleSw_SensorAttrs[];

extern uint8_t  zclSampleSw_OnOff;

extern uint16_t zclSampleSw_IdentifyTime;

extern CONST uint8_t zclSampleSw_NumAttributes;
extern CONST uint8_t zclSampleSw_SensorNumAttributes;

#ifdef BH1750
extern uint16_t zclSampleSw_IlluminanceMeasurment_MeasuredValue;
#endif

#ifdef BME280
extern int16_t zclSampleSw_Temperature_Sensor_MeasuredValue;
extern uint16_t zclSampleSw_HumiditySensor_MeasuredValue;
extern int16_t zclSampleSw_PressureSensor_MeasuredValue;
extern int16_t zclSampleSw_PressureSensor_ScaledValue;
extern int8_t zclSampleSw_PressureSensor_Scale;
#endif

extern uint32_t DisplayFramePartial;
extern uint32_t zclSampleSw_GenTime_TimeUTC;

extern uint8_t zclSampleSw_batteryVoltage;
extern uint32_t zclSampleSw_batteryVoltageRaw;
extern uint8_t zclSampleSw_PercentageRemainig;

extern uint32_t zclSampleSw_FileOffset;
extern uint8_t  zclSampleSw_ImageUpgradeStatus;

/*********************************************************************
 * FUNCTIONS
 */

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSampleSw_ResetAttributesToDefaultValues(void); //implemented in zcl_samplesw_data.c

/*
 *  Function to toggle the remote light
 */
extern void zclSampleSw_actionToggleLight(const int32_t _itemEntry);

/*
 *  Function to allow switch be discovered by light device to get reports.
 */
extern void zclSampleSw_UiActionSwDiscoverable(const int32_t _itemEntry);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEAPP_H */
