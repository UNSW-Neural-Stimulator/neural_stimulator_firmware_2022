/***************************************************************************//**
* \file CY_BLE.c
* \version 2.20
* 
* \brief
*  This file contains the source code of initialization of the config structure
*  for BLE.
* 
********************************************************************************
* \copyright
* Copyright 2017-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "BLE_config.h"

#if (CY_BLE_HOST_CONTR_CORE)    
#include "flash/cy_flash.h"
#include "ble/cy_ble_event_handler.h"
#include "cyfitter_sysint_cfg.h"

#if (CY_BLE_MODE_PROFILE)
/***************************************
* Global Variables
***************************************/
/* Initializes the cy_stc_ble_gapp_disc_mode_info_t cy_ble_discoveryModeInfo  structure */
#if (CY_BLE_GAP_ROLE_PERIPHERAL || CY_BLE_GAP_ROLE_BROADCASTER)
static cy_stc_ble_gapp_adv_params_t cy_ble_gappAdvConfig[0x01u] = {

    /* Peripheral configuration 0 */
    {
        .fastAdvIntervalMin                 = 0x0020u, 
        .fastAdvIntervalMax                 = 0x0030u, 
        .fastAdvTimeOut                     = 0x0000u, 
        .slowAdvEnable                      = 0x00u, 
        .slowAdvIntervalMin                 = 0x0640u, 
        .slowAdvIntervalMax                 = 0x4000u, 
        .slowAdvTimeOut                     = 0x0096u, 
    },
};


cy_stc_ble_gapp_disc_param_t cy_ble_discoveryParam[0x01u] = {

    /* Peripheral configuration 0 */
    {
        0x0020u, /* uint16_t advertising_interval_min */ 
        0x0030u, /* uint16_t advertising_interval_max */ 
        CY_BLE_GAPP_CONNECTABLE_UNDIRECTED_ADV, /* uint8_t advertising_type */ 
        0x00u, /* uint8_t own_addr_type */ 
        0x00u, /* uint8_t direct_addr_type */ 
        {0x00u, 0x00u, 0x00u, 0x50u, 0xA0u, 0x00u}, /* uint8_t* direct_addr */ 
        0x07u, /* uint8_t advertising_channel_map */ 
        0x00u, /* uint8_t advertising_filter_policy */ 
    },
};


cy_stc_ble_gapp_disc_data_t cy_ble_discoveryData[0x01u] = {

    /* Peripheral configuration 0 */
    {
        { 0x02u, 0x01u, 0x06u, 0x06u, 0x09u, 0x6Eu, 0x73u,
        0x74u, 0x69u, 0x6Du, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u }, /* uint8_t advertising_data[CY_BLE_MAX_ADV_DATA_LEN] */ 
        0x0Au, /* uint8_t adv_data_length */ 
    },
};


cy_stc_ble_gapp_scan_rsp_data_t cy_ble_scanRspData[0x01u] = {

    /* Peripheral configuration 0 */
    {
        { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u }, /* uint8_t scan_rsp_data[CY_BLE_MAX_SCAN_RSP_DATA_LEN] */ 
        0x00u, /* uint8_t scan_rsp_data_length */ 
    },
};


/* This array of type cy_stc_ble_gapp_disc_mode_info_t is present only when the 
   BLE component is configured for Peripheral or Broadcaster GAP roles. 
   It contains the Advertisement settings and also the Advertisement and 
   Scan response data parameters entered in the customizer. This variable
   can be used by advanced users to change Advertisement settings,
   Advertisement or Scan response data in runtime. 
*/
cy_stc_ble_gapp_disc_mode_info_t cy_ble_discoveryModeInfo[0x01u] = {

    /* Peripheral configuration 0 */
    {
        0x02u, /* uint8_t discMode */ 
        &cy_ble_discoveryParam[0], 
        &cy_ble_discoveryData[0], 
        &cy_ble_scanRspData[0], 
        0x0000u, /* uint16_t advTo */ 
    },
};


#endif /* CY_BLE_GAP_ROLE_PERIPHERAL || CY_BLE_GAP_ROLE_BROADCASTER */

/* Initializes the cy_stc_ble_gapc_disc_info_t  cy_ble_discoveryInfo  structure */
#if (CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_OBSERVER)

#endif /* CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_OBSERVER */

#if ((CY_BLE_MODE_PROFILE) && (CY_BLE_BONDING_REQUIREMENT == CY_BLE_BONDING_YES))

#if(CY_BLE_MODE_PROFILE)
    CY_SECTION(".cy_em_eeprom") CY_ALIGN(CY_FLASH_SIZEOF_ROW) static const cy_stc_ble_flash_storage_t cy_ble_flashStorage =
    {
        { 0x00u }, 
        {{
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        },
        {
            0x00u, 0x00u, 
            0x00u /* CRC */
        }}, 
        0x02u, /* CY_BLE_GATT_DB_CCCD_COUNT */ 
        0x11u, 
    };
#endif /* (CY_BLE_MODE_PROFILE) */

#endif  /* (CY_BLE_MODE_PROFILE) && (CY_BLE_BONDING_REQUIREMENT == CY_BLE_BONDING_YES) */

#if (CY_BLE_GATT_ROLE_SERVER)
static const cy_stc_ble_gatts_t cy_ble_gatts =
{
    0x000Cu,    /* Handle of the GATT service */
    0x000Eu,    /* Handle of the Service Changed characteristic */
    0x000Fu,    /* Handle of the Client Characteristic Configuration descriptor */
};
static const cy_stc_ble_gaps_t cy_ble_gaps =
{
    0x0001u,    /* Handle of the GAP service */
    0x0003u,    /* Handle of the Device Name characteristic */
    0x0005u,    /* Handle of the Appearance characteristic */
    0x0007u,    /* Handle of the Peripheral Preferred Connection Parameters characteristic */
    0x0009u,    /* Handle of the Central Address Resolution characteristic */
    0x000Bu,    /* Handle of the Resolvable Private Address Only characteristic */
};
static uint8_t cy_ble_attValues[0x01D9u] = {
    /* Device Name */
    (uint8_t)'n', (uint8_t)'s', (uint8_t)'t', (uint8_t)'i', (uint8_t)'m', 

    /* Appearance */
    0x00u, 0x00u, 

    /* Peripheral Preferred Connection Parameters */
    0x06u, 0x00u, 0x28u, 0x00u, 0x00u, 0x00u, 0xE8u, 0x03u, 

    /* Central Address Resolution */
    0x00u, 

    /* Resolvable Private Address Only */
    0x00u, 

    /* Service Changed */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* command */
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    (uint8_t)'W', (uint8_t)'r', (uint8_t)'i', (uint8_t)'t', (uint8_t)'e', (uint8_t)' ', (uint8_t)'V', (uint8_t)'a',
(uint8_t)'l', 0xA5u, 0x37u, 0xD8u, 0x36u, 0xD7u, 0x06u, 0x33u, 0x8Eu, 0xDFu, 0x40u, 0xD6u, 0x44u, 0x96u, 0x59u, 0xD1u,
0x3Fu, 

    /* Characteristic User Description */
    (uint8_t)'W', (uint8_t)'r', (uint8_t)'i', (uint8_t)'t', (uint8_t)'e', (uint8_t)' ', (uint8_t)'c', (uint8_t)'o',
(uint8_t)'m', (uint8_t)'m', (uint8_t)'a', (uint8_t)'n', (uint8_t)'d', (uint8_t)' ', (uint8_t)'f', (uint8_t)'o',
(uint8_t)'r', (uint8_t)' ', (uint8_t)'b', (uint8_t)'o', (uint8_t)'a', (uint8_t)'r', (uint8_t)'d', 

    /* err */
    0x00u, 

    /* Custom Descriptor */
    0x00u, 0x5Bu, 0x48u, 0x3Eu, 0x73u, 0xDCu, 0x08u, 0x0Au, 0xA1u, 0x8Bu, 0x40u, 0xABu, 0x50u, 0x40u, 0x54u, 0xC3u, 0x0Fu, 

    /* Characteristic User Description */
    (uint8_t)'R', (uint8_t)'e', (uint8_t)'a', (uint8_t)'d', (uint8_t)' ', (uint8_t)'e', (uint8_t)'r', (uint8_t)'r',
(uint8_t)' ', (uint8_t)'v', (uint8_t)'a', (uint8_t)'l', (uint8_t)'u', (uint8_t)'e', (uint8_t)' ', (uint8_t)'f',
(uint8_t)'o', (uint8_t)'r', (uint8_t)' ', (uint8_t)'p', (uint8_t)'r', (uint8_t)'e', (uint8_t)'v', (uint8_t)'i',
(uint8_t)'o', (uint8_t)'u', (uint8_t)'s', (uint8_t)' ', (uint8_t)'c', (uint8_t)'o', (uint8_t)'m', (uint8_t)'m',
(uint8_t)'a', (uint8_t)'n', (uint8_t)'d', 

    /* stim_state */
    0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0x51u, 0xD2u, 0x50u, 0x3Bu, 0xDDu, 0xBAu, 0x63u, 0x84u, 0x58u, 0x44u, 0x57u, 0x77u, 0x62u, 0x90u, 0x1Eu, 0x5Fu, 

    /* Characteristic User Description */
    (uint8_t)'G', (uint8_t)'e', (uint8_t)'t', (uint8_t)' ', (uint8_t)'s', (uint8_t)'t', (uint8_t)'i', (uint8_t)'m',
(uint8_t)'u', (uint8_t)'l', (uint8_t)'a', (uint8_t)'t', (uint8_t)'o', (uint8_t)'r', (uint8_t)' ', (uint8_t)'s',
(uint8_t)'t', (uint8_t)'a', (uint8_t)'t', (uint8_t)'e', (uint8_t)' ', (uint8_t)'a', (uint8_t)'s', (uint8_t)' ',
(uint8_t)'1', (uint8_t)' ', (uint8_t)'b', (uint8_t)'y', (uint8_t)'t', (uint8_t)'e', (uint8_t)' ', (uint8_t)'a',
(uint8_t)'r', (uint8_t)'r', (uint8_t)'a', (uint8_t)'y', 

    /* tar_curr */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* r_vdac_tar_desc */
    0x00u, 0xFAu, 0x01u, 0xEAu, 0x04u, 0xDAu, 0x3Eu, 0xF4u, 0x90u, 0xE3u, 0x43u, 0x47u, 0x64u, 0xA9u, 0x8Au, 0x2Du, 0xCAu, 

    /* Characteristic User Description */
    (uint8_t)'R', (uint8_t)'e', (uint8_t)'a', (uint8_t)'d', (uint8_t)' ', (uint8_t)'d', (uint8_t)'c', (uint8_t)' ',
(uint8_t)'c', (uint8_t)'u', (uint8_t)'r', (uint8_t)'r', (uint8_t)' ', (uint8_t)'t', (uint8_t)'a', (uint8_t)'r',
(uint8_t)'g', (uint8_t)'e', (uint8_t)'t', 

    /* base_curr */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* r_vdac_base */
    0x00u, 0xC8u, 0x77u, 0x69u, 0xB4u, 0x69u, 0x96u, 0xA2u, 0xB4u, 0xB6u, 0x45u, 0x70u, 0x99u, 0x14u, 0x64u, 0xC4u, 0xA7u, 

    /* Characteristic User Description */
    (uint8_t)'R', (uint8_t)'e', (uint8_t)'a', (uint8_t)'d', (uint8_t)' ', (uint8_t)'d', (uint8_t)'c', (uint8_t)' ',
(uint8_t)'c', (uint8_t)'u', (uint8_t)'r', (uint8_t)'r', (uint8_t)'e', (uint8_t)'n', (uint8_t)'t', (uint8_t)' ',
(uint8_t)'b', (uint8_t)'a', (uint8_t)'s', (uint8_t)'e', 

    /* timing */
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0x1Du, 0x1Au, 0x1Fu, 0x51u, 0xFEu, 0xB7u, 0xC3u, 0xBFu, 0x89u, 0x44u, 0x69u, 0x12u, 0x2Fu, 0x9Eu, 0x24u, 0xD5u, 

    /* Characteristic User Description */
    (uint8_t)'R', (uint8_t)'e', (uint8_t)'a', (uint8_t)'d', (uint8_t)' ', (uint8_t)'d', (uint8_t)'c', (uint8_t)' ',
(uint8_t)'t', (uint8_t)'i', (uint8_t)'m', (uint8_t)'i', (uint8_t)'n', (uint8_t)'g', (uint8_t)' ', (uint8_t)'v',
(uint8_t)'a', (uint8_t)'l', (uint8_t)'u', (uint8_t)'e', (uint8_t)'s', 

    /* pulses */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xA4u, 0xBFu, 0xF1u, 0x3Au, 0x68u, 0xB0u, 0x3Bu, 0xA0u, 0x49u, 0x41u, 0x2Eu, 0x29u, 0x83u, 0x23u, 0xD9u, 0x70u, 

    /* pulses */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

    /* bursts */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

    /* p1_curr */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

    /* p2_curr */
    0x00u, 0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

    /* anodic */
    0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

    /* timing */
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
0x00u, 0x00u, 0x00u, 

    /* Custom Descriptor */
    0x00u, 0xE3u, 0x55u, 0xF3u, 0xCAu, 0xF4u, 0xEDu, 0x21u, 0x9Eu, 0x70u, 0x4Du, 0xFFu, 0xFEu, 0x06u, 0x2Fu, 0x35u, 0xE0u, 

};
#if(CY_BLE_GATT_DB_CCCD_COUNT != 0u)
static uint8_t cy_ble_attValuesCCCD[CY_BLE_GATT_DB_CCCD_COUNT];
#endif /* CY_BLE_GATT_DB_CCCD_COUNT != 0u */

static const uint8_t cy_ble_attUuid128[][16u] = {
    /* nstim */
    { 0x76u, 0x9Cu, 0x65u, 0x5Bu, 0xE5u, 0x77u, 0xB8u, 0x89u, 0xDEu, 0x4Bu, 0x31u, 0x3Eu, 0x92u, 0x7Au, 0xD4u, 0x13u },
    /* command */
    { 0xF3u, 0xDBu, 0x91u, 0x5Du, 0x5Fu, 0xEAu, 0x2Fu, 0x88u, 0x26u, 0x45u, 0x5Au, 0xEFu, 0x38u, 0x3Au, 0x58u, 0xC4u },
    /* err */
    { 0x9Fu, 0x61u, 0xEAu, 0xF7u, 0xE6u, 0xB1u, 0x7Bu, 0x90u, 0xDFu, 0x4Au, 0x86u, 0xF8u, 0xABu, 0x65u, 0x22u, 0xDBu },
    /* stim_state */
    { 0x66u, 0x23u, 0xD3u, 0x55u, 0x36u, 0x8Cu, 0xA3u, 0xA7u, 0x22u, 0x4Au, 0xBEu, 0x7Au, 0x3Cu, 0xCCu, 0x81u, 0x5Bu },
    /* dc_serv */
    { 0x79u, 0x7Cu, 0xC2u, 0x10u, 0xE0u, 0x9Du, 0x48u, 0xBBu, 0xA3u, 0x47u, 0x23u, 0xF9u, 0x87u, 0x0Fu, 0x43u, 0x7Fu },
    /* tar_curr */
    { 0x1Fu, 0x4Cu, 0x56u, 0x23u, 0x2Fu, 0xC4u, 0x39u, 0x80u, 0xC3u, 0x44u, 0xF7u, 0x21u, 0x6Cu, 0xD9u, 0x15u, 0x5Bu },
    /* base_curr */
    { 0x6Eu, 0x8Bu, 0xBCu, 0x5Cu, 0x83u, 0xFBu, 0x39u, 0x97u, 0x4Cu, 0x45u, 0x54u, 0x11u, 0x96u, 0xEBu, 0x7Au, 0x34u },
    /* timing */
    { 0x61u, 0xFDu, 0x1Au, 0x8Fu, 0xF1u, 0x57u, 0x80u, 0x9Du, 0x1Du, 0x47u, 0x35u, 0x60u, 0xD6u, 0xC2u, 0x9Au, 0xB4u },
    /* pulses */
    { 0xBDu, 0xF6u, 0x5Bu, 0x1Fu, 0x9Au, 0xE0u, 0x74u, 0xBBu, 0xA5u, 0x4Du, 0x87u, 0xA0u, 0x8Au, 0x9Au, 0x40u, 0x21u },
    /* ac_serv */
    { 0x28u, 0xA5u, 0x1Eu, 0x34u, 0xF2u, 0xB0u, 0x79u, 0x8Bu, 0xC3u, 0x45u, 0xE7u, 0xEAu, 0x41u, 0x97u, 0x0Cu, 0xBAu },
    /* pulses */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
    /* bursts */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
    /* p1_curr */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
    /* p2_curr */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
    /* anodic */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
    /* timing */
    { 0x06u, 0x04u, 0x4Du, 0xC6u, 0xB5u, 0xD0u, 0x01u, 0x86u, 0x08u, 0x43u, 0xC7u, 0xFDu, 0x7Cu, 0x96u, 0xF9u, 0x28u },
};

static cy_stc_ble_gatts_att_gen_val_len_t cy_ble_attValuesLen[0x37u] = {
    { 0x0005u, (void *)&cy_ble_attValues[0] }, /* Device Name */
    { 0x0002u, (void *)&cy_ble_attValues[5] }, /* Appearance */
    { 0x0008u, (void *)&cy_ble_attValues[7] }, /* Peripheral Preferred Connection Parameters */
    { 0x0001u, (void *)&cy_ble_attValues[15] }, /* Central Address Resolution */
    { 0x0001u, (void *)&cy_ble_attValues[16] }, /* Resolvable Private Address Only */
    { 0x0004u, (void *)&cy_ble_attValues[17] }, /* Service Changed */
    { 0x0002u, (void *)&cy_ble_attValuesCCCD[0] }, /* Client Characteristic Configuration */
    { 0x0010u, (void *)&cy_ble_attUuid128[0] }, /* nstim UUID */
    { 0x0010u, (void *)&cy_ble_attUuid128[1] }, /* command UUID */
    { 0x0005u, (void *)&cy_ble_attValues[21] }, /* command */
    { 0x0009u, (void *)&cy_ble_attValues[26] }, /* Custom Descriptor */
    { 0x0017u, (void *)&cy_ble_attValues[51] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[2] }, /* err UUID */
    { 0x0001u, (void *)&cy_ble_attValues[74] }, /* err */
    { 0x0001u, (void *)&cy_ble_attValues[75] }, /* Custom Descriptor */
    { 0x0023u, (void *)&cy_ble_attValues[92] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[3] }, /* stim_state UUID */
    { 0x0002u, (void *)&cy_ble_attValues[127] }, /* stim_state */
    { 0x0001u, (void *)&cy_ble_attValues[129] }, /* Custom Descriptor */
    { 0x0024u, (void *)&cy_ble_attValues[146] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[4] }, /* dc_serv UUID */
    { 0x0010u, (void *)&cy_ble_attUuid128[5] }, /* tar_curr UUID */
    { 0x0004u, (void *)&cy_ble_attValues[182] }, /* tar_curr */
    { 0x0001u, (void *)&cy_ble_attValues[186] }, /* r_vdac_tar_desc */
    { 0x0013u, (void *)&cy_ble_attValues[203] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[6] }, /* base_curr UUID */
    { 0x0004u, (void *)&cy_ble_attValues[222] }, /* base_curr */
    { 0x0001u, (void *)&cy_ble_attValues[226] }, /* r_vdac_base */
    { 0x0014u, (void *)&cy_ble_attValues[243] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[7] }, /* timing UUID */
    { 0x000Cu, (void *)&cy_ble_attValues[263] }, /* timing */
    { 0x0001u, (void *)&cy_ble_attValues[275] }, /* Custom Descriptor */
    { 0x0015u, (void *)&cy_ble_attValues[292] }, /* Characteristic User Description */
    { 0x0010u, (void *)&cy_ble_attUuid128[8] }, /* pulses UUID */
    { 0x0004u, (void *)&cy_ble_attValues[313] }, /* pulses */
    { 0x0001u, (void *)&cy_ble_attValues[317] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[9] }, /* ac_serv UUID */
    { 0x0010u, (void *)&cy_ble_attUuid128[10] }, /* pulses UUID */
    { 0x0004u, (void *)&cy_ble_attValues[334] }, /* pulses */
    { 0x0001u, (void *)&cy_ble_attValues[338] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[11] }, /* bursts UUID */
    { 0x0004u, (void *)&cy_ble_attValues[355] }, /* bursts */
    { 0x0001u, (void *)&cy_ble_attValues[359] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[12] }, /* p1_curr UUID */
    { 0x0004u, (void *)&cy_ble_attValues[376] }, /* p1_curr */
    { 0x0001u, (void *)&cy_ble_attValues[380] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[13] }, /* p2_curr UUID */
    { 0x0004u, (void *)&cy_ble_attValues[397] }, /* p2_curr */
    { 0x0001u, (void *)&cy_ble_attValues[401] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[14] }, /* anodic UUID */
    { 0x0001u, (void *)&cy_ble_attValues[418] }, /* anodic */
    { 0x0001u, (void *)&cy_ble_attValues[419] }, /* Custom Descriptor */
    { 0x0010u, (void *)&cy_ble_attUuid128[15] }, /* timing UUID */
    { 0x0014u, (void *)&cy_ble_attValues[436] }, /* timing */
    { 0x0001u, (void *)&cy_ble_attValues[456] }, /* Custom Descriptor */
};

static const cy_stc_ble_gatts_db_t cy_ble_gattDB[0x3Fu] = {
    { 0x0001u, 0x2800u /* Primary service                     */, 0x00000001u /*       */, 0x000Bu, {{0x1800u, NULL}}                           },
    { 0x0002u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0003u, {{0x2A00u, NULL}}                           },
    { 0x0003u, 0x2A00u /* Device Name                         */, 0x01020001u /* rd    */, 0x0003u, {{0x0005u, (void *)&cy_ble_attValuesLen[0]}} },
    { 0x0004u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0005u, {{0x2A01u, NULL}}                           },
    { 0x0005u, 0x2A01u /* Appearance                          */, 0x01020001u /* rd    */, 0x0005u, {{0x0002u, (void *)&cy_ble_attValuesLen[1]}} },
    { 0x0006u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0007u, {{0x2A04u, NULL}}                           },
    { 0x0007u, 0x2A04u /* Peripheral Preferred Connection Par */, 0x01020001u /* rd    */, 0x0007u, {{0x0008u, (void *)&cy_ble_attValuesLen[2]}} },
    { 0x0008u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0009u, {{0x2AA6u, NULL}}                           },
    { 0x0009u, 0x2AA6u /* Central Address Resolution          */, 0x01020001u /* rd    */, 0x0009u, {{0x0001u, (void *)&cy_ble_attValuesLen[3]}} },
    { 0x000Au, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x000Bu, {{0x2AC9u, NULL}}                           },
    { 0x000Bu, 0x2AC9u /* Resolvable Private Address Only     */, 0x01020001u /* rd    */, 0x000Bu, {{0x0001u, (void *)&cy_ble_attValuesLen[4]}} },
    { 0x000Cu, 0x2800u /* Primary service                     */, 0x00000001u /*       */, 0x000Fu, {{0x1801u, NULL}}                           },
    { 0x000Du, 0x2803u /* Characteristic                      */, 0x00200001u /* ind   */, 0x000Fu, {{0x2A05u, NULL}}                           },
    { 0x000Eu, 0x2A05u /* Service Changed                     */, 0x01200000u /* ind   */, 0x000Fu, {{0x0004u, (void *)&cy_ble_attValuesLen[5]}} },
    { 0x000Fu, 0x2902u /* Client Characteristic Configuration */, 0x030A0101u /* rd,wr */, 0x000Fu, {{0x0002u, (void *)&cy_ble_attValuesLen[6]}} },
    { 0x0010u, 0x2800u /* Primary service                     */, 0x08000001u /*       */, 0x001Cu, {{0x0010u, (void *)&cy_ble_attValuesLen[7]}} },
    { 0x0011u, 0x2803u /* Characteristic                      */, 0x00080001u /* wr    */, 0x0014u, {{0x0010u, (void *)&cy_ble_attValuesLen[8]}} },
    { 0x0012u, 0x3A38u /* command                             */, 0x09080100u /* wr    */, 0x0014u, {{0x0005u, (void *)&cy_ble_attValuesLen[9]}} },
    { 0x0013u, 0x5996u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0013u, {{0x0009u, (void *)&cy_ble_attValuesLen[10]}} },
    { 0x0014u, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x0014u, {{0x0017u, (void *)&cy_ble_attValuesLen[11]}} },
    { 0x0015u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0018u, {{0x0010u, (void *)&cy_ble_attValuesLen[12]}} },
    { 0x0016u, 0x65ABu /* err                                 */, 0x09020001u /* rd    */, 0x0018u, {{0x0001u, (void *)&cy_ble_attValuesLen[13]}} },
    { 0x0017u, 0x5440u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0017u, {{0x0001u, (void *)&cy_ble_attValuesLen[14]}} },
    { 0x0018u, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x0018u, {{0x0023u, (void *)&cy_ble_attValuesLen[15]}} },
    { 0x0019u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x001Cu, {{0x0010u, (void *)&cy_ble_attValuesLen[16]}} },
    { 0x001Au, 0xCC3Cu /* stim_state                          */, 0x09020001u /* rd    */, 0x001Cu, {{0x0002u, (void *)&cy_ble_attValuesLen[17]}} },
    { 0x001Bu, 0x9062u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x001Bu, {{0x0001u, (void *)&cy_ble_attValuesLen[18]}} },
    { 0x001Cu, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x001Cu, {{0x0024u, (void *)&cy_ble_attValuesLen[19]}} },
    { 0x001Du, 0x2800u /* Primary service                     */, 0x08000001u /*       */, 0x002Cu, {{0x0010u, (void *)&cy_ble_attValuesLen[20]}} },
    { 0x001Eu, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0021u, {{0x0010u, (void *)&cy_ble_attValuesLen[21]}} },
    { 0x001Fu, 0xD96Cu /* tar_curr                            */, 0x09020001u /* rd    */, 0x0021u, {{0x0004u, (void *)&cy_ble_attValuesLen[22]}} },
    { 0x0020u, 0x8AA9u /* r_vdac_tar_desc                     */, 0x09000001u /*       */, 0x0020u, {{0x0001u, (void *)&cy_ble_attValuesLen[23]}} },
    { 0x0021u, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x0021u, {{0x0013u, (void *)&cy_ble_attValuesLen[24]}} },
    { 0x0022u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0025u, {{0x0010u, (void *)&cy_ble_attValuesLen[25]}} },
    { 0x0023u, 0xEB96u /* base_curr                           */, 0x09020001u /* rd    */, 0x0025u, {{0x0004u, (void *)&cy_ble_attValuesLen[26]}} },
    { 0x0024u, 0x6414u /* r_vdac_base                         */, 0x09000001u /*       */, 0x0024u, {{0x0001u, (void *)&cy_ble_attValuesLen[27]}} },
    { 0x0025u, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x0025u, {{0x0014u, (void *)&cy_ble_attValuesLen[28]}} },
    { 0x0026u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0029u, {{0x0010u, (void *)&cy_ble_attValuesLen[29]}} },
    { 0x0027u, 0xC2D6u /* timing                              */, 0x09020001u /* rd    */, 0x0029u, {{0x000Cu, (void *)&cy_ble_attValuesLen[30]}} },
    { 0x0028u, 0x9E2Fu /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0028u, {{0x0001u, (void *)&cy_ble_attValuesLen[31]}} },
    { 0x0029u, 0x2901u /* Characteristic User Description     */, 0x01020001u /* rd    */, 0x0029u, {{0x0015u, (void *)&cy_ble_attValuesLen[32]}} },
    { 0x002Au, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x002Cu, {{0x0010u, (void *)&cy_ble_attValuesLen[33]}} },
    { 0x002Bu, 0x9A8Au /* pulses                              */, 0x09020001u /* rd    */, 0x002Cu, {{0x0004u, (void *)&cy_ble_attValuesLen[34]}} },
    { 0x002Cu, 0x2383u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x002Cu, {{0x0001u, (void *)&cy_ble_attValuesLen[35]}} },
    { 0x002Du, 0x2800u /* Primary service                     */, 0x08000001u /*       */, 0x003Fu, {{0x0010u, (void *)&cy_ble_attValuesLen[36]}} },
    { 0x002Eu, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0030u, {{0x0010u, (void *)&cy_ble_attValuesLen[37]}} },
    { 0x002Fu, 0x967Cu /* pulses                              */, 0x09020001u /* rd    */, 0x0030u, {{0x0004u, (void *)&cy_ble_attValuesLen[38]}} },
    { 0x0030u, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0030u, {{0x0001u, (void *)&cy_ble_attValuesLen[39]}} },
    { 0x0031u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0033u, {{0x0010u, (void *)&cy_ble_attValuesLen[40]}} },
    { 0x0032u, 0x967Cu /* bursts                              */, 0x09020001u /* rd    */, 0x0033u, {{0x0004u, (void *)&cy_ble_attValuesLen[41]}} },
    { 0x0033u, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0033u, {{0x0001u, (void *)&cy_ble_attValuesLen[42]}} },
    { 0x0034u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0036u, {{0x0010u, (void *)&cy_ble_attValuesLen[43]}} },
    { 0x0035u, 0x967Cu /* p1_curr                             */, 0x09020001u /* rd    */, 0x0036u, {{0x0004u, (void *)&cy_ble_attValuesLen[44]}} },
    { 0x0036u, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0036u, {{0x0001u, (void *)&cy_ble_attValuesLen[45]}} },
    { 0x0037u, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x0039u, {{0x0010u, (void *)&cy_ble_attValuesLen[46]}} },
    { 0x0038u, 0x967Cu /* p2_curr                             */, 0x09020001u /* rd    */, 0x0039u, {{0x0004u, (void *)&cy_ble_attValuesLen[47]}} },
    { 0x0039u, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x0039u, {{0x0001u, (void *)&cy_ble_attValuesLen[48]}} },
    { 0x003Au, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x003Cu, {{0x0010u, (void *)&cy_ble_attValuesLen[49]}} },
    { 0x003Bu, 0x967Cu /* anodic                              */, 0x09020001u /* rd    */, 0x003Cu, {{0x0001u, (void *)&cy_ble_attValuesLen[50]}} },
    { 0x003Cu, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x003Cu, {{0x0001u, (void *)&cy_ble_attValuesLen[51]}} },
    { 0x003Du, 0x2803u /* Characteristic                      */, 0x00020001u /* rd    */, 0x003Fu, {{0x0010u, (void *)&cy_ble_attValuesLen[52]}} },
    { 0x003Eu, 0x967Cu /* timing                              */, 0x09020001u /* rd    */, 0x003Fu, {{0x0014u, (void *)&cy_ble_attValuesLen[53]}} },
    { 0x003Fu, 0x2F06u /* Custom Descriptor                   */, 0x09000001u /*       */, 0x003Fu, {{0x0001u, (void *)&cy_ble_attValuesLen[54]}} },
};

#endif /* (CY_BLE_GATT_ROLE_SERVER) */

/* Default device security */
#if (CY_BLE_MODE_PROFILE) 
    cy_stc_ble_gap_auth_info_t cy_ble_authInfo[0x01u] = {

    /* Security configuration 0 */
    {
        .security = (CY_BLE_GAP_SEC_MODE_1 | CY_BLE_GAP_SEC_LEVEL_1 ), 
        .bonding = CY_BLE_GAP_BONDING, 
        .ekeySize = 0x10u, 
        .authErr = CY_BLE_GAP_AUTH_ERROR_NONE, 
        .pairingProperties = 0x01u, 
    },
};

#endif /* CY_BLE_MODE_PROFILE */

/** Initialize BLE configuration parameters structure */
static const cy_stc_ble_params_t cy_ble_params =
{
        .txPowerLevelAdv                    = CY_BLE_LL_PWR_LVL_0_DBM,
        .txPowerLevelConn                   = CY_BLE_LL_PWR_LVL_0_DBM,

        .securityIoCapability               = CY_BLE_GAP_IOCAP_DISPLAY_ONLY,
        .securityPairingMethod              = 0x00u,
    
        .siliconDeviceAddressEnabled        = 0x01u,
    
        .gattDbIndexCount                   = 0x003Fu,
};

cy_stc_ble_gap_bd_addr_t cy_ble_deviceAddress = {{0x00u, 0x00u, 0x00u, 0x50u, 0xA0u, 0x00u}, 0x00u };

/**
* \addtogroup group_globals
* @{
*/
#endif /* CY_BLE_MODE_PROFILE */

/** The configuration structure for BLE */
cy_stc_ble_config_t cy_ble_config =
{
#if (CY_BLE_MODE_PROFILE)
    /* Initialize the GAPP structures */
    /* Initialize the cy_stc_ble_gapp_disc_mode_info_t cy_ble_discoveryModeInfo structure */
    #if (CY_BLE_GAP_ROLE_PERIPHERAL || CY_BLE_GAP_ROLE_BROADCASTER)
        .discoveryModeInfo = cy_ble_discoveryModeInfo,
        .gappAdvParams = cy_ble_gappAdvConfig,
    #else
        .discoveryModeInfo = NULL,
    #endif /* CY_BLE_GAP_ROLE_PERIPHERAL || CY_BLE_GAP_ROLE_BROADCASTER */

    /* Initialize the cy_stc_ble_gapc_disc_info_t  cy_ble_discoveryInfo  structure */
    #if (CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_OBSERVER)
        .discoveryInfo = cy_ble_discoveryInfo,
        .gapcScanParams = cy_ble_gapcScanConfig,
    #else
        .discoveryInfo = NULL,
    #endif /* CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_OBSERVER */

    /* Initialize the GATT structures */
    #if ((CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_PERIPHERAL) && (CY_BLE_BONDING_REQUIREMENT == CY_BLE_BONDING_YES))
        .flashStorage = &cy_ble_flashStorage,
    #else
        .flashStorage = NULL,
    #endif /* CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_PERIPHERAL) && (CY_BLE_BONDING_REQUIREMENT == CY_BLE_BONDING_YES */

    #if (CY_BLE_GATT_ROLE_SERVER)
        .gatts            = &cy_ble_gatts,
        .gaps             = &cy_ble_gaps,
        .attValuesCCCD    = cy_ble_attValuesCCCD,
        .gattDB           = cy_ble_gattDB,
    #else
        .gatts            = NULL,
        .attValuesCCCD    = NULL,
        .gattDB           = NULL,
    #endif /* CY_BLE_GATT_ROLE_SERVER */

    #if (CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_PERIPHERAL)
        /* Initialize the device security structure */
        .authInfo = cy_ble_authInfo,
    #else
        .authInfo = NULL,
    #endif /* (CY_BLE_GAP_ROLE_CENTRAL || CY_BLE_GAP_ROLE_PERIPHERAL */

    /* Initialize the BLE configuration parameters structure */
    .params   = &cy_ble_params,

    /* An application layer event callback function to receive service events from the BLE Component. */
    .callbackFunc   = NULL,
    
    .deviceAddress  = &cy_ble_deviceAddress,
#endif /* CY_BLE_MODE_PROFILE */

#if (CY_BLE_CONFIG_STACK_CONTR_CORE)
    /* The BLESS interrupt configuration */
    .blessIsrConfig = &BLE_1_bless_isr_cfg,
#endif /* CY_BLE_CONFIG_STACK_CONTR_CORE */
};

#endif /* CY_BLE_HOST_CONTR_CORE */ 

/** @} group_globals */

/* [] END OF FILE */
