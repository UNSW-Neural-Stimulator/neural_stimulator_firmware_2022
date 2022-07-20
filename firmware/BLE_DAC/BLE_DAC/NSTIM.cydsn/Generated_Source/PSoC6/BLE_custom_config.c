/***************************************************************************//**
* \file CY_BLE_custom_config.c
* \version 2.20
* 
* \brief
*  This file contains the source code of initialization of the config structure for
*  the Custom Service.
*
********************************************************************************
* \copyright
* Copyright 2017-2019, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ble/cy_ble_custom.h"

#if (CY_BLE_MODE_PROFILE && defined(CY_BLE_CUSTOM))
#ifdef CY_BLE_CUSTOM_SERVER
/* If any Custom Service with custom characteristics is defined in the
* customizer's GUI, their handles will be present in this array.
*/
/* This array contains attribute handles for the defined Custom Services and their characteristics and descriptors.
   The array index definitions are located in the BLE_custom.h file. */
static const cy_stc_ble_customs_t cy_ble_customs[0x03u] = {

    /* nstim service */
    {
        0x0010u, /* Handle of the nstim service */ 
        {

            /* command characteristic */
            {
                0x0012u, /* Handle of the command characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0013u, /* Handle of the Custom Descriptor descriptor */ 
                    0x0014u, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },

            /* err characteristic */
            {
                0x0016u, /* Handle of the err characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0017u, /* Handle of the Custom Descriptor descriptor */ 
                    0x0018u, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },

            /* stim_state characteristic */
            {
                0x001Au, /* Handle of the stim_state characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x001Bu, /* Handle of the Custom Descriptor descriptor */ 
                    0x001Cu, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },
            {
                CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                
                /* Array of Descriptors handles */
                {
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
            {
                CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                
                /* Array of Descriptors handles */
                {
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
            {
                CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                
                /* Array of Descriptors handles */
                {
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
        }, 
    },

    /* dc_serv service */
    {
        0x001Du, /* Handle of the dc_serv service */ 
        {

            /* tar_curr characteristic */
            {
                0x001Fu, /* Handle of the tar_curr characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0020u, /* Handle of the r_vdac_tar_desc descriptor */ 
                    0x0021u, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },

            /* base_curr characteristic */
            {
                0x0023u, /* Handle of the base_curr characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0024u, /* Handle of the r_vdac_base descriptor */ 
                    0x0025u, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },

            /* timing characteristic */
            {
                0x0027u, /* Handle of the timing characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0028u, /* Handle of the Custom Descriptor descriptor */ 
                    0x0029u, /* Handle of the Characteristic User Description descriptor */ 
                }, 
            },

            /* pulses characteristic */
            {
                0x002Bu, /* Handle of the pulses characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x002Cu, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
            {
                CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                
                /* Array of Descriptors handles */
                {
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
            {
                CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                
                /* Array of Descriptors handles */
                {
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
        }, 
    },

    /* ac_serv service */
    {
        0x002Du, /* Handle of the ac_serv service */ 
        {

            /* pulses characteristic */
            {
                0x002Fu, /* Handle of the pulses characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0030u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* bursts characteristic */
            {
                0x0032u, /* Handle of the bursts characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0033u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* p1_curr characteristic */
            {
                0x0035u, /* Handle of the p1_curr characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0036u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* p2_curr characteristic */
            {
                0x0038u, /* Handle of the p2_curr characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0039u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* anodic characteristic */
            {
                0x003Bu, /* Handle of the anodic characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x003Cu, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* timing characteristic */
            {
                0x003Eu, /* Handle of the timing characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x003Fu, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },
        }, 
    },
};


#endif /* (CY_BLE_CUSTOM_SERVER) */

#ifdef CY_BLE_CUSTOM_CLIENT


#endif /* (CY_BLE_CUSTOM_CLIENT) */

/**
* \addtogroup group_globals
* @{
*/

/** The configuration structure for the Custom Services. */
cy_stc_ble_custom_config_t cy_ble_customConfig =
{
    /* Custom Services GATT DB handles structure */
    #ifdef CY_BLE_CUSTOM_SERVER
    .customs      = cy_ble_customs,
    #endif /* (CY_BLE_CUSTOM_SERVER) */

    /* Custom Service handle */
    #ifdef CY_BLE_CUSTOM_CLIENT
    .customc  = cy_ble_customCServ,
    #endif /* (CY_BLE_CUSTOM_CLIENT) */
};

/** @} group_globals */
#endif /* (CY_BLE_MODE_PROFILE && defined(CY_BLE_CUSTOM)) */

/* [] END OF FILE */
