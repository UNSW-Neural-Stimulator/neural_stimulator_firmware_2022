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
static const cy_stc_ble_customs_t cy_ble_customs[0x01u] = {

    /* NSTIM service */
    {
        0x0010u, /* Handle of the NSTIM service */ 
        {

            /* ReadFirst characteristic */
            {
                0x0012u, /* Handle of the ReadFirst characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0013u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* WriteVal characteristic */
            {
                0x0015u, /* Handle of the WriteVal characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0016u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* ReadSecond characteristic */
            {
                0x0018u, /* Handle of the ReadSecond characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x0019u, /* Handle of the Custom Descriptor descriptor */ 
                    CY_BLE_GATT_INVALID_ATTR_HANDLE_VALUE, 
                }, 
            },

            /* NotifOne characteristic */
            {
                0x001Bu, /* Handle of the NotifOne characteristic */ 
                
                /* Array of Descriptors handles */
                {
                    0x001Cu, /* Handle of the Custom Descriptor descriptor */ 
                    0x001Du, /* Handle of the Client Characteristic Configuration descriptor */ 
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
