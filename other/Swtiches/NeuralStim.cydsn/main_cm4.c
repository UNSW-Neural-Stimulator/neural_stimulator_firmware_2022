/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <limits.h>

#define LED_ON 0
#define LED_OFF 1

SemaphoreHandle_t bleSempahore;

void genericEventHandler(uint32_t event, void *eventParameter)
{
    (void)eventParameter;
    switch (event)
    {
        case CY_BLE_EVT_STACK_ON:
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
            
        case CY_BLE_EVT_GATT_CONNECT_IND:
            Cy_GPIO_Write(GREEN_PORT, GREEN_NUM, LED_ON);
            Cy_GPIO_Write(RED_PORT, RED_NUM, LED_OFF);
            break;
            
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
            Cy_GPIO_Write(RED_PORT, RED_NUM, LED_ON);
            Cy_GPIO_Write(GREEN_PORT, GREEN_NUM, LED_OFF);
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        default:
            break;
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
