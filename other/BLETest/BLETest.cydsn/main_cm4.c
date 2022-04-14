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

SemaphoreHandle_t bleSemaphore;

void genericEventHandler(uint32_t event, void *eventParameter)
{
    (void)eventParameter;
    switch (event)
    {
        case CY_BLE_EVT_STACK_ON:
            printf("Stack started\r\n");
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;
        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("Connected\r\n");
            UART_1_PutString("Something connected\n");
            Cy_GPIO_Write(green_PORT, green_NUM, LED_ON);
        break;
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("Disconnected\r\n");
            Cy_GPIO_Write(green_PORT, green_NUM, LED_OFF);
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;
        default:
        break;
    }
}

void iasEventHandler(uint32_t eventCode, void *eventParam)
{
    (void)eventParam;
    uint8_t alertLevel;
    if (eventCode == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL, sizeof(alertLevel), &alertLevel);
        switch(alertLevel)
        {
            case CY_BLE_NO_ALERT:
                printf("No alert\r\n");
                Cy_GPIO_Write(red_PORT, red_NUM, LED_OFF);
                Cy_GPIO_Write(green_PORT, green_NUM, LED_OFF);
            break;
            case CY_BLE_MILD_ALERT:
                printf("MILD alert\r\n");
                Cy_GPIO_Write(red_PORT, red_NUM, LED_ON);
                Cy_GPIO_Write(green_PORT, green_NUM, LED_OFF);
            break;
            case CY_BLE_HIGH_ALERT:
                printf("HIGH alert\r\n");
                Cy_GPIO_Write(red_PORT, red_NUM, LED_ON);
                Cy_GPIO_Write(green_PORT, green_NUM, LED_ON);
            break;
        }
    }
}

void bleInterruptNotify()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void bleTask(void *arg)
{
    (void)arg;
    printf("Ble task started\r\n");
    
    bleSemaphore = xSemaphoreCreateCounting(UINT_MAX, 0);
    Cy_BLE_Start(genericEventHandler);
    
    while (Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
    Cy_BLE_IAS_RegisterAttrCallback(iasEventHandler);
    for (;;)
    {
        xSemaphoreTake(bleSemaphore, portMAX_DELAY);
        Cy_BLE_ProcessEvents();
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("Sys start\r\n");
    xTaskCreate(bleTask, "bleTask", 8*1024, 0, 1, 0);
    UART_1_PutString("beginning\n");
    
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    vTaskStartScheduler();
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
