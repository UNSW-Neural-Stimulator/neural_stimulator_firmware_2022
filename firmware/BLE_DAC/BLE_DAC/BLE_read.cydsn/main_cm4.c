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

SemaphoreHandle_t bleSemaphore;

uint32_t val1 = 1;
uint32_t val2 = 2;

// BLE event handler
void genericEventHandler(uint32_t event, void *eventParameter) {
    cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParameter;
    printf("BLE Event handler, event 0x%x\n", event);
    // Take an action based on current event
    switch (event) {
        case CY_BLE_EVT_STACK_ON:
        
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("disconnected\r\n");
            Cy_GPIO_Write(P6_3_PORT, P6_3_PIN, 1);                           // RED is ON
            Cy_GPIO_Write(P7_1_PORT, P7_1_PIN, 0);                           // GREEN is OFF
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("connected\r\n");
            Cy_GPIO_Write(P6_3_PORT, P6_3_PIN, 0);                           // RED is OFF
            Cy_GPIO_Write(P7_1_PORT, P7_1_PIN, 1);                           // GREEN is ON
            break;
            
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            printf("Write req\n");
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) eventParameter;
            uint8_t command[5];
            for (int i = 0; i < writeReqParameter->handleValPair.value.len; i++) {
                command[i] = writeReqParameter->handleValPair.value.val[i];
            }
            
            if (command[0] == 1) val1 = command[1];
            else if (command[0] == 2) val2 = command[1];
            else printf("Command param invalid\n");
            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            printf("Read req");
            cy_stc_ble_gatts_char_val_read_req_t *readReqParam = (cy_stc_ble_gatts_char_val_read_req_t *) eventParameter;
            
            switch (readReqParam->attrHandle)
            {
                case (CY_BLE_NSTIM_READFIRST_CHAR_HANDLE):
                    CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &val1, 4);
                    break;
                case (CY_BLE_NSTIM_READSECOND_CHAR_HANDLE):
                    CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &val2, 4);
                    break;
                default:
                    printf("Broken\n");
            }
        default:
            printf("Not found\n");
            break;
    }
}

void bleInterruptNotify() {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void bleTask(void *arg) {
    (void) arg;
    printf("BLE Task Started\r\n");
    
    bleSemaphore = xSemaphoreCreateCounting(UINT_MAX, 0);
    
    Cy_BLE_Start(genericEventHandler);
    
    while (Cy_BLE_GetState() != CY_BLE_STATE_ON) {
        Cy_BLE_ProcessEvents();
    }
    
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);
    
    for(;;) {
        xSemaphoreTake(bleSemaphore, portMAX_DELAY);
        Cy_BLE_ProcessEvents();
        vTaskDelay(100);
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();
    printf("Process started\n");
    
    xTaskCreate(bleTask, "bleTask", 1024, 0, 1, 0);
    
    vTaskStartScheduler();
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
