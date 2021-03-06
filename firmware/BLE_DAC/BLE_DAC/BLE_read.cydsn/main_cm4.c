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
uint8_t val = 0;

uint8_t vals[2] = {0};


// BLE event handler
void genericEventHandler(uint32_t event, void *eventParameter) {
    cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParameter;
    printf("\nBLE Event handler, event 0x%x\n", event);
    // Take an action based on current event
    switch (event) {
        case CY_BLE_EVT_STACK_ON:
        
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("disconnected\r\n");
            Cy_GPIO_Write(P6_3_PORT, P6_3_PIN, 0);                           // RED is ON
            Cy_GPIO_Write(P7_1_PORT, P7_1_PIN, 1);                           // GREEN is OFF
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("connected\r\n");
            Cy_GPIO_Write(P6_3_PORT, P6_3_PIN, 1);                           // RED is OFF
            Cy_GPIO_Write(P7_1_PORT, P7_1_PIN, 0);                           // GREEN is ON
            
            break;
            
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            printf("Write req: ");
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) eventParameter;
            printf("attId: 0x%x\n", writeReqParameter->connHandle.attId);
            printf("attrHandle: 0x%x\n", writeReqParameter->handleValPair.attrHandle);
            
            if (writeReqParameter->handleValPair.attrHandle == CY_BLE_NSTIM_NOTIFONE_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE) {
                printf("Notify request\n");   
                
                cy_stc_ble_gatts_handle_value_ntf_t ntf_param;
                ntf_param.connHandle = writeReqParameter->connHandle;
                cy_stc_ble_gatt_handle_value_pair_t val_pair;
                val++;
                val_pair.value.val = &val;
                val_pair.value.len = 1;
                val_pair.value.actualLen = 1;
                val_pair.attrHandle = CY_BLE_NSTIM_NOTIFONE_CHAR_HANDLE;
                ntf_param.handleValPair = val_pair;
                CyDelay(1500);
                Cy_BLE_GATTS_Notification(&ntf_param);
                
                break;
            }
            
            uint8_t command[5];
            for (int i = 0; i < writeReqParameter->handleValPair.value.len; i++) {
                command[i] = writeReqParameter->handleValPair.value.val[i];
            }
            
            if (command[0] < 3 && command[0] > 0) {
                printf("writing %d to %d\n", command[1], command[0]);
                vals[command[0] - 1] = command[1];   
            } else {
                printf("Command param invalid\n");   
            }
           
            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            printf("Read req: ");
            cy_stc_ble_gatts_char_val_read_req_t *readReqParam = (cy_stc_ble_gatts_char_val_read_req_t *) eventParameter;
            
            switch (readReqParam->attrHandle)
            {
                case (CY_BLE_NSTIM_READFIRST_CHAR_HANDLE):
                    printf("reading ReadFirst\n");
                    CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &vals[0], 4);
                    break;
                case (CY_BLE_NSTIM_READSECOND_CHAR_HANDLE):
                    printf("reading ReadSecond\n");
                    CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &vals[1], 4);
                    break;
                default:
                    printf("Broken\n");
            }
            break;
            
        default:
            printf("BLE request not found\n");
            
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
