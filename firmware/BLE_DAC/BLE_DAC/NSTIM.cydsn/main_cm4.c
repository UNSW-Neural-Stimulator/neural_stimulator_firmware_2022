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

#define SW_ISO P5_2_PORT
#define SW_LOAD  P5_3_PORT
#define SW_SHORT P5_4_PORT
#define SW_EVM P5_5_PORT

#define LED_ON 0
#define LED_OFF 1

/* Converts microseconds to phase timing. We only care about phase timings in code! */
#define US_TO_PT(n) (n / 2)
/* Converts phase timing to microseconds */
#define PT_TO_US(n) (n * 2)
/* Convert a current (float) to a vdac value (uint) */
#define CURR_TO_VDAC(n)     ((uint32_t)((((float)n)+3.48600000e+00)/1.70315347e-03))
/* Convert a vdac value (uint) to current (float) */
#define VDAC_TO_CURR(n)     ((float)(((float)n)*1.70315347e-03)-3.48600000e+00)
/* Our baseline reference for 0V stim */
#define V0 2056u

/* Used by BLE task to wait for next BLE request */
SemaphoreHandle_t bleSemaphore;

static TaskHandle_t bleTaskHandle = NULL, adcdacTaskHandle = NULL;

/* Phase of pulse */
uint32_t phase = 0u;
/* Generic phase timing counter */
uint32_t counter = 0u;
/* Current code is what the voltage is going to be set as */
uint32_t vdac_curr = 0u;
/* Generic error status register, accessible through a read directly after a write */
uint32_t err = 0;

/* 0 is Burst stim, 1 is DC stim */
int stim_type = 0;

/* VDAC values for each phase in burst mode */
uint32_t burst_vdac_values[] = {V0, 4095u, V0, 0u};
/* Phase timings for burst mode, interstim comes first */
uint32_t burst_phase_timings[] = {100u, 50u, 50u, 50u};
/* Max value for DC mode, what we want to ramp up to */
uint32_t dc_vdac_target = 4095u;
/* Starting value for DC mode, what we start the ramp from */
uint32_t dc_vdac_base = V0;
/* Phase timings for DC mode */
uint32_t dc_phase_timings[] = {100u, 200u, 100u, 200u};

int command_start(uint32_t param);
int command_stop(uint32_t param);
int command_stim_type(uint32_t param);

static int (*command_handlers[])(uint32_t param) = {
    NULL,
    command_start,
    command_stop,
    command_stim_type,
};

void burst_handler();
void dc_handler();

//float vdac_to_curr(int vdac) {
//    return ((float)(((float)vdac)*1.70315347e-03)-3.48600000e+00);

//}

void command_handler(uint8_t command, uint32_t params) {
    if (command <= 0 || command > 11) {
        printf("Invalid command '%d'\n", command);   
        err = 1;
    }
    
    err = command_handlers[command](params);
    if (command == 3) {
        printf("Changed to %d stim type\n", params);   
    }
}

// BLE event handler
void genericEventHandler(uint32_t event, void *eventParameter) {
    cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParameter;
    printf("\nBLE Event handler, event 0x%x\n", event);
    // Take an action based on current event
    switch (event) {
        case CY_BLE_EVT_STACK_ON:
        
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("disconnected\r\n");
            Cy_GPIO_Write(RED_PORT, RED_NUM, 0);                           // RED is ON
            Cy_GPIO_Write(GREEN_PORT, GREEN_NUM, 1);                           // GREEN is OFF
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("connected\r\n");
            Cy_GPIO_Write(RED_PORT, RED_NUM, 1);                           // RED is OFF
            Cy_GPIO_Write(GREEN_PORT, GREEN_NUM, 0);                           // GREEN is ON
            break;
            
        case CY_BLE_EVT_GATTS_WRITE_REQ:
            printf("Write req\n");
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) eventParameter;
            uint8_t req_param[5] = {0};
            for (int i = 0; i < writeReqParameter->handleValPair.value.len; i++) {
                req_param[i] = writeReqParameter->handleValPair.value.val[i];
            }
            
            uint8_t command = req_param[0];
            uint32_t param =    (req_param[1] << 24) + 
                                (req_param[2] << 16) + 
                                (req_param[3] << 8) + 
                                (req_param[4]);
            
            printf("Executed command %x with param %x\n", command, param);
            command_handler(command, param);
           
            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            printf("Read req: ");
            cy_stc_ble_gatts_char_val_read_req_t *readReqParam = (cy_stc_ble_gatts_char_val_read_req_t *) eventParameter;
            
            switch (readReqParam->attrHandle)
            {
                case (CY_BLE_NSTIM_READFIRST_CHAR_HANDLE):
                    printf("reading ReadFirst\n");
                    //CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &vals[0], 4);
                    break;
                case (CY_BLE_NSTIM_READSECOND_CHAR_HANDLE):
                    printf("reading ReadSecond\n");
                    //CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &vals[1], 4);
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

void userIsr(void)
{
    uint8_t intrStatus;

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_GetInterruptStatus();
    if (intrStatus)
    {
        /* Clear the interrupt. */
        VDAC_ClearInterrupt();
        /* Run burst handler or dc handler depending on stim_type */
        stim_type ? burst_handler() : dc_handler();
    }
}

void dc_handler() {
    counter++;
    if (counter >= dc_phase_timings[phase]) {
        phase = (phase + 1) % 3;
    }
    
    // TODO switches for dc
    
    if (phase == 0) {
        /* Ramping up */
        vdac_curr = ((dc_vdac_target - dc_vdac_base) / dc_phase_timings[phase]) * counter;
    } else if (phase == 1) {
        /* Holding steady */
        vdac_curr = dc_vdac_target;
    } else if (phase == 2) {
        /* Ramping down */
        vdac_curr = ((dc_vdac_base - dc_vdac_target) / dc_phase_timings[phase]) * counter;
    }
    VDAC_SetValueBuffered(vdac_curr);
}

void burst_handler() {
    counter++;
    if (counter >= burst_phase_timings[phase]) {
        phase++;
        phase %= 4;
        vdac_curr = burst_vdac_values[phase];
        counter = 0u;
        Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, (phase & 1) ? 1 : 0);
        Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, (phase == 0) ? 1 : 0);
        Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, (phase & 1) ? 0 : 1);
        Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, (phase == 0) ? 0 : 1);
    }

    VDAC_SetValueBuffered(vdac_curr);
}

int command_start(uint32_t param) {
    printf("Inside start command with param %u\n", param);   
    VDAC_Start();
    return 0;
}

int command_stop(uint32_t param) {
    printf("Inside command stop with param %u\n", param);   
    VDAC_Stop();
    return 0;
}

int command_stim_type(uint32_t param) {
    if (param == 0 || param == 1) {
        stim_type = param;
        return 0;
    }
    return 1;
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();
    printf("Process started\n");
    
    (void)Cy_SysInt_Init(&SysInt_VDAC_cfg, userIsr);
    
    NVIC_EnableIRQ(SysInt_VDAC_cfg.intrSrc);
    //VDAC_Start();
    
    printf("V0 vdac to %f\n", VDAC_TO_CURR(V0));
    printf("0 amps to %u\n", CURR_TO_VDAC(0));
    printf("1 amps to %u\n", CURR_TO_VDAC(1));
    printf("3 amps to %u\n", CURR_TO_VDAC(3));
    
    xTaskCreate(bleTask, "bleTask", 1024, 0, 1, 0);
    
    vTaskStartScheduler();
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
