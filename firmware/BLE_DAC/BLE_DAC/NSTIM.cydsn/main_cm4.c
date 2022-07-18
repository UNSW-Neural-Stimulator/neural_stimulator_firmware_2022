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
#include "FreeRTOS.h"
#include "project.h"
#include "semphr.h"
#include "task.h"
#include <limits.h>
#include <stdio.h>
#include <math.h>

#define SW_ISO P5_2_PORT
#define SW_LOAD P5_3_PORT
#define SW_SHORT P5_4_PORT
#define SW_EVM P5_5_PORT

#define LED_ON 0
#define LED_OFF 1

/* Converts microseconds to phase timing. We only care about phase timings in
 * code! */
#define US_TO_PT(n) (n / 2)
/* Converts phase timing to microseconds */
#define PT_TO_US(n) (n * 2)
/* Convert a current (float) to a vdac value (uint) */
#define CURR_TO_VDAC(n)                                                        \
  ((uint32_t)((((float)n) + 3.48600000e+00) / 1.70315347e-03))
/* Convert a vdac value (uint) to current (float) */
#define VDAC_TO_CURR(n) ((float)(((float)n) * 1.70315347e-03) - 3.48600000e+00)
/* Our baseline reference for 0V stim */
#define V0 2059u
/* Our max value for VDAC */
#define VDAC_MAX 4096u // Leaving it as this for now
/* Invert a dac value around the baseline */
#define INVERT_VDAC(n) (V0 * 2 - n)
/* Return the abs value of a number */
#define ABS(n) (n > 0) ? n : -n

/* Struct for converting uint32 bits to float and vice versa */
typedef union overlay_t {
  uint32_t u;
  float f;
} overlay_t;

typedef union uint_int_overlay_t {
    uint32_t u;
    int i;
} uint_int_overlay_t;

/* Used by BLE task to wait for next BLE request */
SemaphoreHandle_t bleSemaphore;

/* Phase of pulse */
uint32_t phase = 0u;
/* Generic phase timing counter */
uint32_t counter = 0u;
/* Current code is what the voltage is going to be set as */
uint32_t vdac_curr = 0u;
/* Generic error status register, accessible through a read directly after a
 * write */
uint32_t err = 0;

/* Pulses per burst */
uint32_t ac_pulse_num = 0;
/* Pulse completed */
uint32_t ac_pulse_done = 0;
/* Bursts target */
uint32_t ac_burst_num = 0;
/* Bursts completed */
uint32_t ac_burst_done = 0;

/* Number of dc bursts target */
uint32_t dc_pulse_num = 0;
/* Number of dc_pulses done */
uint32_t dc_pulse_done = 0;

/* stim_state[0] = stim_on (1 on 0 off)
 * stim_state[1] = stim_type (0 ac, 1 dc) */
uint8_t stim_state[2] = {0};

/* Indicates if ac mode is anodic or cathodic right now */
uint8_t anodic = 0;

/* VDAC values for each phase in ac mode */
uint16_t ac_vdac_values[] = {V0, 4095u, V0, 0u, V0};
/* Phase timings for ac mode, interstim, p1, interphase, p2, interburst */
uint32_t ac_phase_timings[] = {400u, 100u, 100u, 100u, 2000u};
/* Max value for DC mode, what we want to ramp up to */
uint32_t dc_vdac_target = 4095u;
/* Starting value for DC mode, what we start the ramp from */
uint32_t dc_vdac_base = V0;
/* Phase timings for DC mode, 0 is interstim, 1 ramp up, 2 is high, 3 is ramp down */
uint32_t dc_phase_timings[] = {100u, 100u, 200u, 100u};
/* Keeps track of our dc_slope so we don't need to keep calculating it */
double dc_slope;

int command_start(uint32_t param);
int command_stop(uint32_t param);
int command_stim_type(uint32_t param);
int command_anodic_cathodic(uint32_t param);
int command_ac_phase_one(uint32_t param);
int command_ac_phase_two(uint32_t param);
int command_ac_phase_gap(uint32_t param);
int command_ac_stim_gap(uint32_t param);
int command_ac_burst_gap(uint32_t param);
int command_ac_pulse_num(uint32_t param);
int command_ac_burst_num(uint32_t param);
int command_ac_phase_one_curr(uint32_t param);
int command_ac_phase_two_curr(uint32_t param);
int command_dc_ramp_time(uint32_t param);
int command_dc_hold_time(uint32_t param);
int command_dc_curr_target(uint32_t param);

static int (*command_handlers[])(uint32_t param) = {
    NULL,                           // 0x00
    command_start,                  // 0x01
    command_stop,                   // 0x02
    command_stim_type,              // 0x03
    command_anodic_cathodic,        // 0x04
    command_ac_phase_one,           // 0x05
    command_ac_phase_two,           // 0x06
    command_ac_phase_gap,           // 0x07
    command_ac_stim_gap,            // 0x08
    command_ac_burst_gap,           // 0x09
    command_ac_pulse_num,           // 0x0A
    command_ac_burst_num,           // 0x0B
    command_ac_phase_one_curr,      // 0x0C
    command_ac_phase_two_curr,      // 0x0D
    command_dc_ramp_time,           // 0x0E
    command_dc_hold_time,           // 0x0F
    command_dc_curr_target,         // 0x10
};

/* Helper functions */
void burst_handler();
void dc_handler();
void set_dc_slope();
int compliance_check(uint16_t p1_time, uint32_t p1_dac, uint16_t p2_time,
                     uint32_t p2_dac);
void read_req_handler(cy_stc_ble_gatts_char_val_read_req_t *readReqParam);
float uint32_to_float(uint32_t u);
uint32_t float_to_uint32(float f);

void command_handler(uint8_t command, uint32_t params) {
    if (command < 1 || command > 16) {
        printf("Invalid command '%d'\n", command);
        err = 1;
    }
    printf("Command 0x%x with param 0x%x\n", command, params);
    err = command_handlers[command](params);
    if (err) printf("Command 0x%x failed with status %d\n", command, err);
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
            Cy_GPIO_Write(RED_PORT, RED_NUM, 0);     // RED is ON
            Cy_GPIO_Write(BLUE_PORT, BLUE_NUM, 1); // GREEN is OFF
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
               CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;
        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("connected\r\n");
            Cy_GPIO_Write(RED_PORT, RED_NUM, 1);     // RED is OFF
            Cy_GPIO_Write(BLUE_PORT, BLUE_NUM, 0); // GREEN is ON
            break;

        case CY_BLE_EVT_GATTS_WRITE_REQ:
            printf("Write req\n");
            writeReqParameter =
            (cy_stc_ble_gatts_write_cmd_req_param_t *)eventParameter;
            uint8_t req_param[5] = {0};
            for (int i = 0; i < writeReqParameter->handleValPair.value.len; i++) {
                req_param[i] = writeReqParameter->handleValPair.value.val[i];
            }

            uint8_t command = req_param[0];
            uint32_t param = req_param[1] + (req_param[2] << 8) +
                            (req_param[3] << 16) + (req_param[4] << 24);
            printf("Param %u\n", param);

  
            command_handler(command, param);

            Cy_BLE_GATTS_WriteRsp(writeReqParameter->connHandle);
            break;
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
            printf("Read req: ");
            cy_stc_ble_gatts_char_val_read_req_t *readReqParam =
            (cy_stc_ble_gatts_char_val_read_req_t *)eventParameter;
            read_req_handler(readReqParam);
            break;
        default:
            printf("BLE request not found\n");
            break;
    }
}

void read_req_handler(cy_stc_ble_gatts_char_val_read_req_t *readReqParam) {
    float ret_f;
    switch (readReqParam->attrHandle) {
        case (CY_BLE_NSTIM_ERR_CHAR_HANDLE):
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &err, 1);
            break;
        case (CY_BLE_NSTIM_STIM_STATE_CHAR_HANDLE):
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &stim_state, 2);
            break;
        case (CY_BLE_DC_SERV_BASE_CURR_CHAR_HANDLE):;
            ret_f = VDAC_TO_CURR(dc_vdac_base);
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &ret_f, 4);
            break;
        case (CY_BLE_DC_SERV_TAR_CURR_CHAR_HANDLE):;
            ret_f = CURR_TO_VDAC(dc_vdac_target);
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, &ret_f, 4);
            break;
        case (CY_BLE_DC_SERV_TIMING_CHAR_HANDLE):
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle,
                                              &dc_phase_timings, 12);
            for (int i = 0; i < 8; i++) {
                printf("0x%x ", ((uint8_t *)dc_phase_timings)[i]);
            }
            break;
        case (CY_BLE_DC_SERV_PULSES_CHAR_HANDLE):
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle, 
                                             &dc_pulse_num, 4);
            break;
        case (CY_BLE_AC_SERV_PULSES_CHAR_HANDLE):
            CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(readReqParam->attrHandle,
                                             &ac_pulse_num, 4);
            break;
        default:
            printf("Attr handle '0x%x' not found\n", readReqParam->attrHandle);
    }
}

void bleInterruptNotify() {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void bleTask(void *arg) {
    (void)arg;
    printf("BLE Task Started\r\n");

    bleSemaphore = xSemaphoreCreateCounting(UINT_MAX, 0);

    Cy_BLE_Start(genericEventHandler);

    while (Cy_BLE_GetState() != CY_BLE_STATE_ON) {
        Cy_BLE_ProcessEvents();
    }

    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);

    for (;;) {
        xSemaphoreTake(bleSemaphore, portMAX_DELAY);
        Cy_BLE_ProcessEvents();
        vTaskDelay(100);
    }
}

void set_dc_slope() {
    dc_slope = ((double)dc_vdac_target - (double)dc_vdac_base) /
             (double)dc_phase_timings[0];
}

int compliance_check(uint16_t p1_time, uint32_t p1_dac, uint16_t p2_time,
                     uint32_t p2_dac) {
    return ABS(p1_time * (V0 - p1_dac)) == ABS(p2_time * (V0 - p2_dac));
}

void userIsr(void) {
    uint8_t intrStatus;

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_GetInterruptStatus();
    if (intrStatus) {
        /* Clear the interrupt. */
        VDAC_ClearInterrupt();
        /* Run burst handler or dc handler depending on stim_type */
        if (stim_state[0]) {
            stim_state[1] ? dc_handler() : burst_handler();
        } else {
            VDAC_SetValueBuffered(V0);
        }
    }
}

void dc_handler() {
    counter++;
    if (counter >= dc_phase_timings[phase]) {
        if (phase == 4) {
            dc_pulse_done++;
            if (dc_pulse_done < dc_pulse_num || dc_pulse_num == 0) {
                phase = 0;
            } else {
                command_stop(0);
                return;
            }
        } else {
            phase++;
            counter = 0u;
        }
    }
    if (phase == 0) {
        Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, 0);
        Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, 1);
        Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, 1);
        Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, 0);
        /* Holding at base */
        vdac_curr = dc_vdac_base;

    } else if (phase == 1) {
        /* Ramping up */
        Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, 1);
        Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, 0);
        Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, 0);
        Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, 1);
        vdac_curr = (uint32_t)((double)counter * dc_slope + dc_vdac_base);

    } else if (phase == 2) {
        /* Holding at high */
        Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, 0);
        Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, 1);
        Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, 1);
        Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, 0);
        vdac_curr = dc_vdac_target;

    } else if (phase == 3) {
        /* Ramping down */
        Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, 1);
        Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, 0);
        Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, 0);
        Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, 1);
        vdac_curr = (uint32_t)((double)counter * (-dc_slope) + dc_vdac_target);
    }
    VDAC_SetValueBuffered(vdac_curr);
}

void burst_handler() {
    counter++;
    if (counter >= ac_phase_timings[phase]) {
        counter = 0u;
        if (phase < 3) {
            phase++;
            Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, (phase & 1) ? 1 : 0);
            Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, (phase == 0) ? 1 : 0);
            Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, (phase & 1) ? 0 : 1);
            Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, (phase == 0) ? 0 : 1);
        } else if (phase == 3) {
            ac_pulse_done++;
            if (ac_pulse_done < ac_pulse_num) {
                // Not finished with this burst
                phase = 0;

                Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, (phase & 1) ? 1 : 0);
                Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, (phase == 0) ? 1 : 0);
                Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, (phase & 1) ? 0 : 1);
                Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, (phase == 0) ? 0 : 1);
            } else {
                // Finished with this burst
                phase++;
                ac_pulse_done = 0;
                // SWITCHES
            }
        } else if (phase == 4) {
            ac_burst_done++;
            if (ac_burst_done < ac_burst_num || ac_burst_num == 0) {
                phase = 0;
            } else {
                command_stop(0);
                return;
            }
        }
    }

    VDAC_SetValueBuffered(ac_vdac_values[phase]);
}

int command_start(uint32_t param) {
    printf("Inside start command with param %u\n", param);
    stim_state[0] = 1;
    // Compliance check and err if not passed
    return 0;
}

int command_stop(uint32_t param) {
    printf("Inside command stop with param %u\n", param);
    stim_state[0] = 0;
    counter = 0;
    ac_burst_done = 0;
    ac_pulse_done = 0;
    vdac_curr = 0;
    return 0;
}

int command_stim_type(uint32_t param) {
    if (param == 0 || param == 1) {
        stim_state[1] = param;
        command_stop(0);
        return 0;
    }
    return 1;
}

int command_anodic_cathodic(uint32_t param) {
    (void)param;
    if (param == 0 || param == 1) {
        anodic = param;
        ac_vdac_values[1] = INVERT_VDAC(ac_vdac_values[1]);
        ac_vdac_values[3] = INVERT_VDAC(ac_vdac_values[3]);
        return 0;
    } else {
        return 1;
    }
}

float uint32_to_float(uint32_t u) {
    overlay_t p;
    p.u = u;
    return p.f;
}

uint32_t float_to_uint32(float f) {
    overlay_t p;
    p.f = f;
    return p.u;
}

int uint_to_int(uint32_t u) {
    uint_int_overlay_t p;
    p.u = u;
    return p.i;
}

int command_ac_phase_one(uint32_t param) {
    ac_phase_timings[1] = US_TO_PT(param);
    return 0;
}

int command_ac_phase_two(uint32_t param) {
    ac_phase_timings[3] = US_TO_PT(param);
    return 0;
}

int command_ac_phase_gap(uint32_t param) {
    ac_phase_timings[2] = US_TO_PT(param);
    return 0;
}

int command_ac_stim_gap(uint32_t param) {
    ac_phase_timings[0] = US_TO_PT(param);
    return 0;
}

int command_ac_burst_gap(uint32_t param) {
    ac_phase_timings[4] = US_TO_PT(param);
    return 0;
}

int command_ac_pulse_num(uint32_t param) {
    ac_pulse_num = param;
    return 0;
}

int command_ac_burst_num(uint32_t param) {
    ac_burst_num = param;
    return 0;
}

int command_ac_phase_one_curr(uint32_t param) {
    int curr_uamps = uint_to_int(param);
    printf("Requesting %d uamps\n", curr_uamps);
    float curr = ((float) curr_uamps) / 1000;
    // float curr = uint32_to_float(param);
    if (curr == NAN || curr < -3.48 || curr > 3.48) {
        printf("Invalid current\n");
        return 1;
    }
    
    printf("Current: %f\n", curr);
    
    uint32_t vdac_val = CURR_TO_VDAC(curr);
    
    printf("To vdac: %u\n", vdac_val);

    /* if ((anodic && vdac_val > V0) || (!anodic && vdac_val < V0)) {
        return 1;
    } else */
    if (vdac_val > VDAC_MAX) {
        return 2;
    }

    ac_vdac_values[1] = vdac_val;
    return 0;
}   

int command_ac_phase_two_curr(uint32_t param) {
    int curr_uamps = uint_to_int(param);
    float curr = ((float) curr_uamps) / 1000;
    // float curr = uint32_to_float(param);
    if (curr == NAN || curr < -3.48 || curr > 3.48) {
        printf("Invalid current\n");   
        return 1;
    }
    printf("Curr: %f\n", curr);
    uint32_t vdac_val = CURR_TO_VDAC(curr);
    printf("VDAC: %u\n", vdac_val);

    /*if ((!anodic && vdac_val > V0) || (anodic && vdac_val < V0)) {
        return 1;
    } else */if (vdac_val > VDAC_MAX) {
        return 2;
    }

    ac_vdac_values[3] = vdac_val;
    return 0;
}

int command_dc_ramp_time(uint32_t param) {
  dc_phase_timings[0] = US_TO_PT(param);
  set_dc_slope();
  return 0;
}

int command_dc_hold_time(uint32_t param) {
  dc_phase_timings[1] = US_TO_PT(param);
  return 0;
}

int command_dc_curr_target(uint32_t param) {
    int curr_uamps = uint_to_int(param);
    float curr = ((float) curr_uamps) / 1000;
    //float curr = uint32_to_float(param);
    if (curr == NAN || curr < -3.48 || curr > 3.48) {
        printf("Invalid current\n");
    }
    
    uint32_t vdac_val = CURR_TO_VDAC(curr);
    
    if (vdac_val > VDAC_MAX || vdac_val < V0) {
        return 1;
    }
    dc_vdac_target = vdac_val;
    return 0;
}

int main(void) {
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();
    printf("Process started\n");

    (void)Cy_SysInt_Init(&SysInt_VDAC_cfg, userIsr);

    NVIC_EnableIRQ(SysInt_VDAC_cfg.intrSrc);
    VDAC_Start();
    Cy_GPIO_Write(HOWLAND_NEG_EN_0_PORT, HOWLAND_NEG_EN_0_NUM, 1);
    Cy_GPIO_Write(HOWLAND_POS_EN_0_PORT, HOWLAND_POS_EN_0_NUM, 1);

    set_dc_slope();

    xTaskCreate(bleTask, "bleTask", 1024, 0, 1, 0);

    vTaskStartScheduler();
    for (;;) {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
