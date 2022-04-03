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
#include <stdio.h>

// The THREE_VOLTS value is retrieved from dividing the MAX possible DAC voltage (3.2992)
// then multiplying by 3 (Currently a float)
#define THREE_VOLTS (CY_CTDAC_UNSIGNED_MAX_CODE_VALUE * 3/3.2992)

// The desired increment value to add to the DAC output each cycle
#define INCREMENT_VALUE (uint32_t)(THREE_VOLTS/60.0)

// (Time spent on each output in seconds)/500
#define TIME_ON_EACH_DAC_OUTPUT 5000

// Time spent on each DAC output (every increment represent 2us passing)
uint32_t counter = 0u;

// Current DAC voltage level
uint32_t voltage_level = 0u;

// String that gets output to UART terminal so that DAC value can be observed
char outputstr[30];

// The output voltage that SHOULD be dipslayed on the measuring device 
// (This should be used for comparison)
int curr_voltage_output = 0;

//Used to correct for missing component after integer division
int correction_counter = 0;

void userIsr(void);

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    (void)Cy_SysInt_Init(&SysInt_1_cfg, userIsr);
    
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);
    
    VDAC_1_Start();
    UART_1_Start();
    
    for(;;)
    {  
    }
}

void userIsr(void) {
    uint8_t intrStatus;

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_1_GetInterruptStatus();
    if (intrStatus)
    {
        VDAC_1_ClearInterrupt();
        VDAC_1_SetValueBuffered(voltage_level);
    
        if (counter >= TIME_ON_EACH_DAC_OUTPUT) {
            if (correction_counter >= 15) {
                correction_counter = 0;
                voltage_level++;
            }
            voltage_level += INCREMENT_VALUE;
            curr_voltage_output += 50;
            //Puts current DAC output value 
            sprintf(outputstr, "%lu (%d mV)\n", voltage_level, curr_voltage_output);
            UART_1_PutString(outputstr);
            counter = 0u;
            correction_counter++;
        }
        
        if (voltage_level >= THREE_VOLTS) {
            voltage_level = 0u;
            curr_voltage_output = 0;
            sprintf(outputstr, "%lu (%d mV)\n", voltage_level, curr_voltage_output);
            UART_1_PutString(outputstr);
        }
        
        counter++;
        
    }
    
}

/* [] END OF FILE */
