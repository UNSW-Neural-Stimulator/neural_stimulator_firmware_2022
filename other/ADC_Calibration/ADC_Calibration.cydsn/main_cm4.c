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
#include <math.h>

uint32_t vdac_out = 0u;
uint32_t counter = 0u;

void userIsr(void) {
    
    uint8_t intrStatus;
    

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_GetInterruptStatus();
    if (intrStatus) {
        /* Clear the interrupt. */
        VDAC_ClearInterrupt();
        /* Increment VDAC value every 10 seconds */
        if (counter >= 500) {
            vdac_out++;
            counter = 0u;
            //printf("REACHED HERE\r\n");
            
            if (vdac_out > 4095) {
                vdac_out = 0u;
            }
        }
        
        if (counter == 250) {
            Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
            uint16_t result = Cy_SAR_GetResult16(SAR, 0);
            printf("DAC Value: %u, ADC Measurement: %f\r\n", vdac_out, Cy_SAR_CountsTo_Volts(SAR,0, result));
        }
        
        VDAC_SetValueBuffered(vdac_out);
        counter++;
    }
    
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    UART_Start();
    ADC_Start();
    printf("Process started\n");
    
    (void)Cy_SysInt_Init(&DacInt_cfg, userIsr);
    
    NVIC_EnableIRQ(DacInt_cfg.intrSrc);
    
    VDAC_Start();
    
    Cy_GPIO_Write(HOWLAND_NEG_PORT, HOWLAND_NEG_NUM, 1);
    Cy_GPIO_Write(HOWLAND_POS_PORT, HOWLAND_POS_NUM, 1);
    
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
