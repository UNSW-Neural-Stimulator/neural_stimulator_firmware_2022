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
#include <stdlib.h>

/*void userIsr(void) {
    
  int8_t intrStatus;
  intrStatus = VDAC_GetInterruptStatus();
  if (intrStatus) {  
    VDAC_ClearInterrupt();
    VDAC_SetValueBuffered(rand());
  }
}*/

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    UART_Start();
    printf("Process started\n");
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    //(void)Cy_SysInt_Init(&SysInt_VDAC_cfg, userIsr);

    //NVIC_EnableIRQ(SysInt_VDAC_cfg.intrSrc);
    VDAC_Start();
    VDAC_SetValueBuffered(2048);
    ADC_Start();
    Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
    
    int16_t result = 0;
    
    for(;;)
    {
        result = Cy_SAR_GetResult16(SAR,0);
        printf("Measured: %f\r\n", Cy_SAR_CountsTo_Volts(SAR,0,result));
        CyDelay(200);
    }
}

/* [] END OF FILE */
