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

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    //Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    int waitPeriod = 50;
    int adjust = 1;
    for(;;)
    {
        Cy_GPIO_Write(RED_PORT, RED_NUM, 0);
        CyDelay(waitPeriod);
        waitPeriod += 50 * adjust;
        Cy_GPIO_Write(RED_PORT, RED_NUM, 1);
        CyDelay(waitPeriod);
        waitPeriod += 50 * adjust;
        if (waitPeriod > 500 || waitPeriod < 100) adjust *= -1;
    }
}

/* [] END OF FILE */
