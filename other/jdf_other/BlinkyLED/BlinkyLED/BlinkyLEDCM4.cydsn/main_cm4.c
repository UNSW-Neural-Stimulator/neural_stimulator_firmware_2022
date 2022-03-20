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

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    int waitPeriod = 500;
    int adjust = 1;
    for(;;)
    {
        Cy_GPIO_Write(RED_PORT, RED_NUM, 0);
        CyDelay(waitPeriod);
        Cy_GPIO_Write(RED_PORT, RED_NUM, 1);
        CyDelay(waitPeriod);
    }
}

/* [] END OF FILE */
