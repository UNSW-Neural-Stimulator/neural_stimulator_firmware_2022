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

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    setvbuf(stdin, NULL, _IONBF, 0);
    
    char c;
    
    printf("Started UART\n");

    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
