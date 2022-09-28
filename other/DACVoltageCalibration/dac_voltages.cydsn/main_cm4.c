#include "project.h"
#include <stdio.h>


// (Time spent on each output in seconds)/500
#define TIME_ON_EACH_DAC_OUTPUT 400

// Time spent on each DAC output (every increment represent 2us passing)
uint32_t counter = 0u;

// Current DAC voltage level
int voltage_level = 4000;


void userIsr(void);

int main(void)
{
    
    UART_1_Start();
    //setting iso to high
    Cy_GPIO_Write(P5_2_PORT, P5_2_NUM, 1);
    //enable high and low voltage lines
    Cy_GPIO_Write(P5_6_PORT, P5_6_NUM, 1);
    Cy_GPIO_Write(P0_5_PORT, P0_5_NUM, 1);
    // set all other switches to low
    Cy_GPIO_Write(P5_3_PORT, P5_3_NUM, 0);
    Cy_GPIO_Write(P5_4_PORT, P5_4_NUM, 0);
    Cy_GPIO_Write(P5_5_PORT, P5_5_NUM, 0);
    
    
    
    //turn led blue on
    //Cy_GPIO_Write(LED_BLUE_PORT, LED_BLUE_NUM, 1);
    
    
    __enable_irq(); /* Enable global interrupts. */

    (void)Cy_SysInt_Init(&SysInt_1_cfg, userIsr);
    printf("line 35\r\n");
    NVIC_ClearPendingIRQ(SysInt_1_cfg.intrSrc);/* Clears the interrupt */
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);
    
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    
    VDAC_1_Start();
    
    printf("anything\r\n");

}

void userIsr(void) {
    uint8_t intrStatus;
    //printf("line 52\r\n");
    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_1_GetInterruptStatus();
    //printf("line 55\r\n");
    if (intrStatus)
    {
        //printf("line 58\r\n");
        VDAC_1_ClearInterrupt();
        VDAC_1_SetValueBuffered(voltage_level);
        //printf("line 60\r\n");
        if (counter >= TIME_ON_EACH_DAC_OUTPUT) {
            //printf("line 62\r\n");
            voltage_level += 1;
            if (voltage_level > 4095) {
                voltage_level = 0;
            }

            //Puts current DAC output value
            printf("Set dac value is %u\r\n", voltage_level);
            counter = 0u;
        }
        
        
        
        counter++;
        
    }
    
}

/* [] END OF FILE */
