/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This example demonstrates how to use the Voltage DAC (12-bit)
*              component and CTDAC as a sawtooth wave generator in a PSoC 6 MCU.
*
* Related Document: CE220923.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 MCU BLE Pioneer Kit or
*                      CY8CKIT-062 PSoC 6 MCU Pioneer Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "project.h"
#include "ctdac/cy_ctdac.h"

#define SW_ISO P5_2_PORT
#define SW_LOAD  P5_3_PORT
#define SW_SHORT P5_4_PORT
#define SW_EVM P5_5_PORT

#define SW_ISO_NUM P5_2_NUM
#define SW_LOAD_NUM  P5_3_NUM
#define SW_SHORT_NUM P5_4_NUM
#define SW_EVM_NUM P5_5_NUM

// Stage indicates where we are in a pulse, 0 is interpulse, 1 is cathodic, 2 is interphase
// 3 is anodic
uint32_t phase = 0u;
// Counter indicates how far through a phase we are 
uint32_t counter = 0u;
// Current code is what the voltage is going to be set as 
uint32_t currentCode = 0u;

uint32_t vdac_values[4] = {2048u, 4095u, 2048u, 0u};
uint32_t phase_timings[4] = {100u, 50u, 50u, 50u};


void userIsr(void);
void swap_phase();

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes Interrupt Component
*   2. Starts the CTDAC hardware
*   3. On each interrupt, updates the CTDAC output value.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
    /* Enable global interrupts. */
    __enable_irq();

    /* Configure the interrupt and provide the ISR address. */
    (void)Cy_SysInt_Init(&SysInt_cfg, userIsr);

    /* Enable the interrupt. */
    NVIC_EnableIRQ(SysInt_cfg.intrSrc);
    
    /* Set default pin values */
    Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, 0);//set stim enable to high 
    Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, 1);//set dummy load to low
    Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, 1);//set short electrode to high 
    Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, 0);//set toggle output to low
    
    swap_phase();
   

    /* Start the component. */
    VDAC_Start();
    
    for(;;)
    {
    }
}

/*******************************************************************************
* Function Name: userIsr
********************************************************************************
*
*  Summary:
*  Interrupt service routine for the VDAC update.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
/* This function gets called when an interrupt occurs in any CTDAC on the PSoC 6 MCU device. */
void userIsr(void)
{
    uint8_t intrStatus;

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_GetInterruptStatus();
    if (intrStatus)
    {
        /* Clear the interrupt. */
        VDAC_ClearInterrupt();
       
        counter++;
        if (counter >= (phase_timings[phase] / 2)) {
            phase++;
            phase %= 4; 
            currentCode = vdac_values[phase];
            counter = 0u;
            Cy_GPIO_Write(SW_ISO, SW_ISO_NUM, (phase & 1) ? 1 : 0);
            Cy_GPIO_Write(SW_SHORT, SW_SHORT_NUM, (phase == 0) ? 1 : 0);
            Cy_GPIO_Write(SW_LOAD, SW_LOAD_NUM, (phase & 1) ? 0 : 1);
            Cy_GPIO_Write(SW_EVM, SW_EVM_NUM, (phase == 0) ? 0 : 1);
        }
        
        VDAC_SetValueBuffered(currentCode);
    }
}

void swap_phase() {
    int temp = vdac_values[1];
    vdac_values[1] = vdac_values[3];
    vdac_values[3] = temp;
}

/* [] END OF FILE */
