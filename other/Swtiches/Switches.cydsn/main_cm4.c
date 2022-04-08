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

#define STIM_ENABLE P9_0_PORT
#define DUMMY_LOAD  P9_4_PORT
#define SHORT_ELECTRODE P9_1_PORT
#define TOGGLE_OUTPUT P9_3_PORT

#define STIM_ENABLE_NUM P9_0_NUM
#define DUMMY_LOAD_NUM  P9_4_NUM
#define SHORT_ELECTRODE_NUM P9_1_NUM
#define TOGGLE_OUTPUT_NUM P9_3_NUM

// Note there is kind of not a maximum for each of these phases, it is limited only by 
// uint overflow (each phase cannot be more than 4294s)
#define INTERPULSE_GAP 100u
#define CATHODIC_PHASE 50u
#define INTERPHASE_GAP 50u
#define ANODIC_PHASE 50u

// Stage indicates where we are in a pulse, 0 is interpulse, 1 is cathodic, 2 is interphase
// 3 is anodic
uint32_t phase = 0u;
// Counter indicates how far through a phase we are 
uint32_t counter = 0u;
// Current code is what the voltage is going to be set as 
uint32_t currentCode = 0u;

uint32_t vdac_values[4] = {2048u, 4095u - 25, 2048u, 0u + 25};
uint32_t phase_timings[4] = {INTERPULSE_GAP, CATHODIC_PHASE, INTERPHASE_GAP, ANODIC_PHASE};


void userIsr(void);

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
    Cy_GPIO_Write(STIM_ENABLE, STIM_ENABLE_NUM, 1);//set stim enable to high 
    Cy_GPIO_Write(DUMMY_LOAD, DUMMY_LOAD_NUM, 0);//set dummy load to low
    Cy_GPIO_Write(SHORT_ELECTRODE, SHORT_ELECTRODE_NUM, 1);//set short electrode to high 
    Cy_GPIO_Write(TOGGLE_OUTPUT, TOGGLE_OUTPUT_NUM, 0);//set toggle output to low
   

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
            Cy_GPIO_Write(SHORT_ELECTRODE, SHORT_ELECTRODE_NUM, (phase == 0) ? 1 : 0);
            Cy_GPIO_Write(TOGGLE_OUTPUT, TOGGLE_OUTPUT_NUM, (phase & 1) ? 1 : 0);
        }
        
        VDAC_SetValueBuffered(currentCode);
    }
}

/* [] END OF FILE */
