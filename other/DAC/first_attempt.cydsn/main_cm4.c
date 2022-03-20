#include "project.h"

#define V_MAX 0x7FF

uint32_t voltage = 0u;

/* Main global variables:
v_high: The peak voltage level (max is 0x7FF)
inclramp: (Include ramp) whether or not a rmap will be present

*/
uint32_t v_high = V_MAX;
bool inclramp = true;
int high_duration = 1000;
int ramp_duration = 500;

/* Extra variables:
    high_duration: How long the waveform spends on HIGH, (milliseconds)
*/

int time_spent_on_high = 0;
int time_spent_ramping = 0;
bool currently_ramping = true;
bool ramp_up = true;



void dacUpdate(void);

int main(void)
{
    /* Enable global interrupts. */
    __enable_irq();

    (void)Cy_SysInt_Init(&SysInt_1_cfg, dacUpdate);

    /* Enable the interrupt. */
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);

    /* Start the component. */
    VDAC_1_Start();


    for(;;)
    {
    }
}

void dacUpdate(void) {
    uint8_t intrStatus;

    /* Check that an interrupt occurred in the VDAC component instance. */
    intrStatus = VDAC_1_GetInterruptStatus();
    if (intrStatus){
        /* Clear the interrupt. */
        VDAC_1_ClearInterrupt();
            
        /* Set the next value that the DAC will output. */
        VDAC_1_SetValue(voltage);
        
        
        //If "include-ramp" is on, procude "ramp signal"
        if (inclramp) {
            if (currently_ramping) {
                if (ramp_up) {
                    if (time_spent_ramping == ramp_duration) {
                        currently_ramping = false;
                        ramp_up = false;
                    } else {
                        voltage = (v_high * time_spent_ramping)/ramp_duration;
                        time_spent_ramping++;   
                    }
                } else {
                    if (time_spent_ramping == 0) {
                        ramp_up = true;
                    } else {
                        time_spent_ramping--;  
                        voltage = (v_high *time_spent_ramping)/ramp_duration;
                    }
                    
                }
                
            } else {
                if (time_spent_on_high == high_duration) {
                    currently_ramping = true;
                    ramp_up = false;
                    time_spent_on_high = 0;
                } else {
                    time_spent_on_high++;    
                }
                
            }
        } 
        
        //Produce square wave if ramp option is off
        else {
            
            //Toggle voltage when half-period is complete
            //Also reset "time_on_high"
            if (time_spent_on_high == high_duration) {
                if (voltage == v_high) {
                    voltage = 0;
                } else {
                    voltage = v_high;   
                }
                time_spent_on_high = 0;
            } else {
                time_spent_on_high++;
            }
        }
        
       

        
    }
}


/* [] END OF FILE */
