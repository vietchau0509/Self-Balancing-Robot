// Motor Control Library

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "motor_control.h"
#include "gpio.h"

// Bitband aliases
#define DIRECTION_F    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define DIRECTION_A    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
// Pins
#define PWM_PF PORTF,2
#define DIR_PF PORTF,3

#define PWM_PA PORTA,6
#define DIR_PA PORTA,7

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize motor control
void initMotorControl()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);
    enablePort(PORTF);
    enablePort(PORTA);

    // Configure PWM_PA and DIRECTION outputs
    selectPinPushPullOutput(DIR_PA);
    selectPinPushPullOutput(PWM_PA);
    setPinAuxFunction(PWM_PA, GPIO_PCTL_PA6_M1PWM2);

    // Configure PWM_PF and DIRECTION outputs
    selectPinPushPullOutput(DIR_PF);
    selectPinPushPullOutput(PWM_PF);
    setPinAuxFunction(PWM_PF, GPIO_PCTL_PF2_M1PWM6);

    // Configure PWM module 1 to drive servo
    // PWM on M1PWM6 (PF2), M0PWM3a
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;               // reset PWM1 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1;              // leave reset state
    _delay_cycles(3);                                // wait 3 clocks
    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1
    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM1_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_1_CMPA_R = 0;                               // PWM off (0=always low, 1023=always high)
    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3


    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM1_3_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_CMPA_R = 0;                               // PWM off (0=always low, 1023=always high)
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM2EN;               // enable PWM output


}
void setMotor(unsigned int n, unsigned int dutyCycle, bool dir)
{
    // Set the motor direction
    switch(n)
    {
        case 0:
            DIRECTION_A = dir;
            PWM1_1_CMPA_R = dutyCycle;
            break;
        case 1:
            DIRECTION_F = dir;
            PWM1_3_CMPA_R = dutyCycle;
            break;
    }
}

