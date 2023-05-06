/** 
 * @file pwm.c
 * @brief PWM module for the helicopter control project
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @cite Adapted from P.J. Bones UCECE
 * @date 2023-05-06
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "stdio.h"

#include "pwm.h"

// ===================================== Constants ====================================
// PWM configuration
#define PWM_RATE_MAIN_HZ 150
#define PWM_RATE_TAIL_HZ 150

#define PWM_DIVIDER_CODE SYSCTL_PWMDIV_4
#define PWM_DIVIDER 4

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE	     PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM	 SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J3-10
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

// ===================================== Globals ======================================
uint8_t mainRotorDuty = 0;
uint8_t tailRotorDuty = 0;
bool masterEnable = false; // Master enable for the PWM moduals

// ===================================== Function Definitions =========================
/**
 * @brief set the PWM parameters
 * @param uint8_t duty the duty cycle of the PWM
 * @param uint8_t pin the pin to set the PWM on
 *
 */
static void setPWM (uint8_t duty, uint8_t pin) {
    if (pin == PWM_MAIN_GPIO_PIN) {
        if (duty == 0) {
            // Disable the output
            PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
        } else if (duty <= 100) {
            // Calculate the PWM period corresponding to the freq.
            uint32_t ui32Period =
                SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_MAIN_HZ;

            PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
            PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
                ui32Period * duty / 100);
        }
    }

    if (pin == PWM_TAIL_GPIO_PIN) {
        if (duty == 0) {
            // Disable the output
            PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
        } else if (duty <= 100) {
            // Enable the output
            PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

            // Calculate the PWM period corresponding to the freq.
            uint32_t ui32Period =
                SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_TAIL_HZ;
            
            PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
            PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
                ui32Period * duty / 100);
        }
    }
}

/** 
 * @brief setup the pwm moduals for the main and tail rotors
 * 
 */
void PWM_init(void) {
    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Main rotor PWM setup
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    setPWM(0, PWM_MAIN_GPIO_PIN);

    // Enable the PWM generator
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);

    // Tail rotor PWM setup
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Set the initial PWM parameters
    setPWM(0, PWM_TAIL_GPIO_PIN);

    // Enable the PWM generator
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}


/** 
 * @brief set the duty cycle of the main rotor
 * @param duty_cycle the duty cycle to set the main rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
uint8_t PWM_setMainRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > 100) {
        return 0; // Out of range
    }

    // Set the duty cycle
    mainRotorDuty = duty_cycle;
    setPWM(duty_cycle, PWM_MAIN_GPIO_PIN);
    return 1;
}

/** 
 * @brief set the duty cycle of the tail rotor
 * @param duty_cycle the duty cycle to set the tail rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
uint8_t PWM_setTailRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > 100) {
        return 0; // Out of range
    }

    // Set the duty cycle
    tailRotorDuty = duty_cycle;
    setPWM(duty_cycle, PWM_TAIL_GPIO_PIN);
    return 1;
}

/** 
 * @brief Return the current duty cycle of the main rotor
 * 
 * @return uint8_t duty cycle of the main rotor
 */
uint8_t PWM_getMainRotorDuty(void) {
    return mainRotorDuty;
}

/** 
 * @brief Return the current duty cycle of the tail rotor
 * 
 * @return uint8_t duty cycle of the tail rotor
 */
uint8_t PWM_getTailRotorDuty(void) {
    return tailRotorDuty;
}

/**
 * @brief Enable the PWM moduals
 * 
 */
void PWM_enable(void) {
    masterEnable = true;
    
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

/**
 * @brief Disable the PWM moduals
 * 
 */
void PWM_disable(void) {
    masterEnable = false;

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}
