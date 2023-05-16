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
#define PWM_RATE_MAIN_HZ 300
#define PWM_RATE_TAIL_HZ 300

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

enum PWM_MOTOR {MAIN_MOTOR, TAIL_MOTOR};

// ===================================== Globals ======================================
bool masterEnable = false; // Master enable for the PWM moduals

// ===================================== Function Definitions =========================
/**
 * @brief set the PWM parameters
 * @param duty the duty cycle of the PWM (0 = off, 101 = enabled)
 * @param motor the motor to set the PWM on
 *
 */
void PWM_set (uint8_t duty, uint8_t motor) {
    if (motor == MAIN_MOTOR) {
        // Calculate the PWM period corresponding to the freq.
        uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_MAIN_HZ;

        PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
        PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM,
            ui32Period * duty / 100);
    }

    if (motor == TAIL_MOTOR) {
        // Calculate the PWM period corresponding to the freq.
        uint32_t ui32Period =
            SysCtlClockGet() / PWM_DIVIDER / PWM_RATE_TAIL_HZ;
        
        PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
        PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
            ui32Period * duty / 100);
    }
}

/**
 * @brief Disable the give PWM signal
 * @param motor the motor to disable
 * 
 */
void PWM_disable(uint8_t motor) {
    if (motor == MAIN_MOTOR) {
        PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    } else if (motor == TAIL_MOTOR) {
        PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
    }
}

/**
 * @brief Enable the give PWM signal
 * @param motor the motor to enable
 * 
 */
void PWM_enable(uint8_t motor) {
    if (motor == MAIN_MOTOR) {
        PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    } else if (motor == TAIL_MOTOR) {
        PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
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
    PWM_set(0, MAIN_MOTOR);

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
    PWM_set(0, TAIL_MOTOR);

    // Enable the PWM generator
    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}
