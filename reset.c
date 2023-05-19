/** 
 * @file reset.c
 * @brief Performs a software reset of the microcontroller	
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-05-18
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"


#include "utils/ustdlib.h"

// ===================================== Constants ====================================
// Soft reset pin (J1-09)
#define SOFT_RESET_PERIPH SYSCTL_PERIPH_GPIOA
#define SOFT_RESET_PORT GPIO_PORTA_BASE
#define SOFT_RESET_PIN GPIO_PIN_6

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/**
 * @brief initialize the reset button
 * 
 */
void reset_init(void) {
    // Setup reset pin (active low)
    SysCtlPeripheralEnable(SOFT_RESET_PERIPH);
    GPIOPinTypeGPIOInput(SOFT_RESET_PORT, SOFT_RESET_PIN);
    GPIOPadConfigSet(SOFT_RESET_PORT, SOFT_RESET_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

/**
 * @brief check if the reset button is pressed and perform a reset if it is
 * 
 */
void reset_check(void) {
    if (GPIOPinRead(SOFT_RESET_PORT, SOFT_RESET_PIN) != SOFT_RESET_PIN) {
        SysCtlReset();
    }
}
