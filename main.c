/*
 * @file main.c
 * @brief Main file for the helicopter control project
 * @author Jack Duignan
 * @date 2023-03-12
 * 
 * This file contains the main function for the helicopter control project. 
 * It takes insperation from the lab 4 code produced by P.J. Bones	UCECE
*/

// ========================= Include files =========================
#include <stdint.h>
#include <stdbool.h>

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
#include "stdio.h"

#include "circBufT.h"
#include "buttons4.h"

#include "SerialUART.h"
#include "altitude.h"
// #include "inc/hw_memmap.h"
// #include "driverlib/adc.h"

// ========================= Constants and types =========================
#define SYSTICK_RATE_HZ 100
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ
#define CIRC_BUFFER_SIZE 8 // size of the circular buffer

// ========================= Global Variables =========================
bool slowTickFlag = false;

/**
 * @brief System tick interupt handler used to trigger the adc conversion
 * 
 */
void SysTickInterupt_Handler(void) {
    static uint32_t tickInteruptCount = 0;
    const uint32_t slowTick_period = SYSTICK_RATE_HZ / SLOWTICK_RATE_HZ;

    // Increment the tick count
    tickInteruptCount++;

    if (tickInteruptCount >= slowTick_period) {
        // Reset the tick count
        tickInteruptCount = 0;

        slowTickFlag = true;
    }

    // Initiate the next ADC conversion
    altitude_read();

    // Update the button state
    updateButtons();
}

/**
 * @brief Initialize the system clock (Taken from lab 4 code)
 * 
 */
void Clock_init(void) {
    // Set the clock to 20 MHz
    SysCtlClockSet ( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ );

    // Set the period of the system tick timer
    SysTickPeriodSet ( SysCtlClockGet() / SYSTICK_RATE_HZ );

    // Enable the system tick interupt
    SysTickIntRegister(SysTickInterupt_Handler);

    // Enable the system tick interupt and device
    SysTickIntEnable();
    SysTickEnable();
}

/**
 * @brief Slow tick handler
 * 
 */
void slowTick_Handler(void) {
    slowTickFlag = false;

    char string[100];

    usnprintf (string, sizeof(string), "Mean Alt: %3d, Raw ADC: %4d, Sample Number: %5d\r\n", altitude_get(), altitude_getRaw(), altitude_getSamples());

    // Send the current time over UART
    SerialUART_SendInformation(string);
}

/**
 * @brief Main function for the helicopter control project
 * 
 * @return int 
 */

int main(void) {
    // ========================= Initialise =========================
    Clock_init();
    SerialUART_init();
    altitude_init(CIRC_BUFFER_SIZE);
    initButtons ();

    // Enable interrupts to the processor.
    IntMasterEnable();

    // ========================= Main Loop =========================
    while (1) {

        // Check if the slow tick flag is set
        if (slowTickFlag) {
            slowTick_Handler();
        }

        if (checkButton(LEFT) == PUSHED) {
            altitude_reset();
        }
    }

}

