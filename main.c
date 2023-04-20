/*
 * @file main.c
 * @brief Main file for the helicopter control project
 * @author Jack Duignan
 * @date 2023-03-12
 * 
 * This file contains the main function for the helicopter control project. 
 * It takes inspiration from the lab 4 code produced by P.J. Bones	UCECE
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
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "utils/ustdlib.h"
#include "stdio.h"

#include "circBufT.h"
#include "buttons4.h"
#include "serialUART.h"

#include "altitude.h"
#include "display.h"
#include "yaw.h"


// ========================= Constants and types =========================
#define SYSTICK_RATE_HZ 64 // 2 * CIRC_BUFFER_SIZE * Altitued Rate (4 Hz)
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ 

#define CIRC_BUFFER_SIZE 8 // size of the circular buffer used to store the altitued samples

// #define DEBUG // Sends information over serial UART


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

    // Check if the slow tick period has elapsed
    if (tickInteruptCount >= slowTick_period) {
        // Reset the tick count
        tickInteruptCount = 0;

        slowTickFlag = true;
    }

    // Initiate the next ADC conversion
    altitude_read(); // technically this should not be called in interupt handler but it is done in labs 3 and 4

    // Update the button state
    updateButtons(); // technically this should not be called in interupt handler but it is done in labs 3 and 4
}


/**
 * @brief Initialize the system clock (Taken from lab 4 code)
 * 
 */
void clock_init(void) {
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
 * @brief Send altitued information over serial UART
 * 
 */
void sendSerial(void) {
    // Send current altitude over UART
    char string[200];

    uint8_t yaw_raw = yaw_getChannels();
    bool channelA = yaw_raw & 1; 
    bool channelB = yaw_raw & (1 >> 1);
    bool channelA_prev = yaw_raw & (1 >> 2);
    bool channelB_prev = yaw_raw & (1 >> 3);

    usnprintf (string, sizeof(string), 
        "Mean Alt: %3d, Yaw: %3d, CHB: %d, CHA: %d, CHB prev: %d, CHA prev: %d", 
        altitude_get(), yaw_get(), channelB, channelA, channelB_prev, channelA_prev);

    serialUART_SendInformation(string);
}


/**
 * @brief Main function for the helicopter control project
 * 
 * @return int 
 */
int main(void) {
    // ========================= Initialise =========================
    clock_init();
    serialUART_init();
    altitude_init(CIRC_BUFFER_SIZE);
    initButtons ();
    initDisplay ();

    altitude_setMinimumAltitude(); // Set the minimum altitude to the current altitude

    uint8_t displayState = 1;

    // Enable interrupts to the processor.
    IntMasterEnable();


    // ========================= Main Loop =========================
    while (1) {
        // Check if the slow tick flag is set
        if (slowTickFlag) {
            slowTickFlag = false;

            #ifdef DEBUG
            sendSerial();
            #endif
        }

        // Reset the minimum altitude 
        if (checkButton(LEFT) == PUSHED) {
            altitude_setMinimumAltitude();
        }

        // Display Altitude and yaw
        displayYawAndAltitude(yaw_get(), altitude_get());
    }
}
