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

// Channel A input pin for yaw (J1-03)
#define YAW_ENC_CHA_PIN GPIO_PIN_0 
#define YAW_ENC_CHA_PORT GPIO_PORTB_BASE

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
    bool channelB = yaw_raw & (1 << 1);
    bool channelA_prev = yaw_raw & (1 << 2);
    bool channelB_prev = yaw_raw & (1 << 3);

    int32_t yaw = yaw_get();
    int32_t degrees = yaw / 10;
    // Find the decimal value an convert it to absolute value
    int32_t decimalDegrees = (yaw < 0) ? yaw % 10 * -1 : yaw % 10;

    usnprintf (string, sizeof(string), 
       "Desired Yaw: %4d, Yaw: %4d.%1d, Desired Alt: %3d%, Alt: %3d%, Main Motor PWM: %3d%, Tail Motor PWML %3d%, Operating mode: \n\r",
       YAW_SETPOINT, degrees, decimalDegrees, ALTITUDE_SETPOINT, altitude_get(), motorControl_getMainRotorDuty(), motorControl_getTailRotorDuty());
    // usnprintf (string, sizeof(string), "Alt: %3d, Yaw: %3d, Pin: %1d \n\r", altitude_get(), yaw_get(), GPIOPinRead(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN));

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
    yaw_init ();

    altitude_setMinimumAltitude(); // Set the minimum altitude to the current altitude

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
