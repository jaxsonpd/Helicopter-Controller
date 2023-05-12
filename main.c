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
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "utils/ustdlib.h"


#include "circBufT.h"
#include "buttons4.h"
#include "serialUART.h"

#include "altitude.h"
#include "display.h"
#include "yaw.h"
#include "MotorControl.h"


// ========================= Constants and types =========================
#define SYSTICK_RATE_HZ 64 // 2 * CIRC_BUFFER_SIZE * Altitued Rate (4 Hz)
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ 

#define CIRC_BUFFER_SIZE 8 // size of the circular buffer used to store the altitued samples

#define DEBUG // Sends information over serial UART

// Channel A input pin for yaw (J1-03)
#define YAW_ENC_CHA_PIN GPIO_PIN_0 
#define YAW_ENC_CHA_PORT GPIO_PORTB_BASE

// ========================= Global Variables =========================
bool slowTickFlag = false;

uint32_t deltaT = 0; // the time between each PID loop update [ms]

uint8_t altitudeSetpoint = 0;
int16_t yawSetpoint = 0;

// ========================= Function Definitions =========================
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

    // update the deltaT
    deltaT += 1000 / SYSTICK_RATE_HZ; // [ms]

    // Initiate the next ADC conversion
    altitude_read(); // technically this should not be called in interupt handler but it is done in labs 3 and 4
}


/**
 * @brief Initialize the system clock (Taken from lab 4 code)
 * @cite P.J. Bones	UCECE
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
    motorControl_init();

    // Enable interrupts to the processor.
    IntMasterEnable();
    
    altitude_setMinimumAltitude(); // Set the minimum altitude to the current altitude
    
    // Test the PID loop
    motorControl_setAltitudeSetpoint(0);
    motorControl_setYawSetpoint(0);
    altitudeSetpoint = 0;
    yawSetpoint = 0;

    motorControl_enable(MAIN_MOTOR);
    motorControl_enable(TAIL_MOTOR);

    // ========================= Main Loop =========================
    while (1) {
        // Check if the slow tick flag is set
        if (slowTickFlag) {
            slowTickFlag = false;

            #ifdef DEBUG
            serialUART_SendInformation(yawSetpoint, yaw_get(), altitudeSetpoint, altitude_get(), motorControl_getMainRotorDuty(), motorControl_getTailRotorDuty());
            #endif
        }

        // Display Altitude and yaw
        displayYawAndAltitude(yaw_get(), altitude_get());

        // Update the PID controller
        motorControl_update(deltaT); // [ms]
        deltaT = 0;

        // Change altitude setpoint
        if (checkButton(UP) == PUSHED) {
            altitudeSetpoint += 10;
            motorControl_setAltitudeSetpoint(altitudeSetpoint);
        }
        else if (checkButton(DOWN) == PUSHED) {
            altitudeSetpoint -= 10;
            motorControl_setAltitudeSetpoint(altitudeSetpoint);
        }

        // Check yaw
        if (checkButton(LEFT) == PUSHED) {
            yawSetpoint -= 150;
            yawSetpoint = (yawSetpoint <= -1800) ? yawSetpoint + 3600 : yawSetpoint;
            motorControl_setYawSetpoint(yawSetpoint);
        }
        else if (checkButton(RIGHT) == PUSHED) {
            yawSetpoint += 150;
            yawSetpoint = (yawSetpoint > 1800) ? yawSetpoint - 3600 : yawSetpoint;
            motorControl_setYawSetpoint(yawSetpoint);
        }

        // Update the button state
        updateButtons();

    }
}
