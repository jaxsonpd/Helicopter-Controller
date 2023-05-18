/*
 * @file main.c
 * @brief Main file for the helicopter control project
 * @author Jack Duignan (jdu80@uclive.ac.nz), Daniel Hawes
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
#include "switch.h"
#include "main.h"
#include "reset.h"
#include "heliFunctions.h"

// ========================= Constants and types =========================
#define SYSTICK_RATE_HZ 64 // 2 * CIRC_BUFFER_SIZE * Altitued Rate (4 Hz)
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ 

#define CIRC_BUFFER_SIZE 8 // size of the circular buffer used to store the altitued samples

// ========================= Global Variables =========================
bool slowTickFlag = false; // Flag set by the systick interupt handler at a rate of SLOWTICK_RATE_HZ

uint32_t deltaT = 0; // the time between each PID loop update [ms]

heliInfo_t heliInfo = {0}; // The helicopter information struct

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

    switch_update();
    updateButtons();
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

// ===================================== Main =====================================
/**
 * @brief Main function for the helicopter control project
 * 
 * @return int 
 */
int main(void) {
    // ========================= Initialise Moduals =========================
    initButtons();
    switch_init();
    clock_init();
    serialUART_init();
    altitude_init(CIRC_BUFFER_SIZE);
    display_init ();
    yaw_init ();
    motorControl_init();
    reset_init();

    // Enable interrupts to the processor.
    IntMasterEnable();
    
    SysCtlDelay(16000000); // Wait to let the helicopter settle

    // Setup to start the program
    altitude_setMinimumAltitude(); // zero the altitude

    // Clean switch 
    switch_update();
    switch_update();
    switch_update();
    switch_check(SW1);

    heliInfo.mode = LANDED; // Start in landed mode

    // ========================= Main Loop =========================
    while (true) {
        // Update the helicopter device information
        heliInfo.altitude = altitude_get();
        heliInfo.yaw = yaw_get();
        heliInfo.mainMotorDuty = motorControl_getMainRotorDuty();
        heliInfo.tailMotorDuty = motorControl_getTailRotorDuty();
        
        // Check if the slow tick flag is set
        if (slowTickFlag) {
            slowTickFlag = false;

            serialUART_SendInformation(&heliInfo);
            main_display(&heliInfo);
        }
        
        // Check for a soft reset
        reset_check();

        // Update the PID controller
        motorControl_update(deltaT); // [ms]
        deltaT = 0;

        // FSM
        switch (heliInfo.mode) {
        case LANDED:
            if (switch_check(SW1) == SWITCH_UP) {
                heliInfo.mode = TAKING_OFF;
            }
            break;
        
        case TAKING_OFF:
            heliFunctions_takeoff(&heliInfo);
            break;
        
        case FLYING:
            if (switch_check(SW1) == SWITCH_DOWN) {
                heliInfo.mode = LANDING;
            }

            heliFunctions_updateSetpoints(&heliInfo);
            break;
        
        case LANDING:
            heliFunctions_land(&heliInfo);
            break;        
        }
    }
}
