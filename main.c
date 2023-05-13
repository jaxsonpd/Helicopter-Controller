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


// ========================= Constants and types =========================
#define SYSTICK_RATE_HZ 64 // 2 * CIRC_BUFFER_SIZE * Altitued Rate (4 Hz)
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ 

#define CIRC_BUFFER_SIZE 8 // size of the circular buffer used to store the altitued samples

#define DEBUG // Sends information over serial UART

// Channel A input pin for yaw (J1-03)
#define YAW_ENC_CHA_PIN GPIO_PIN_0 
#define YAW_ENC_CHA_PORT GPIO_PORTB_BASE

enum MAIN_STATE {LANDED, FLYING, LANDING, TAKING_OFF};
enum TAKE_OFF_STATE {TAKEOFF_START, TAKE_OFF_ROTATE, TAKE_OFF_RISING, TAKE_OFF_DONE};
enum LANDING_STATE {LANDING_START, LANDING_ROTATE, LANDING_DESCENDING, LANDING_DONE};

// ========================= Global Variables =========================
bool slowTickFlag = false;

uint32_t deltaT = 0; // the time between each PID loop update [ms]

uint8_t altitudeSetpoint = 0;
int16_t yawSetpoint = 0;

uint8_t state = 0; // the current state of the helicopter

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
 * @brief Take off the helicopter
 * 
 */
void heli_takeoff(void) {
    static uint8_t takeOffState = 0;

    switch (takeOffState) {
    case TAKEOFF_START:
        motorControl_enable(MAIN_MOTOR);
        motorControl_enable(TAIL_MOTOR);

        motorControl_setAltitudeSetpoint(0);
        motorControl_setYawSetpoint(0);
        altitudeSetpoint = 0;
        yawSetpoint = 0;

        takeOffState = TAKE_OFF_ROTATE;
        break;

    case TAKE_OFF_ROTATE:
        if (true) {
            yaw_reset();
            motorControl_setYawSetpoint(0);

            takeOffState = TAKE_OFF_RISING;
        } else {
            motorControl_setYawSetpoint(yaw_get() + 5);
            yawSetpoint = yaw_get() + 5;
        }
        break;
    
    case TAKE_OFF_RISING:
        if (altitude_get() >= 10) {
            takeOffState = TAKE_OFF_DONE;
        }

        motorControl_setAltitudeSetpoint(10);
        altitudeSetpoint = 10;
        break;
    
    case TAKE_OFF_DONE:
        takeOffState = TAKEOFF_START;
        state = FLYING;
        break;
    }
}


/**
 * @brief Land the helicopter
 * 
 */
void heli_land(void) {
    static uint8_t landingState = 0;
    static uint16_t referenceTimer = 0;

    switch (landingState) {
    case LANDING_START:
        if (altitude_get() <= 10) {
            landingState = LANDING_ROTATE;
        }

        motorControl_setAltitudeSetpoint(10);
        altitudeSetpoint = 10; 
        break;

    case LANDING_ROTATE:
        if (yaw_get() == 0) {
            referenceTimer += 1;
            if (referenceTimer > 100) {
                referenceTimer = 0;
                landingState = LANDING_DESCENDING;
            }
        } else {
            referenceTimer = 0;	
        }

        motorControl_setAltitudeSetpoint(10);
        motorControl_setYawSetpoint(0);
        altitudeSetpoint = 10;
        yawSetpoint = 0;
        break;

    case LANDING_DESCENDING:
        if (altitude_get() < 1) {
            landingState = LANDING_DONE;
        }

        motorControl_setAltitudeSetpoint(0);
        altitudeSetpoint = 0;
        break;

    case LANDING_DONE:
        motorControl_disable(MAIN_MOTOR);
        motorControl_disable(TAIL_MOTOR);

        landingState = LANDING_START;
        state = LANDED;
        break;
    }
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
    initSwitch ();
    
    initDisplay (); 
    yaw_init ();
    motorControl_init();

    // Enable interrupts to the processor.
    IntMasterEnable();
    
    altitude_setMinimumAltitude(); // Set the minimum altitude to the current altitude
    
    motorControl_setAltitudeSetpoint(0);
    motorControl_setYawSetpoint(0);
    altitudeSetpoint = 0;
    yawSetpoint = 0;

    // ========================= Main Loop =========================
    while (1) {
        // Check if the slow tick flag is set
        if (slowTickFlag) {
            slowTickFlag = false;

            serialUART_SendInformation(yawSetpoint, yaw_get(), altitudeSetpoint, altitude_get(), motorControl_getMainRotorDuty(), motorControl_getTailRotorDuty(), state);
        }

        // Display Altitude and yaw
        displayYawAndAltitude(yaw_get(), altitude_get());

        // Update the PID controller
        motorControl_update(deltaT); // [ms]
        deltaT = 0;

        // Update the button state
        updateButtons();

        // Update the switches
        updateSwitch();

        // FSM
        switch (state) {
        case LANDED:
            if (checkSwitch(SW1) == SWITCH_UP) {
                state = TAKING_OFF;
            }
            break;
        
        case TAKING_OFF:
            heli_takeoff();
            break;
        
        case FLYING:
            if (checkSwitch(SW1) == SWITCH_DOWN) {
                state = LANDING;
            }

            // Check altitude setpoint
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
            break;
        
        case LANDING:
            heli_land();
            break;        
        }
    }
}
