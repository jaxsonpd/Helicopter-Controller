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
#include "deviceInfo.h"

// ========================= Constants and types =========================
// Soft reset pin (J1-09)
#define SOFT_RESET_PERIPH SYSCTL_PERIPH_GPIOA
#define SOFT_RESET_PORT GPIO_PORTA_BASE
#define SOFT_RESET_PIN GPIO_PIN_6

#define SYSTICK_RATE_HZ 64 // 2 * CIRC_BUFFER_SIZE * Altitued Rate (4 Hz)
#define SLOWTICK_RATE_HZ 8  // Max rate = SYSTICK_RATE_HZ 

#define CIRC_BUFFER_SIZE 8 // size of the circular buffer used to store the altitued samples

enum TAKE_OFF_STATE {TAKEOFF_START, TAKE_OFF_ROTATE, TAKE_OFF_RISING, TAKE_OFF_DONE};
enum LANDING_STATE {LANDING_START, LANDING_ROTATE, LANDING_DESCENDING, LANDING_DONE};


// ========================= Global Variables =========================
bool slowTickFlag = false;

uint32_t deltaT = 0; // the time between each PID loop update [ms]

uint8_t altitudeSetpoint = 0;
int16_t yawSetpoint = 0;

uint8_t state = 0; // the current state of the helicopter

deviceInfo_t heliInfo = {0}; // the current helicopter information

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


/**
 * @brief initialise the soft reset pin
 * 
 */
void softReset_init(void) {
    // Setup reset pin (active low)
    SysCtlPeripheralEnable(SOFT_RESET_PERIPH);
    GPIOPinTypeGPIOInput(SOFT_RESET_PORT, SOFT_RESET_PIN);
    GPIOPadConfigSet(SOFT_RESET_PORT, SOFT_RESET_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}


/**
 * @brief Take off the helicopter
 * 
 */
void heli_takeoff(void) {
    static uint8_t takeOffState = 0;

    switch (takeOffState) {
    case TAKEOFF_START:
        heliInfo.altitudeSetpoint = 0;
        heliInfo.yawSetpoint = 0;

        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
        motorControl_setYawSetpoint(heliInfo.yaw);

        motorControl_enable(TAIL_MOTOR);
        motorControl_enable(MAIN_MOTOR);

        takeOffState = TAKE_OFF_ROTATE;
        break;

    case TAKE_OFF_ROTATE:
        if (yaw_getRef() == 0) {
            yaw_reset();
            heliInfo.yawSetpoint = 0;
            motorControl_setYawSetpoint(heliInfo.yawSetpoint);

            takeOffState = TAKE_OFF_RISING;
        } else {
            // Rotate the helicopter to face the reference
            heliInfo.yawSetpoint = yaw_get() + 150;
            heliInfo.yawSetpoint = (heliInfo.yawSetpoint > 1800) ? heliInfo.yawSetpoint - 3600 : heliInfo.yawSetpoint;

            motorControl_setYawSetpoint(heliInfo.yawSetpoint);
        }
        break;
    
    case TAKE_OFF_RISING:
        takeOffState = TAKE_OFF_DONE;
        break;
    
    case TAKE_OFF_DONE:
        takeOffState = TAKEOFF_START;
        heliInfo.mode = FLYING;
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

        heliInfo.altitudeSetpoint = (altitude_get() - 5);
        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
        break;

    case LANDING_ROTATE:
        if (yaw_get() <= 16 && yaw_get() >= -16) {
            referenceTimer += 1;
            if (referenceTimer > 10) {
                referenceTimer = 0;
                landingState = LANDING_DESCENDING;
            }
        } else {
            referenceTimer = 0;	
        }

        heliInfo.yawSetpoint = 0;
        heliInfo.altitudeSetpoint = 10;
        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
        motorControl_setYawSetpoint(heliInfo.yawSetpoint);
        break;

    case LANDING_DESCENDING:
        if (altitude_get() < 1) {
            landingState = LANDING_DONE;
        }

        heliInfo.altitudeSetpoint = 0;
        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
        break;

    case LANDING_DONE:
        motorControl_disable(MAIN_MOTOR);
        motorControl_disable(TAIL_MOTOR);

        landingState = LANDING_START;
        heliInfo.mode = LANDED;
        break;
    }
}


/**
 * @brief Update the helicopter setpoints while flying
 * 
 */
void updateSetpoints(void) {
    // Check altitude setpoint
    if (checkButton(UP) == PUSHED) {
        heliInfo.altitudeSetpoint += 10;
        heliInfo.altitudeSetpoint = (heliInfo.altitudeSetpoint > 100) ? 100 : heliInfo.altitudeSetpoint;

        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
    } else if (checkButton(DOWN) == PUSHED) {
        heliInfo.altitudeSetpoint -= 10;
        heliInfo.altitudeSetpoint = (heliInfo.altitudeSetpoint < 10) ? 10 : heliInfo.altitudeSetpoint;
        
        motorControl_setAltitudeSetpoint(heliInfo.altitudeSetpoint);
    }
    
    // Check yaw
    if (checkButton(LEFT) == PUSHED) {
        heliInfo.yawSetpoint -= 150;
        heliInfo.yawSetpoint = (heliInfo.yawSetpoint <= -1800) ? heliInfo.yawSetpoint + 3600 : heliInfo.yawSetpoint;
        motorControl_setYawSetpoint(heliInfo.yawSetpoint);
    } else if (checkButton(RIGHT) == PUSHED) {
        heliInfo.yawSetpoint += 150;
        heliInfo.yawSetpoint = (heliInfo.yawSetpoint > 1800) ? heliInfo.yawSetpoint - 3600 : heliInfo.yawSetpoint;
        motorControl_setYawSetpoint(heliInfo.yawSetpoint);
    }
}

// ===================================== Main =====================================
/**
 * @brief Main function for the helicopter control project
 * 
 * @return int 
 */
int main(void) {
    // ========================= Initialise =========================
    initButtons();
    switch_init();
    clock_init();
    serialUART_init();
    altitude_init(CIRC_BUFFER_SIZE);
    display_init ();
    yaw_init ();
    motorControl_init();
    softReset_init();

    // Enable interrupts to the processor.
    IntMasterEnable();
    
    SysCtlDelay(16000000); // Delay for 2s

    // Setup to start the program
    altitude_setMinimumAltitude(); // Set the minimum altitude to the current altitude
    
    motorControl_setAltitudeSetpoint(0);
    motorControl_setYawSetpoint(0);
    altitudeSetpoint = 0;
    yawSetpoint = 0;

    // Clean switch
    switch_update();
    switch_update();
    switch_update();
    switch_check(SW1);

    heliInfo.mode = 0;

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
        }

        // update ui
        switch_update();
        updateButtons(); 
        
        // Display Altitude and yaw
        main_display(&heliInfo);

        // check for soft reset
        if (GPIOPinRead(SOFT_RESET_PORT, SOFT_RESET_PIN) != SOFT_RESET_PIN) {
            // SysCtlReset();
        }

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
            heli_takeoff();
            break;
        
        case FLYING:
            if (switch_check(SW1) == SWITCH_DOWN) {
                heliInfo.mode = LANDING;
            }

            updateSetpoints();
            break;
        
        case LANDING:
            heli_land();
            break;        
        }
    }
}
