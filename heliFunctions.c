/** 
 * @file heliFunctions.c
 * @brief Functions for the helicopter control project
 * @author Jack Duignan (jdu80@uclive.ac.nz), Daniel Hawes (dha144@uclive.ac.nz)
 * @date 2023-05-18
 */


// ===================================== Includes =====================================
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "heliFunctions.h"
#include "MotorControl.h"
#include "altitude.h"
#include "yaw.h"
#include "main.h"
#include "buttons4.h"

// ===================================== Constants ====================================
enum TAKE_OFF_STATE {TAKEOFF_START, TAKEOFF_RISING, TAKE_OFF_ROTATE, TAKE_OFF_DONE};
enum LANDING_STATE {LANDING_START, LANDING_ROTATE, LANDING_DESCENDING, LANDING_DONE};

#define ROTATE_SPEED 150
#define LIFT_SPEED 10
#define LANDING_SPEED 5

#define MAX_ALTITUDE 100
#define MIN_ALTITUDE 10
#define MIN_LANDING_ALTITUDE 5

#define MAX_YAW 1800
#define MIN_YAW -1800
#define ONE_REV 3600
#define UPPER_YAW_BOUND 8
#define LOWER_YAW_BOUND -8
#define YAW_LANDING_TIMER_COUNT 10

// ===================================== Globals ======================================


// ===================================== Function Definitions =========================
/**
 * @brief Take off the helicopter
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_takeoff(heliInfo_t *heliInfo) {
    static uint8_t takeOffState = 0;

    switch (takeOffState) {
    case TAKEOFF_START:
        // Reset the setpoints and enable the motors
        heliInfo->altitudeSetpoint = 0;
        heliInfo->yawSetpoint = 0;

        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        motorControl_setYawSetpoint(heliInfo->yawSetpoint);

        motorControl_enable(TAIL_MOTOR);
        motorControl_enable(MAIN_MOTOR);

        takeOffState = TAKEOFF_RISING;

    case TAKEOFF_RISING:
        // Ramp up the main rotor to hover speed
        heliInfo->mainMotorRamped = motorControl_rampUpMainRotor();

        if (heliInfo->mainMotorRamped) { 
            takeOffState = TAKE_OFF_ROTATE;
        }
        break;

    case TAKE_OFF_ROTATE:
        // Find the reference for the yaw
        if (yaw_getRef() == 0) { // At the reference
            yaw_reset();
            
            heliInfo->yawRefFound = true;
            heliInfo->yawSetpoint = 0;
            motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        } else {
            // Rotate the helicopter to face the reference
            heliInfo->yawSetpoint = yaw_get() + ROTATE_SPEED;
            heliInfo->yawSetpoint = (heliInfo->yawSetpoint > MAX_YAW) ? heliInfo->yawSetpoint - ONE_REV : heliInfo->yawSetpoint;

            motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        }

        if (heliInfo->yawRefFound) {
            takeOffState = TAKE_OFF_DONE;
        }
        break;
    
    case TAKE_OFF_DONE:
        takeOffState = TAKEOFF_START;
        heliInfo->mode = FLYING;
        break;
    }
}


/**
 * @brief Land the helicopter
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_land(heliInfo_t *heliInfo) {
    static uint8_t landingState = 0;
    static uint16_t referenceTimer = 0;

    switch (landingState) {
    case LANDING_START:
        // Lower the helicopter to just above the ground
        if (altitude_get() <= MIN_LANDING_ALTITUDE) {
            landingState = LANDING_ROTATE;
        } else {
            heliInfo->altitudeSetpoint = (altitude_get() - LANDING_SPEED); // Lower the altitude slowly
            heliInfo->altitudeSetpoint = (heliInfo->altitudeSetpoint < MIN_LANDING_ALTITUDE) ? MIN_LANDING_ALTITUDE : heliInfo->altitudeSetpoint;

            motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        }
        break;

    case LANDING_ROTATE:
        // Rotate the helicopter to face the reference 
        if (yaw_get() <= UPPER_YAW_BOUND && yaw_get() >= LOWER_YAW_BOUND) {
            referenceTimer += 1;
            if (referenceTimer > YAW_LANDING_TIMER_COUNT) { // Make sure the helicopter is actually at the reference
                referenceTimer = 0;
                landingState = LANDING_DESCENDING;
            }
        } else {
            referenceTimer = 0;	
            heliInfo->yawSetpoint = 0;
            heliInfo->altitudeSetpoint = MIN_LANDING_ALTITUDE;

            motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
            motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        }
        break;

    case LANDING_DESCENDING:
        // Lower the helicopter to the ground
        if (altitude_get() < 1) {
            landingState = LANDING_DONE;
        } else {
            heliInfo->altitudeSetpoint = 0;
            heliInfo->yawSetpoint = 0;

            motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
            motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        }

        break;

    case LANDING_DONE:
        motorControl_disable(MAIN_MOTOR);
        motorControl_disable(TAIL_MOTOR);

        // Reset the FSMs
        landingState = LANDING_START;        
        heliInfo->mode = LANDED;
        heliInfo->mainMotorRamped = false;
        heliInfo->yawRefFound = false;

        break;
    }
}


/**
 * @brief Update the helicopter setpoints while flying
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_updateSetpoints(heliInfo_t *heliInfo) {
    // Check altitude setpoint and bound it if needed
    if (checkButton(UP) == PUSHED) {
        heliInfo->altitudeSetpoint += LIFT_SPEED;
        heliInfo->altitudeSetpoint = (heliInfo->altitudeSetpoint > MAX_ALTITUDE) ? MAX_ALTITUDE : heliInfo->altitudeSetpoint;

        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
    } else if (checkButton(DOWN) == PUSHED) {
        heliInfo->altitudeSetpoint -= LIFT_SPEED;
        heliInfo->altitudeSetpoint = (heliInfo->altitudeSetpoint < MIN_ALTITUDE) ? MIN_ALTITUDE : heliInfo->altitudeSetpoint;

        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
    }
    
    // Check yaw and bound it if needed
    if (checkButton(LEFT) == PUSHED) {
        heliInfo->yawSetpoint -= ROTATE_SPEED;
        heliInfo->yawSetpoint = (heliInfo->yawSetpoint <= MIN_YAW) ? heliInfo->yawSetpoint + ONE_REV : heliInfo->yawSetpoint;

        motorControl_setYawSetpoint(heliInfo->yawSetpoint);
    } else if (checkButton(RIGHT) == PUSHED) {
        heliInfo->yawSetpoint += ROTATE_SPEED;
        heliInfo->yawSetpoint = (heliInfo->yawSetpoint > MAX_YAW) ? heliInfo->yawSetpoint - ONE_REV : heliInfo->yawSetpoint;

        motorControl_setYawSetpoint(heliInfo->yawSetpoint);
    }
}
