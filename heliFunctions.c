/** 
 * @file heliFunctions.c
 * @brief Functions for the helicopter control project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-18
 */


// ===================================== Includes =====================================
#include <stdint.h>

#include "heliFunctions.h"
#include "MotorControl.h"
#include "altitude.h"
#include "yaw.h"
#include "main.h"
#include "buttons4.h"

// ===================================== Constants ====================================
enum TAKE_OFF_STATE {TAKEOFF_START, TAKE_OFF_ROTATE, TAKE_OFF_RISING, TAKE_OFF_DONE};
enum LANDING_STATE {LANDING_START, LANDING_ROTATE, LANDING_DESCENDING, LANDING_DONE};

#define ROTATE_SPEED 150
#define LIFT_SPEED 10
#define LANDING_SPEED 5

#define MAX_ALTITUDE 100
#define MIN_ALTITUDE 10

#define MAX_YAW 1800
#define MIN_YAW -1800
#define ONE_REV 3600
#define UPPER_YAW_BOUND 24
#define LOWER_YAW_BOUND -24
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

        heliInfo->altitudeSetpoint = 0;
        heliInfo->yawSetpoint = 0;
        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        motorControl_setYawSetpoint(heliInfo->yaw);

        motorControl_enable(TAIL_MOTOR);
        motorControl_enable(MAIN_MOTOR);

        heliInfo->mainMotorRamped = motorControl_rampUpMainRotor();
        

        if (heliInfo->mainMotorRamped) {
            takeOffState = TAKE_OFF_ROTATE;
        }
        break;

    case TAKE_OFF_ROTATE:
        if (yaw_getRef() == 0) {
            yaw_reset();
            
            heliInfo->yawSetpoint = 0;
            motorControl_setYawSetpoint(heliInfo->yawSetpoint);

            takeOffState = TAKE_OFF_RISING;
        } else {
            // Rotate the helicopter to face the reference
            heliInfo->yawSetpoint = yaw_get() + ROTATE_SPEED;
            heliInfo->yawSetpoint = (heliInfo->yawSetpoint > MAX_YAW) ? heliInfo->yawSetpoint - ONE_REV : heliInfo->yawSetpoint;

            motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        }
        break;
    
    case TAKE_OFF_RISING:
        takeOffState = TAKE_OFF_DONE;
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
        if (altitude_get() <= MIN_ALTITUDE) {
            landingState = LANDING_ROTATE;
        }

        heliInfo->altitudeSetpoint = (altitude_get() - LANDING_SPEED);
        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        break;

    case LANDING_ROTATE:
        if (yaw_get() <= UPPER_YAW_BOUND && yaw_get() >= LOWER_YAW_BOUND) {
            referenceTimer += 1;
            if (referenceTimer > 10) {
                referenceTimer = 0;
                landingState = LANDING_DESCENDING;
            }
        } else {
            referenceTimer = 0;	
        }

        heliInfo->yawSetpoint = 0;
        heliInfo->altitudeSetpoint = MIN_ALTITUDE;
        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        motorControl_setYawSetpoint(heliInfo->yawSetpoint);
        break;

    case LANDING_DESCENDING:
        if (altitude_get() < 1) {
            landingState = LANDING_DONE;
        }

        heliInfo->altitudeSetpoint = 0;
        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
        break;

    case LANDING_DONE:
        motorControl_disable(MAIN_MOTOR);
        motorControl_disable(TAIL_MOTOR);

        landingState = LANDING_START;
        
        heliInfo->mainMotorRamped = false; // reset the motor ramped flag
        
        heliInfo->mode = LANDED;

        break;
    }
}

/**
 * @brief Update the helicopter setpoints while flying
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_updateSetpoints(heliInfo_t *heliInfo) {
        // Check altitude setpoint
    if (checkButton(UP) == PUSHED) {
        heliInfo->altitudeSetpoint += LIFT_SPEED;
        heliInfo->altitudeSetpoint = (heliInfo->altitudeSetpoint > MAX_ALTITUDE) ? MAX_ALTITUDE : heliInfo->altitudeSetpoint;

        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
    } else if (checkButton(DOWN) == PUSHED) {
        heliInfo->altitudeSetpoint -= LIFT_SPEED;
        heliInfo->altitudeSetpoint = (heliInfo->altitudeSetpoint < MIN_ALTITUDE) ? MIN_ALTITUDE : heliInfo->altitudeSetpoint;

        motorControl_setAltitudeSetpoint(heliInfo->altitudeSetpoint);
    }
    
    // Check yaw
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