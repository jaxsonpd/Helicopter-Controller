/** 
 * @file MotorControl.c
 * @brief Motor control module for the helicopter control project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-06
 */


// ===================================== Includes =====================================
#include <stdint.h>


#include "MotorControl.h"
#include "pwm.h"
#include "altitude.h"
#include "yaw.h"

// ===================================== Constants ====================================
// Define controller gains
#define MAIN_P_GAIN 1
#define MAIN_I_GAIN 1
#define MAIN_D_GAIN 1
#define MAIN_CONSTANT 50

#define TAIL_P_GAIN 1
#define TAIL_I_GAIN 1
#define TAIL_D_GAIN 1
#define TAIL_CONSTANT 50

// ===================================== Globals ======================================
uint8_t altSetpoint = 0; // The setpoint for the main rotor
int16_t yawSetpoint = 0; // The setpoint for the tail rotor

// ===================================== Function Definitions =========================
/** 
 * @brief initilise the motor control module
 * 
 */
void MotorControl_init(void) {
    PWM_init();

    MotorControl_disable(); // Ensure that motors are disabled

    MotorControl_setAltitudeSetpoint(0);
    MotorControl_setYawSetpoint(0);
}


/**
 * @brief update the controller error etc
 * @param deltaT the time since the last update
 * 
 */
void MotorControl_update(uint32_t deltaT) {
    static int32_t altErrorIntergrated = 0;
    static int32_t yawErrorIntergrated = 0;

    static int16_t altErrorPrevious = 0;
    static int16_t yawErrorPrevious = 0;

    // Update the altitude controller
    int16_t altError = altSetpoint - Altitude_getAltitude();

    altErrorIntergrated += altError * deltaT;
    int32_t altErrorDerivative = (altError - altErrorPrevious) / deltaT;

    int32_t mainRotorDuty = MAIN_P_GAIN * altError + MAIN_I_GAIN * altErrorIntergrated 
                            + MAIN_D_GAIN * altErrorDerivative;
                            + MAIN_CONSTANT; 

    // Check the duty cycle is within range
    if (mainRotorDuty > 100) {
        mainRotorDuty = 100;
    } else if (mainRotorDuty < 1) {
        mainRotorDuty = 1;
    }

    PWM_setMainRotorDuty(mainRotorDuty);

    // Update the yaw controller
    int16_t yawError = yawSetpoint - Yaw_getYaw();

    yawErrorIntergrated += yawError * deltaT;
    int32_t yawErrorDerivative = (yawError - yawErrorPrevious) / deltaT;

    int32_t tailRotorDuty = TAIL_P_GAIN * yawError + TAIL_I_GAIN * yawErrorIntergrated 
                            + TAIL_D_GAIN * yawErrorDerivative;
                            + TAIL_CONSTANT;
    
    // Check the duty cycle is within range
    if (tailRotorDuty > 100) {
        tailRotorDuty = 100;
    } else if (tailRotorDuty < 1) {
        tailRotorDuty = 1;
    }

    PWM_setTailRotorDuty(tailRotorDuty);

    // Update the previous error
    altErrorPrevious = altError;
    yawErrorPrevious = yawError;
}

/**
 * @brief Change the altitude setpoint
 * 
 * @param setpoint the new altitude setpoint
 */
void MotorControl_setAltitudeSetpoint(uint32_t setpoint) {
    altSetpoint = setpoint;
}

/**
 * @brief Change the yaw setpoint
 * 
 * @param setpoint the new yaw setpoint
 */
void MotorControl_setYawSetpoint(uint32_t setpoint);

/**
 * @brief Enable the control modual
 * 
 */
void MotorControl_enable(void) {
    PWM_enable();
}

/**
 * @brief Disable the control modual
 * 
 */
void MotorControl_disable(void) {
    PWM_disable();
}