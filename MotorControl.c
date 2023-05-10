/** 
 * @file MotorControl.c
 * @brief Motor control module for the helicopter control project
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-06
 */


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

#include "MotorControl.h"
#include "pwm.h"
#include "altitude.h"
#include "yaw.h"

// ===================================== Constants ====================================
// Define controller gains
#define MAIN_P_GAIN 1
#define MAIN_I_GAIN 0
#define MAIN_D_GAIN 0
#define MAIN_CONSTANT 50

#define TAIL_P_GAIN 1
#define TAIL_I_GAIN 0
#define TAIL_D_GAIN 0
#define TAIL_CONSTANT 50

// ===================================== Globals ======================================
uint8_t altSetpoint = 0; // The setpoint for the main rotor
int16_t yawSetpoint = 0; // The setpoint for the tail rotor

uint8_t mainRotorDuty = 0;
uint8_t tailRotorDuty = 0;

bool mainRotorEnabled = false;
bool tailRotorEnabled = false;

// ===================================== Function Definitions =========================
/**
 * @brief Disable the motors
 * @param motor the motor to disable
 */
void motorControl_disable(uint8_t motor) {
    if (motor == MAIN_MOTOR) {
        mainRotorEnabled = false;
        PWM_disable(MAIN_MOTOR);
    } else if (motor == TAIL_MOTOR) {
        tailRotorEnabled = false;
        PWM_disable(TAIL_MOTOR);
    }
}

/**
 * @brief Enable the motors
 * @param motor the motor to Enable
 */
void motorControl_enable(uint8_t motor) {
    if (motor == MAIN_MOTOR) {
        mainRotorEnabled = true;
        PWM_enable(MAIN_MOTOR);
    } else if (motor == TAIL_MOTOR) {
        tailRotorEnabled = true;
        PWM_enable(TAIL_MOTOR);
    }
}

/** 
 * @brief set the duty cycle of the main rotor
 * @param duty_cycle the duty cycle to set the main rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
static uint8_t motorControl_setMainRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > 100) {
        return 0; // Out of range
    } else if (duty_cycle < 1) {
        return 0; // Out of range
    }

    // Set the duty cycle
    mainRotorDuty = duty_cycle;
    PWM_set(duty_cycle, MAIN_MOTOR);
    return 1;
}


/** 
 * @brief set the duty cycle of the tail rotor
 * @param duty_cycle the duty cycle to set the tail rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
static uint8_t motorControl_setTailRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > 100) {
        return 0; // Out of range
    } else if (duty_cycle < 1) {
        return 0; // Out of range
    }

    // Set the duty cycle
    tailRotorDuty = duty_cycle;
    PWM_set(duty_cycle, TAIL_MOTOR);
    return 1;
}


/**
 * @brief update the controller error etc
 * @param deltaT the time since the last update [ms]
 * 
 */
void motorControl_update(uint32_t deltaT) {
    // Supporting variables
    static int32_t altErrorIntergrated = 0;
    static int32_t yawErrorIntergrated = 0;

    static int16_t altErrorPrevious = 0;
    static int16_t yawErrorPrevious = 0;

    // Update the altitude controller
    int16_t currentAltitude = altitude_get();
    if (currentAltitude < 0) { // Clean up the altitude
        currentAltitude = 0;
    }
    
    int16_t altError = altSetpoint - currentAltitude;

    altErrorIntergrated += altError * deltaT;
    int32_t altErrorDerivative = (altError - altErrorPrevious) / deltaT;

    int32_t mainRotorDuty = (MAIN_P_GAIN * altError) + ((MAIN_I_GAIN * altErrorIntergrated) / 1000)
                            + ((MAIN_D_GAIN * altErrorDerivative) / 1000) + (MAIN_CONSTANT); 

    // Limit the duty cycle to 1-100%
    if (mainRotorDuty > 100) {
        mainRotorDuty = 100;
    } else if (mainRotorDuty < 1) {
        mainRotorDuty = 1;
    }

    motorControl_setMainRotorDuty(mainRotorDuty);

    // Update the yaw controller
    int16_t yawError = yawSetpoint - yaw_get();

    yawErrorIntergrated += yawError * deltaT;
    int32_t yawErrorDerivative = (yawError - yawErrorPrevious) / deltaT;

    int32_t tailRotorDuty = (TAIL_P_GAIN * yawError) + ((TAIL_I_GAIN * yawErrorIntergrated) / 1000) 
                            + ((TAIL_D_GAIN * yawErrorDerivative) / 1000) + (TAIL_CONSTANT);
    
    // Limit the duty cycle to 1-100%
    if (tailRotorDuty > 100) {
        tailRotorDuty = 100;
    } else if (tailRotorDuty < 1) {
        tailRotorDuty = 1;
    }

    motorControl_setTailRotorDuty(tailRotorDuty);

    // Update the previous error
    altErrorPrevious = altError;
    yawErrorPrevious = yawError;
}

/**
 * @brief Change the altitude setpoint
 * 
 * @param setpoint the new altitude setpoint
 */
void motorControl_setAltitudeSetpoint(uint32_t setpoint) {
    altSetpoint = setpoint;
}

/**
 * @brief Change the yaw setpoint
 * 
 * @param setpoint the new yaw setpoint
 */
void motorControl_setYawSetpoint(uint32_t setpoint) {
    yawSetpoint = setpoint;
}

/** 
 * @brief Return the current duty cycle of the main rotor
 * 
 * @return duty cycle of the main rotor
 */
uint8_t motorControl_getMainRotorDuty(void) {
    return (mainRotorEnabled) ? mainRotorDuty : 0;
}

/** 
 * @brief Return the current duty cycle of the tail rotor
 * 
 * @return duty cycle of the tail rotor
 */
uint8_t motorControl_getTailRotorDuty(void) {
    return (tailRotorEnabled) ? tailRotorDuty : 0;
}

/** 
 * @brief initilise the motor control module
 * 
 */
void motorControl_init(void) {
    PWM_init();

    // Ensure that motors are disabled
    motorControl_disable(MAIN_MOTOR); 
    motorControl_disable(TAIL_MOTOR);

    motorControl_setAltitudeSetpoint(0);
    motorControl_setYawSetpoint(0);
}
