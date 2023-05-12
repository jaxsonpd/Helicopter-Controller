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
#define MAIN_P_GAIN 40
#define MAIN_I_GAIN 2
#define MAIN_D_GAIN 0
#define MAIN_CONSTANT 40

#define TAIL_P_GAIN 50
#define TAIL_I_GAIN 3
#define TAIL_D_GAIN 0
#define TAIL_CONSTANT 35

#define MAX_MAIN_DUTY 80
#define MAX_TAIL_DUTY 70

// ===================================== Globals ======================================
static uint8_t altSetpoint = 0; // The setpoint for the main rotor
static int16_t yawSetpoint = 0; // The setpoint for the tail rotor

static uint8_t mainRotorDuty = 0;
static uint8_t tailRotorDuty = 0;

static bool mainRotorEnabled = false;
static bool tailRotorEnabled = false;

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
    // Update the altitude controller
    static int32_t altErrorIntergrated = 0;
    static int16_t altErrorPrevious = 0;
    
    int16_t currentAltitude = 0;
    int16_t altError = 0;
    int32_t altErrorDerivative = 0;
    int32_t mainRotorDuty = 0;

    // Clean up the altitude
    currentAltitude = altitude_get();
    if (currentAltitude < 0) { 
        currentAltitude = 0;
    }
    
    // Calculate errors
    altError = altSetpoint - currentAltitude;

    altErrorIntergrated += altError * deltaT;
    altErrorDerivative = (altError - altErrorPrevious) / deltaT;

    // Convert to Duty cycle (Divide by 1000 for ms -> s)
    mainRotorDuty = (MAIN_P_GAIN * altError) + ((MAIN_I_GAIN * altErrorIntergrated) / 1000)
                    + ((MAIN_D_GAIN * altErrorDerivative) / 1000); 

    // Scale to allow for more fine tuning
    mainRotorDuty = mainRotorDuty / 10 + (MAIN_CONSTANT);

    // Limit the duty cycle to 1-100%
    if (mainRotorDuty > MAX_MAIN_DUTY) {
        mainRotorDuty = MAX_MAIN_DUTY;
    } else if (mainRotorDuty < 1) {
        mainRotorDuty = 1;
    }

    motorControl_setMainRotorDuty(mainRotorDuty);

    // Update the yaw controller
    static int32_t yawErrorIntergrated = 0;
    static int16_t yawErrorPrevious = 0;

    int16_t yawError = 0;
    int32_t yawErrorDerivative = 0;
    int32_t tailRotorDuty = 0;
    
    // Calculate errors
    yawError = yawSetpoint - yaw_get();

    yawErrorIntergrated += yawError * deltaT;
    yawErrorDerivative = (yawError - yawErrorPrevious) / deltaT;

    // Convert to duty cycle (Divide by 1000 for ms -> s and by 10 for degrees * 10 -> degrees)
    tailRotorDuty = ((TAIL_P_GAIN * yawError) / 10) + (((TAIL_I_GAIN * yawErrorIntergrated) / 1000) / 10) 
                    + (((TAIL_D_GAIN * yawErrorDerivative) / 1000) / 10);

    // Scale to allow for more fine tuning
    tailRotorDuty = tailRotorDuty / 10 + (TAIL_CONSTANT);
    
    // Limit the duty cycle to 1-100%
    if (tailRotorDuty > MAX_TAIL_DUTY) {
        tailRotorDuty = MAX_TAIL_DUTY;
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
