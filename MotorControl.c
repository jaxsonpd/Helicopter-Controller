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
#include "main.h"

// ===================================== Constants ====================================
// Define controller gains
#define MAIN_P_GAIN 70
#define MAIN_I_GAIN 10
#define MAIN_D_GAIN 0

#define TAIL_P_GAIN 145
#define TAIL_I_GAIN 4
#define TAIL_D_GAIN 0
#define TAIL_CONSTANT 41

// Max and min duty cycles for each motor
#define MAX_MAIN_DUTY 80
#define MAX_TAIL_DUTY 70
#define MIN_MAIN_DUTY 1
#define MIN_TAIL_DUTY 1

// Scale factors
#define S_TO_MS 1000

#define MAIN_MOTOR_SCALE 100
#define TAIL_MOTOR_SCALE 100

#define RAMP_TIMER 10 // The number of ticks to wait between ramps of the motor

// Yaw error bounds
#define MAX_YAW_ERROR 1800
#define MIN_YAW_ERROR -1800
#define YAW_ERROR_OFFSET 3600

// Alititude error bounds
#define MIN_ALTITUDE_ERROR 0

// Absolute max and min duty cycles
#define ABS_MAX_DUTY 100
#define ABS_MIN_DUTY 1

// ===================================== Globals ======================================
static uint8_t altSetpoint = 0; // The setpoint for the main rotor
static int16_t yawSetpoint = 0; // The setpoint for the tail rotor

static uint8_t mainRotorDuty = 0;
static uint8_t tailRotorDuty = 0;

static uint8_t mainConstant = 0;

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
 * @return 1 if the duty cycle is out of range, 0 otherwise
 */
static uint8_t motorControl_setMainRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > ABS_MAX_DUTY) {
        return 1; // Out of range
    } else if (duty_cycle < ABS_MIN_DUTY) {
        return 1; // Out of range
    }

    // Set the duty cycle
    mainRotorDuty = duty_cycle;
    PWM_set(duty_cycle, MAIN_MOTOR);
    return 0;
}


/** 
 * @brief set the duty cycle of the tail rotor
 * @param duty_cycle the duty cycle to set the tail rotor 
 * 
 * @return 1 if the duty cycle is out of range, 0 otherwise
 */
static uint8_t motorControl_setTailRotorDuty(uint8_t duty_cycle) {
    // Check range
    if (duty_cycle > ABS_MAX_DUTY) {
        return 1; // Out of range
    } else if (duty_cycle < ABS_MIN_DUTY) {
        return 1; // Out of range
    }

    // Set the duty cycle
    tailRotorDuty = duty_cycle;
    PWM_set(duty_cycle, TAIL_MOTOR);
    return 0;
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
    if (currentAltitude < MIN_ALTITUDE_ERROR) { 
        currentAltitude = MIN_ALTITUDE_ERROR;
    }
    
    // Calculate errors
    altError = altSetpoint - currentAltitude;
    altErrorIntergrated += altError * deltaT;
    altErrorDerivative = (altError - altErrorPrevious) / deltaT;

    // if the error has changed sign reset the intergral
    if (altError > 0 && altErrorIntergrated < 0 || altError < 0 && altErrorIntergrated > 0) { 
        altErrorIntergrated = 0;
    }
    
    // Convert to Duty cycle (Divide by 1000 for ms -> s)
    mainRotorDuty = (MAIN_P_GAIN * altError) 
                    + ((MAIN_I_GAIN * altErrorIntergrated) / S_TO_MS)
                    + ((MAIN_D_GAIN * altErrorDerivative) / S_TO_MS); 

    // Scale to allow for more fine tuning
    mainRotorDuty = mainRotorDuty / MAIN_MOTOR_SCALE + (mainConstant);

    // Limit the duty cycle to 1-100%
    if (mainRotorDuty > MAX_MAIN_DUTY) {
        mainRotorDuty = MAX_MAIN_DUTY;
    } else if (mainRotorDuty < MIN_MAIN_DUTY) {
        mainRotorDuty = MIN_MAIN_DUTY;
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

        // Ensure that the error is within bounds
    if (yawError >= MAX_YAW_ERROR) {
        yawError -= YAW_ERROR_OFFSET;
    } else if (yawError < MIN_YAW_ERROR) {
        yawError += YAW_ERROR_OFFSET;
    }

    yawErrorDerivative = (yawError - yawErrorPrevious) / deltaT;
    yawErrorIntergrated += yawError * deltaT;

    // If the error has changed sign reset the intergral
    if (yawError > 0 && yawErrorIntergrated < 0 || yawError < 0 && yawErrorIntergrated > 0) { 
        yawErrorIntergrated = 0;
    }

    // Convert to duty cycle (Divide by 1000 for ms -> s and by 10 for degrees * 10 -> degrees)
    tailRotorDuty = ((TAIL_P_GAIN * yawError) / YAW_DEGREES_SCALE)
                    + (((TAIL_I_GAIN * yawErrorIntergrated) / S_TO_MS) / YAW_DEGREES_SCALE) 
                    + (((TAIL_D_GAIN * yawErrorDerivative) / S_TO_MS) / YAW_DEGREES_SCALE);

    // Scale to allow for more fine tuning
    tailRotorDuty = tailRotorDuty / TAIL_MOTOR_SCALE + (TAIL_CONSTANT);
    
    // Limit the duty cycle to 1-100%
    if (tailRotorDuty > MAX_TAIL_DUTY) {
        tailRotorDuty = MAX_TAIL_DUTY;
    } else if (tailRotorDuty < MIN_TAIL_DUTY) {
        tailRotorDuty = MIN_TAIL_DUTY;
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

/**
 * @brief Ramp up the main rotor to find the hover point
 * 
 * @return true if the main rotor is at the hover point
 */
bool motorControl_rampUpMainRotor(void) {
    static uint8_t currentDuty = 0;
    static uint8_t timer = 0;
    
    if (altitude_get() > 0) {
        mainConstant = currentDuty;    
        return true;
    } else {
        // Ramp up the duty cycle
        if (timer == 0) {
            currentDuty++;
            motorControl_setMainRotorDuty(currentDuty);
            timer = RAMP_TIMER;
        } else {
            timer--;
        }
    }

    return false;

}
