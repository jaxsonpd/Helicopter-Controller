/** 
 * @file MotorControl.h
 * @brief Header file for MotorControl.c
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-06
 */


#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H


// ===================================== Includes =====================================
#include <stdint.h>

// ===================================== Constants ====================================
enum MOTORCONTORL_MOTOR {MAIN_MOTOR, TAIL_MOTOR};

// ===================================== Globals ======================================


// ===================================== Function Prototypes ==========================
/** 
 * @brief initilise the motor control module
 * 
 */
void motorControl_init(void);


/**
 * @brief update the controller error etc
 * @param deltaT the time since the last update
 * 
 */
void motorControl_update(uint32_t deltaT);


/**
 * @brief Change the altitude setpoint
 * 
 * @param setpoint the new altitude setpoint
 */
void motorControl_setAltitudeSetpoint(uint32_t setpoint);


/**
 * @brief Change the yaw setpoint
 * @param setpoint the new yaw setpoint
 * 
 */
void motorControl_setYawSetpoint(uint32_t setpoint);


/**
 * @brief Disable the motors
 * @param motor the motor to disable
 */
void motorControl_disable(uint8_t motor);


/**
 * @brief Enable the motors
 * @param motor the motor to Enable
 */
void motorControl_enable(uint8_t motor);


/** 
 * @brief Return the current duty cycle of the main rotor
 * 
 * @return duty cycle of the main rotor
 */
uint8_t motorControl_getMainRotorDuty(void);


/** 
 * @brief Return the current duty cycle of the tail rotor
 * 
 * @return duty cycle of the tail rotor
 */
uint8_t motorControl_getTailRotorDuty(void);


#endif // MOTORCONTROL_H 
