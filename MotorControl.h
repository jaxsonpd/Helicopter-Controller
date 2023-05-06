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


// ===================================== Globals ======================================


// ===================================== Function Prototypes ==========================
/** 
 * @brief initilise the motor control module
 * 
 */
void MotorControl_init(void);

/**
 * @brief update the controller error etc
 * @param deltaT the time since the last update
 * 
 */
void MotorControl_update(uint32_t deltaT);

/**
 * @brief Change the altitude setpoint
 * 
 * @param setpoint the new altitude setpoint
 */
void MotorControl_setAltitudeSetpoint(uint32_t setpoint);

/**
 * @brief Change the yaw setpoint
 * z
 * @param setpoint the new yaw setpoint
 */
void MotorControl_setYawSetpoint(uint32_t setpoint);

/**
 * @brief Enable the control modual
 * 
 */
void MotorControl_enable(void);

/**
 * @brief Disable the control modual
 * 
 */
void MotorControl_disable(void);

#endif // MOTORCONTROL_H 