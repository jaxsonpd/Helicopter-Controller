/**
 * @file pwm.h
 * @brief Header file for pwm.c
 * @author Jack Duignan (Jdu80@uclive.ac.nz)
 * @date 2023-05-06
*/


#ifndef PWM_H
#define PWM_H


// ===================================== Includes =====================================
#include <stdint.h>

// ===================================== Constants ====================================
#define ENABLE_DUTY 101
#define DISABLE_DUTY 0

// ===================================== Globals ======================================

// ===================================== Function Prototypes ==========================
/** 
 * @brief setup the pwm moduals for the main and tail rotors
 * 
 */
void PWM_init(void);


/**
 * @brief set the PWM parameters
 * @param duty the duty cycle of the PWM
 * @param motor the motor to set the PWM on
 *
 */
void PWM_set (uint8_t duty, uint8_t motor);


#endif // PWM_H
