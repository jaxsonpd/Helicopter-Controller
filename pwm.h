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


// ===================================== Function Prototypes ==========================
/** 
 * @brief setup the pwm moduals for the main and tail rotors
 * 
 */
void PWM_init(void);


/**
 * @brief set the PWM parameters
 * @param duty the duty cycle of the PWM (0 = off)
 * @param motor the motor to set the PWM on
 *
 */
void PWM_set (uint8_t duty, uint8_t motor);


/**
 * @brief Disable the give PWM signal
 * @param motor the motor to disable
 * 
 */
void PWM_disable(uint8_t motor);


/**
 * @brief Enable the give PWM signal
 * @param motor the motor to enable
 * 
 */
void PWM_enable(uint8_t motor);

#endif // PWM_H
