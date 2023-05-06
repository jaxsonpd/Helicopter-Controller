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


// ===================================== Globals ======================================


// ===================================== Function Prototypes ==========================
/** 
 * @brief setup the pwm moduals for the main and tail rotors
 * 
 */
void PWM_init(void);

/** 
 * @brief set the duty cycle of the main rotor
 * @param duty_cycle the duty cycle to set the main rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
uint8_t PWM_setMainRotorDuty(uint8_t duty_cycle);

/** 
 * @brief set the duty cycle of the tail rotor
 * @param duty_cycle the duty cycle to set the tail rotor 
 * 
 * @return uint8_t 0 if the duty cycle is out of range, 1 otherwise
 */
uint8_t PWM_setTailRotorDuty(uint8_t duty_cycle);

/** 
 * @brief Return the current duty cycle of the main rotor
 * 
 * @return uint8_t duty cycle of the main rotor
 */
uint8_t PWM_getMainRotorDuty(void);

/** 
 * @brief Return the current duty cycle of the tail rotor
 * 
 * @return uint8_t duty cycle of the tail rotor
 */
uint8_t PWM_getTailRotorDuty(void);

/**
 * @brief Enable the PWM moduals
 * 
 */
void PWM_enable(void);

/**
 * @brief Disable the PWM moduals
 * 
 */
void PWM_disable(void);

#endif // PWM_H