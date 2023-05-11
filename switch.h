/** 
 * @file switch.h
 * @brief Header file for switch.c
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-11
 */


#ifndef SWITCH_T
#define SWITCH_T


// ===================================== Includes =====================================
#include <stdint.h>

// ===================================== Constants ====================================
enum switchNames {SW1 = 1, SW2, NUM_SWITCHES};
enum switchStates {RELEASED = 0, PUSHED, NO_CHANGE};

// ===================================== Function Prototypes ==========================
/**
 * @brief Initialises the switchs
 * 
 */
void initSwitch(void);

/**
 * @brief Updates the switches (Debounce)
 * 
 */
void updateSwitches(void);

/**
 * @brief Return the switch state
 * 
 * @param switchName 
 * @return uint8_t 
 */

#endif // SWITCH_T