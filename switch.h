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
enum switchNames {SW1 = 0, NUM_SWITCHES};
enum switchStates {SWITCH_DOWN = 0, SWITCH_UP, SWITCH_NO_CHANGE};

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
uint8_t checkSwitch (uint8_t switchName);

#endif // SWITCH_T