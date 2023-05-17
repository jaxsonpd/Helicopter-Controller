/**
 * @file switch.c
 * @author Daniel Hawes ()
 * @brief switch handling for the helicopter project
 * @date 2023-05-10
 *
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
void switch_init(void);

/**
 * @brief Updates the switches (Debounce)
 * 
 */
void switch_update(void);

/**
 * @brief Return the switch state
 * 
 * @param switchName the switch to get the state of
 * @return the current switch state
 */
uint8_t switch_check (uint8_t switchName);

#endif // SWITCH_T
