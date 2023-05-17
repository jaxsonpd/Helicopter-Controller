/**
 * @file switch.c
 * @author Daniel Hawes ()
 * @brief switch handling for the helicopter project
 * @date 2023-05-10
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"

#include "switch.h"

// ============================ Constants ====================================

// Switch 1 hardware configuration
#define SW1_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define SW1_GPIO_BASE      GPIO_PORTA_BASE
#define SW1_GPIO_PIN       GPIO_PIN_7
#define SW1_NORMAL         false

#define NUM_SWITCH_POLLS 3

// ============================ Globals ======================================
static bool switch_state[NUM_SWITCHES];         // Current state of the switchs
static uint8_t switch_count[NUM_SWITCHES];      // Counter for debouncing
static uint8_t switch_flag[NUM_SWITCHES];       // Flag for when the switch state changes
static bool switch_normal[NUM_SWITCHES];        // Normal state of the switchs
static bool switch_value[NUM_SWITCHES];         // Current value of the switchs

/**
 * @brief Initialises the switchs
 * 
 */
void switch_init(void) {
    // Switch 1
    SysCtlPeripheralEnable (SW1_PERIPH_GPIO);
    GPIOPinTypeGPIOInput (SW1_GPIO_BASE, SW1_GPIO_PIN);
    GPIOPadConfigSet (SW1_GPIO_BASE, SW1_GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    switch_normal[SW1] = SW1_NORMAL;
    switch_state[SW1] = SW1_NORMAL;
    switch_count[SW1] = 0;
    switch_flag[SW1] = false;

}


/**
 * @brief Updates the switches (Debounce)
 * 
 */
void switch_update(void) {
    // Switch 1
    switch_value[SW1] = (GPIOPinRead (SW1_GPIO_BASE, SW1_GPIO_PIN) == SW1_GPIO_PIN);

    if (switch_value[SW1] != switch_state[SW1]) {
        switch_count[SW1]++;
        if (switch_count[SW1] >= NUM_SWITCH_POLLS) {
            switch_state[SW1] = switch_value[SW1];
            switch_flag[SW1] = true;
            switch_count[SW1] = 0;
        }
    } else {
        switch_count[SW1] = 0;
    }
}

/**
 * @brief Return the switch state
 * 
 * @param switchName the switch to get the state of
 * @return the current switch state
 */
uint8_t switch_check (uint8_t switchName) {
    if (switch_flag[switchName]) { // If the switch state has changed
            switch_flag[switchName] = false;

            if (switch_state[switchName] == switch_normal[switchName]) {
                return SWITCH_DOWN;
            } else {
                return SWITCH_UP;
            }
        }
    return SWITCH_NO_CHANGE;
}


