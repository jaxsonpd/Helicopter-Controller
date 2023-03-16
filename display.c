/**
 * @file display.c
 * @author Daniel Hawes (dha144@uclive.ac.nz)
 * @brief Take values from altitude as parameters to display on the OLED Display
 * @date 2023-03-16
 *
 */

// ========================= Include files =========================
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"

#include "OrbitOLED/OrbitOLEDInterface.h"

#include "debug.h"

#include "utils/ustdlib.h"
#include "stdio.h"

// ========================= Function Definition =========================

/**
 * @brief Enables GPIO pins for OLEF Peripheral
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 */
void initDisplay (void) {
    // Initalise the Orbit OLED display
    OLEDInitialise ();
}


/**
 * @brief Draws to the OLED Display the percentage altitude
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 * @param percentage The percentage altitude taken from altitude_get() in altitude.c
*/
void displayPercentage (uint16_t percentage) {

    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Percentage      ", 0, 0);

    usnprintf (string, sizeof(string), "%3d%%           ", percentage);

    OLEDStringDraw (string, 0, 1);
}

/**
 * @brief Draws to the OLED Display the mean ADC value
 *
 * @param meanADC The mean ADC value taken from altitude_getRaw() in altitude.c
*/
void displayADC (uint32_t meanADC) {

    char string[17];

    OLEDStringDraw ("Mean ADC        ", 0, 0);

    usnprintf (string, sizeof(string), "%4d            ", meanADC);

    OLEDStringDraw (string, 0, 1);

}

/**
 * @brief Draws the OLED Display blank
 *
 *
*/
void displayNothing (void) {

    OLEDStringDraw ("                ", 0, 0);

    OLEDStringDraw ("                ", 0, 1);

    OLEDStringDraw ("                ", 0, 2);

    OLEDStringDraw ("                ", 0, 3);
}

