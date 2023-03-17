/**
 * @file display.h
 * @author Daniel Hawes (dha144@uclive.ac.nz)
 * @brief Take values from altitude as parameters to display on the OLED Display
 * @date 2023-03-16
 *
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

// ========================= Include files =========================
#include <stdint.h>

// ========================= Function Prototypes =========================

/**
 * @brief Enables GPIO pins for OLEF Peripheral
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 */
void initDisplay(void);


/**
 * @brief Draws to the OLED Display the percentage altitude
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 * @param percentage The percentage altitude taken from altitude_get() in altitude.c
*/
void displayPercentage (uint32_t percentage);


/**
 * @brief Draws to the OLED Display the mean ADC value
 *
 * @param meanADC The mean ADC value taken from altitude_getRaw() in altitude.c
*/
void displayADC (uint32_t meanADC);


/**
 * @brief Draws the OLED Display blank
 *
*/
void displayNothing (void);

#endif /* DISPLAY_H_ */
