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

#include "main.h"

// ========================= Function Prototypes =========================

/**
 * @brief Enables GPIO pins for OLEF Peripheral
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 */
void display_init(void);

/**
 * @brief Draws to the OLED Display the Yaw and Altitude and motor percentages
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 * @param deviceInfo The struct containing the device information
 * 
*/
void main_display (heliInfo_t *deviceInfo);

#endif /* DISPLAY_H_ */
