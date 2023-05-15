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
void display_init(void);

/**
 * @brief Draws to the OLED Display the Yaw and Altitude
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 * @param yaw The yaw taken from yaw_get() in yaw.c
 * @param altitude The altitude taken from altitude_get() in altitude.c
 * @param motor1 The percentage of motor 1
 * @param motor2 The percentage of motor 2 
*/
void main_display (int32_t yaw, int32_t altitude, int8_t motor1, int8_t motor2);

#endif /* DISPLAY_H_ */
