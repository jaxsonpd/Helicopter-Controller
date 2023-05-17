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

#include "utils/ustdlib.h"

#include "OrbitOLED/OrbitOLEDInterface.h"

#include "deviceInfo.h"
#include "display.h"
// ========================= Constants and types =========================


// ========================= Global Variables =========================


// ========================= Function Definition =========================
/**
 * @brief Enables GPIO pins for OLEF Peripheral
 *
 */
void display_init (void) {
    // Initalise the Orbit OLED display
    OLEDInitialise ();
}


/**
 * @brief Draws to the OLED Display the Yaw and Altitude and motor percentages
 * @cite OLEDTest.c from the lab 3 folder author: P.J. Bones UCECE
 *
 * @param deviceInfo The struct containing the device information
 * 
*/
void main_display (deviceInfo_t *deviceInfo) {
    char string1[17];
    char string2[17];
    char string3[17];
    char string4[17];

    // Convert yaw to degrees
    int32_t degrees = deviceInfo->yaw / 10;
    int32_t decimalDegrees = (deviceInfo->yaw < 0) ? deviceInfo->yaw % 10 * -1 : deviceInfo->yaw % 10; // Remove negitive sign on decimal portion

    // Print to OLED
    usnprintf(string1, sizeof(string1), "   YAW:  %4d.%1d   ", degrees, decimalDegrees);
    OLEDStringDraw (string1, 0, 0);
    usnprintf(string2, sizeof(string2), "   ALT:    %3d%%   ", deviceInfo->altitude);
    OLEDStringDraw (string2, 0, 1);
    usnprintf(string3, sizeof(string3), "MOTOR1:    %3d%%   ", deviceInfo->mainMotorDuty);
    OLEDStringDraw (string3, 0, 2);
    usnprintf(string4, sizeof(string4), "MOTOR2:    %3d%%   ", deviceInfo->tailMotorDuty);
    OLEDStringDraw (string4, 0, 3);
}
