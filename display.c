/*
 * display.c
 *
 *  Created on: 16/03/2023
 *      Author: Daniel Hawes
 *      Student Code: dha144
 */
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

void initDisplay (void) {
    // Initalise the Orbit OLED display
    OLEDInitialise ();
}

void displayPercentage (uint16_t percentage) {

    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Percentage      ", 0, 0);

    usnprintf (string, sizeof(string), "%3d%%           ", percentage);

    OLEDStringDraw (string, 0, 1);
}

void displayADC (uint32_t meanADC) {

    char string[17];

    OLEDStringDraw ("Mean ADC        ", 0, 0);

    usnprintf (string, sizeof(string), "%4d            ", meanADC);

    OLEDStringDraw (string, 0, 1);

}

void displayNothing (void) {

    OLEDStringDraw ("                ", 0, 0);

    OLEDStringDraw ("                ", 0, 1);

    OLEDStringDraw ("                ", 0, 2);

    OLEDStringDraw ("                ", 0, 3);
}

