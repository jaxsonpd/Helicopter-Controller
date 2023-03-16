/*
 * display.h
 *
 *  Created on: 16/03/2023
 *      Author: Daniel Hawes
 *      Student code: dha144
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>

void initDisplay(void);

void displayPercentage (uint16_t percentage);

void displayADC (uint32_t meanADC);

void displayNothing (void);

#endif /* DISPLAY_H_ */
