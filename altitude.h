/**
 * @file altitude.h
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief read the altitude of the helicopter and store it in a circular buffer
 * @date 2023-03-12
 * 
*/

#ifndef ALTITUDE_H
#define ALTITUDE_H

// ========================= Include files =========================
#include <stdint.h>

// ========================= Function Prototypes =========================
/**
 * @brief initilise the ADC and the circular buffer
 * @cite ADCDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 * @param buffSize size of the circular buffer
*/
void altitude_init(uint16_t buffSize);

/**
 * @brief get the average altitude of the helicopter from the circular buffer
 *
 * @return uint8_t average altitude (0-100)
 */
uint16_t altitude_get(void);

/**
 * @brief get the average ADC value of the helicopter from the circular buffer
 * 
 * @return uint16_t average ADC value (0-4096)
*/
uint32_t altitude_getRaw(void);

/**
 * @brief get the number of samples that have been taken
 * 
 * @return uint16_t number of samples
 */
uint16_t altitude_getSamples(void);

/**
 * @brief Trigger the ADC to read the altitude of the helicopter
 * 
*/
void altitude_read(void);

/**
 * @brief Reset the circular buffer to 0% altitude
 * 
 */
void altitude_reset(void);

#endif // ALTITUDE_H
