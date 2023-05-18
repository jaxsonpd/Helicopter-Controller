/**
 * @file SerialUART.h
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief Small module to send and receive data over UART for debugging
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * @date 2023-03-12
 */

#ifndef SERIALUART_H
#define SERIALUART_H

// ========================= Include files =========================
#include <stdint.h>

#include "main.h"

// ========================= Function Prototypes =========================
/**
 * @brief Initializes the UART for sending and receiving data
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 */
void serialUART_init();

/**
 * @brief Send the serial infromation 
 * @param deviceInfo The device information struct
 * 
 */
void serialUART_SendInformation(heliInfo_t *deviceInfo);

#endif /* SERIALUART_H */
