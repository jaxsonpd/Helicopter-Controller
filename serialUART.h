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

// ========================= Function Prototypes =========================
/**
 * @brief Initializes the UART for sending and receiving data
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 */
void serialUART_init();

/**
 * @brief Sends a string of data over UART
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 * @param charBuffer The string of data to be sent
 */
void serialUART_SendInformation(char *charBuffer);

#endif /* SERIALUART_H */
