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
 * @brief Send the serial infromation 
 * @param desiredYaw The desired yaw
 * @param currentYaw The current yaw
 * @param desiredAltitude The desired altitude
 * @param currentAltitude The current altitude
 * @param motor1 The percentage of motor 1
 * @param motor2 The percentage of motor 2
 * @param mode The current mode
 * 
 */
void serialUART_SendInformation(int32_t desiredYaw, int32_t currentYaw, int32_t desiredAltitude, int32_t currentAltitude, int8_t motor1, int8_t motor2, uint8_t mode);

#endif /* SERIALUART_H */
