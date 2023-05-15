/**
 * @file SerialUART.c
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief Send and receive data over UART
 * @date 2023-03-12
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 */

#define PART_TM4C1230C3PM // Target device

// ========================= Include files =========================
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "serialUART.h"
// ========================= Constants and types =========================
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

// ========================= Global Variables =========================

// ========================= Function Definitions =========================
/**
 * @brief Initialises the UART for sending and recieving data
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 */
void serialUART_init(uint32_t maxBufferSize) {
    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);

    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);


    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable the UART communcation
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

/**
 * @brief Sends a string of data over UART
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 * @param charBuffer The string of data to be sent
 */
static void serialUART_SendBuffer(char *charBuffer) {
    while (*charBuffer) {
        // Write the next character to the UART Tx.
        UARTCharPut(UART_USB_BASE, *charBuffer);
        charBuffer++;
    } 
}

/**
 * @brief Send the serial infromation 
 * @cite uartDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * @param desiredYaw The desired yaw
 * @param currentYaw The current yaw
 * @param desiredAltitude The desired altitude
 * @param currentAltitude The current altitude
 * @param motor1 The percentage of motor 1
 * @param motor2 The percentage of motor 2
 * @param mode The current mode
 * 
 */
void serialUART_SendInformation(int32_t desiredYaw, int32_t currentYaw, int32_t desiredAltitude, int32_t currentAltitude, int8_t motor1, int8_t motor2, uint8_t mode) {
    char string[200];

    // Convert yaw
    int32_t degrees = currentYaw / 10;

    // Find the decimal value an convert it to absolute value
    int32_t decimalDegrees = (currentYaw < 0) ? currentYaw % 10 * -1 : currentYaw % 10;

    // Convert yaw
    int32_t desiredDegrees = desiredYaw / 10;

    // Find the decimal value an convert it to absolute value
    int32_t desiredDecimalDegrees = (desiredYaw < 0) ? desiredYaw % 10 * -1 : desiredYaw % 10;

    // Send the information
    usnprintf (string, sizeof(string), 
       "Yaw: %4d.%1d [%4d.%1d], Alt: %3d%% [%3d%%], Main: %3d%%, Tail: %3d%%, Mode: %1d\n\r",
       degrees, decimalDegrees, desiredDegrees, desiredDecimalDegrees, currentAltitude, desiredAltitude, motor1, motor2, mode);

    serialUART_SendBuffer(string);
}
