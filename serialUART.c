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

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include <debug.h>
#include <serialUART.h>
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
void serialUART_SendInformation(char *charBuffer) {
    while (*charBuffer) {
        // Write the next character to the UART Tx.
        UARTCharPut(UART_USB_BASE, *charBuffer);
        charBuffer++;
    } 
}
