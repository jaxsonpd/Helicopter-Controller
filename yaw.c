/**
 * @file yaw.c
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief calculate the current yaw of the helicopter from the quadrature encoder
 * @date 2023-04-13
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
#include "OrbitOLED/OrbitOLEDInterface.h"

#include "utils/ustdlib.h"
#include "stdio.h"


// ========================= Constants and types =========================
#define YAW_ENC_PERIPHERAL SYSCTL_PERIPH_GPIOB // Peripheral for yaw encoder pins

// Channel A input pin for yaw (J1-03)
#define YAW_ENC_CHA_PIN GPIO_PIN_0 
#define YAW_ENC_CHA_PORT GPIO_PORTB_BASE

// Channel B input pin for yaw (J1-04)
#define YAW_ENC_CHB_PIN GPIO_PIN_1
#define YAW_ENC_CHB_PORT GPIO_PORTB_BASE

#define NUM_SLOTS_PER_REVOLUTION 196 // Number of slots in the quadrature encoder
#define DEGREES_SCALE 10 // Scale factor for returning degrees so not to use floats


// ========================= Global Variables =========================
static volatile int32_t encoderValue = 0; // Current yaw encoder value of the helicopter
static volatile bool channelA_prev = false; // Previous state of channel A
static volatile bool channelB_prev = false; // Previous state of channel B


// ========================= Function Definition =========================
/**
 * @brief Pin Change intrupt handler for the yaw encoder
 * 
 */
void encoderChangeInt_Handler(void) {
    // Clear the interrupt
    GPIOIntClear(YAW_ENC_CHA_PORT | YAW_ENC_CHB_PORT, YAW_ENC_CHA_PIN | YAW_ENC_CHB_PIN);

    // Get the current state of the pins
    bool channelA = GPIOPinRead(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN);
    bool channelB = GPIOPinRead(YAW_ENC_CHB_PORT, YAW_ENC_CHB_PIN);
    
    // Calculate the new encoder value
    if (channelB != channelB_prev) {
        // Channel B triggered the intrupt
        if (channelA == channelB_prev) encoderValue++; // B leads A so clockwise
        else encoderValue--; // A leads B so anti-clockwise
    } else {
        // Channel A triggered the intrupt
        if (channelB == channelA_prev) encoderValue--; // A leads B so anti-clockwise
        else encoderValue++; // B leads A so clockwise
    }

    // Set the previous states of the channels
    channelA_prev = channelA;
    channelB_prev = channelB;
}


/**
 * @brief Initialise the yaw module
 * 
 */
void yaw_init(void) {
    // Enable the GPIO port that is used for the encoder pins.
    SysCtlPeripheralEnable(YAW_ENC_PERIPHERAL);

    // Yaw channel A
    GPIOPinTypeGPIOInput(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN);
    GPIOPadConfigSet(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Yaw channel B
    GPIOPinTypeGPIOInput(YAW_ENC_CHB_PORT, YAW_ENC_CHB_PIN);
    GPIOPadConfigSet(YAW_ENC_CHB_PORT, YAW_ENC_CHB_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Register the pin change interupts for each channel
    GPIOIntRegister(YAW_ENC_PERIPHERAL, encoderChangeInt_Handler);
    GPIOIntTypeSet(YAW_ENC_CHA_PORT | YAW_ENC_CHB_PORT, YAW_ENC_CHA_PIN | YAW_ENC_CHB_PIN, GPIO_BOTH_EDGES);
    
    // Enable the interrupts for the pins
    GPIOIntEnable(YAW_ENC_CHA_PORT | YAW_ENC_CHB_PORT, YAW_ENC_CHA_PIN | YAW_ENC_CHB_PIN);

    // Set the prevous states of the channels
    channelA_prev = GPIOPinRead(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN);
    channelB_prev = GPIOPinRead(YAW_ENC_CHB_PORT, YAW_ENC_CHB_PIN);

    // Set the current yaw to zero
    encoderValue = 0;
}


/**
 * @brief get the current yaw of the helicopter
 * 
 * @return int32_t current yaw of the helicopter +- from the zero position degrees / 10
 */
int32_t yaw_get(void) {
    // Convert from encoder value to degrees
    return (encoderValue * 360 * DEGREES_SCALE) / NUM_SLOTS_PER_REVOLUTION;
}


/**
 * @brief get the current values of the quadrature encoder channels
 * 
 * @return uint8_t current values of the quadrature encoder channels (0000 BPrev APREV B A)
 */
uint8_t yaw_getChannels(void) {
    return (GPIOPinRead(YAW_ENC_CHA_PORT, YAW_ENC_CHA_PIN) >> 0) |
       (GPIOPinRead(YAW_ENC_CHB_PORT, YAW_ENC_CHB_PIN) >> 1) |
       (channelA_prev << 2) |
       (channelB_prev << 3);
}
