/**
 * @file altitude.c
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief read the altitude of the helicopter and store it in a circular buffer
 * @date 2023-03-12
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
#include "stdio.h"

#include "circBufT.h"


// ========================= Constants and types =========================
#define DEBUG // Change analog input channel to 0 for debugging


// ========================= Global Variables =========================
static circBuf_t g_inBuffer;		// Buffer of size BUF_SIZE integers (sample values)
static uint32_t g_ulSampCnt = 0;		// Counter for the numbler of samples processed
static uint8_t bufferSize;            // Size of the circular buffer
static uint32_t maxAltitudeADC = 1080; // 1V value in the adc
static uint32_t minAltitudeADC = 2250; // 2V value in the adc
static uint32_t ADCValue;


// ========================= Function Definition =========================
/**
 * @brief Store the adc value when the adc conversion is complete triggered by the adc done interupt
 * @cite ADCDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 */
static void ADCCompletedInt_Handler(void) {
	//
	// Get the single sample from ADC0.  ADC_BASE is defined in
	// inc/hw_memmap.h
	ADCSequenceDataGet(ADC0_BASE, 3, &ADCValue);
	//
	// Place it in the circular buffer (advancing write index)
	writeCircBuf (&g_inBuffer, ADCValue);
	//
	// Clean up, clearing the interrupt
	ADCIntClear(ADC0_BASE, 3);    
}


/**
 * @brief initilise the ADC and the circular buffer
 * @cite ADCDemo.c from the lab 4 folder author: P.J. Bones UCECE
 * 
 * @param buffSize size of the circular buffer
*/
void altitude_init(uint16_t buffSize) {
    bufferSize = buffSize;
    
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    #ifdef DEBUG
    
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);    
    #else
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);  
    #endif
                             
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
  
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCCompletedInt_Handler);
  
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);

    // Initialize the circular buffer.
    initCircBuf (&g_inBuffer, bufferSize);
}


/**
 * @brief get the average altitude of the helicopter from the circular buffer (0-100) ONE volt = 100% Two volts = 0%
 * 
 * @return uint8_t average altitude (0-100)
 */
uint32_t altitude_get(void) {
    uint32_t sum = 0;
    uint8_t i;

    // Find mean
    for (i = 0; i < bufferSize; i++) {
        sum += readCircBuf(&g_inBuffer);
    }

    sum = (2* sum + bufferSize) / 2 / bufferSize;
    
    if (sum < maxAltitudeADC) {
        return 100;
    } else {
        // Convert to percentage
        return (uint32_t) (100 - ((sum - maxAltitudeADC) * 100) / (minAltitudeADC - maxAltitudeADC));
    }

}


/**
 * @brief get the average ADC value of the helicopter from the circular buffer
 * 
 * @return uint16_t average ADC value (0-4096)
*/
uint32_t altitude_getRaw(void) {
    uint32_t sum = 0;
    uint8_t i;

    // Find mean
    for (i = 0; i < bufferSize; i++) {
        sum += readCircBuf(&g_inBuffer);
    }

    sum = (2* sum + bufferSize) / 2 / bufferSize;
}


/**
 * @brief Get the number of samples that have been taken
 * 
 * @return uint16_t number of samples
 */
uint32_t altitude_getSamples(void) {
    return g_ulSampCnt;
}


/**
 * @brief Trigger the ADC to read the altitude of the helicopter
 * 
*/
void altitude_read(void) {
    // Trigger the ADC conversion.
    ADCProcessorTrigger(ADC0_BASE, 3);

    g_ulSampCnt++;
}


/**
 * @brief Reset minimum altitude to the current ADC value
 * 
 */
void altitude_setMinimumAltitude(void) {
    minAltitudeADC = ADCValue - 10; // Make sure it stays at 10
}
