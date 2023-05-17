/**
 * @file yaw.h
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @brief HEADER - caculate the current yaw of the helicopter from the quadrature endcoder
 * @date 2023-04-13
 * 
 */

#ifndef YAW_H
#define YAW_H

// ========================= Include files =========================
#include <stdint.h>

// ========================= Function Prototypes =========================
/**
 * @brief Pin Change intrupt handler for the yaw encoder
 *
 */
void encoderChangeInt_Handler(void);

/**
 * @brief initilise the quadrature encoder
 * 
 */
void yaw_init(void);

/**
 * @brief get the current yaw of the helicopter
 * 
 * @return current yaw of the helicopter +- from the zero position in degrees / 10
 */
int32_t yaw_get(void);

/**
 * @brief Return the encoder value
 * 
 * @return encoder value
*/
int32_t yaw_getEncoderValue(void);

/**
 * @brief get the current values of the quadrature encoder channels
 * 
 * @return current values of the quadrature encoder channels (0000 BPrev APREV B A)
 */
uint8_t yaw_getChannels(void);

/**
 * @brief Reset the yaw encoder value to zero
 * 
 */
void yaw_reset(void);

/**
 * @brief Return the yaw reference signal for the yaw reset function on takeoff
 * 
 * @return yaw reference signal (1 = high, 0 = low)
 */
uint8_t yaw_getRef(void);

#endif /* YAW_H */
