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
 * @brief initilise the quadrature encoder
 * 
 */
void yaw_init(void);

/**
 * @brief get the current yaw of the helicopter
 * 
 * @return int32_t current yaw of the helicopter +- from the zero position in degrees / 10
 */
int32_t yaw_get(void);

/**
 * @brief get the current values of the quadrature encoder channels
 * 
 * @return uint8_t current values of the quadrature encoder channels (0000 BPrev APREV B A)
 */
uint8_t yaw_getChannels(void);

#endif /* YAW_H */