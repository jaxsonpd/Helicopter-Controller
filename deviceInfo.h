/** 
 * @file deviceInfo.h
 * @brief Struct to allow easier access to device information (Altitude etc.)
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-17
 */


#ifndef DEVICEINFO_H
#define DEVICEINFO_H


// ===================================== Includes =====================================
#include <stdint.h>

// ===================================== Constants ====================================
typedef struct {
    uint8_t mode;
    int16_t altitude;
    int16_t yaw;
    int16_t altitudeSetpoint;
    int16_t yawSetpoint;
    uint8_t mainMotorDuty;
    uint8_t tailMotorDuty;
} deviceInfo_t;


enum MAIN_STATE {LANDED, TAKING_OFF, FLYING, LANDING};
#endif // DEVICEINFO_H
