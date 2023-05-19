/** 
 * @file main.h
 * @brief general constants and user defined types
 * @author Jack Duignan (jdu80@uclive.ac.nz)
 * @date 2023-05-17
 */


#ifndef MAIN_H
#define MAIN_H


// ===================================== Includes =====================================
#include <stdint.h>
#include <stdbool.h>

// ===================================== Constants ====================================
typedef struct {
    uint8_t mode;
    int16_t altitude;
    int16_t yaw;
    int16_t altitudeSetpoint;
    int16_t yawSetpoint;
    uint8_t mainMotorDuty;
    uint8_t tailMotorDuty;
    bool mainMotorRamped;
    bool yawRefFound;
} heliInfo_t;

#define YAW_DEGREES_SCALE 10

enum MOTOR {MAIN_MOTOR, TAIL_MOTOR};
enum MAIN_STATE {LANDED, TAKING_OFF, FLYING, LANDING};


#endif // MAIN_H
