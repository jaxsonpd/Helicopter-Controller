/** 
 * @file heliFunctions.h
 * @brief Header file for heliFunctions.c
 * @author Jack Duignan (jdu80@uclive.ac.nz), Daniel Hawes (dha144@uclive.ac.nz)
 * @date 2023-05-18
 */


#ifndef HELIFUNCTIONS_H
#define HELIFUNCTIONS_H


// ===================================== Includes =====================================
#include "main.h"

// ===================================== Constants ====================================


// ===================================== Function Prototypes ==========================
/**
 * @brief Take off the helicopter
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_takeoff(heliInfo_t *heliInfo);


/**
 * @brief Land the helicopter
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_land(heliInfo_t *heliInfo);


/**
 * @brief Update the helicopter setpoints while flying
 * @param heliInfo the helicopter info struct
 * 
 */
void heliFunctions_updateSetpoints(heliInfo_t *heliInfo);


#endif // HELIFUNCTIONS_H
