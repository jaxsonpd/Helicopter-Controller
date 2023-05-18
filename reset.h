/** 
 * @file reset.h
 * @brief Header file for reset.c
 * @author Jack Duignan (Jackpduignan@gmail.com)
 * @date 2023-05-18
 */


#ifndef RESET_H
#define RESET_H
// ===================================== Includes =====================================


// ===================================== Constants ====================================


// ===================================== Function Prototypes ==========================
/**
 * @brief initialize the reset button
 * 
 */
void reset_init(void);

/**
 * @brief check if the reset button is pressed and perform a reset if it is
 * 
 */
void reset_check(void);

#endif // RESET_H
