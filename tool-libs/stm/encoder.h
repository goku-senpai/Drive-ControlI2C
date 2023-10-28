#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f7xx_hal.h"

// Define the encoder struct
typedef struct {
    TIM_HandleTypeDef* timer;  // Timer instance used to read encoder counts
    int16_t counts_per_rev;    // Number of encoder counts per revolution
    float angle_per_count;     // Angle moved per encoder count (in degrees)
    int32_t count_offset;      // Offset for encoder counts
    float angle_offset;        // Offset for encoder angle (in degrees)
} Encoder_t;

// Initialize an encoder with the given parameters
void Encoder_Init(Encoder_t* encoder, TIM_HandleTypeDef* timer, int16_t counts_per_rev, float angle_per_count);

// Reset the encoder counts and angle
void Encoder_Reset(Encoder_t* encoder);

// Get the current encoder counts
int32_t Encoder_Get_Counts(Encoder_t* encoder);

// Get the current encoder angle (in degrees)
float Encoder_Get_Angle(Encoder_t* encoder);

#endif
