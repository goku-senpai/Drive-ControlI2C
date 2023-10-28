#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"

class Encoder {
public:
    Encoder(TIM_HandleTypeDef* htim, GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB);
    void init();
    int32_t get_count();
    void reset_count();
    float get_position();

    float get_positionI2C();

private:
    TIM_HandleTypeDef* _htim;
    GPIO_TypeDef* _portA;
    uint16_t _pinA;
    GPIO_TypeDef* _portB;
    uint16_t _pinB;
};

#endif
