/**
 * @file encoder.cpp
 * @brief Encoder class
 *
 */
#include "encoder.h"
#include "constants.h"
#include "stm32f7xx_hal_tim.h"

/*
 I2C variable declaration
 */
extern I2C_HandleTypeDef hi2c1;
static const uint8_t i2c_adress = 0x6C;
//static const uint8_t i2c_adress=0x36;
static const float convertValue = 0.08789f;  //12bit in winkel
static const uint8_t angleAdress=0x0C;
static const uint8_t statusAdress=0x0B;
uint16_t rawData =0;
uint8_t status =0;
float angle = 0;
uint8_t data[5]; //data i recieve from i2c encoder
uint8_t buff[20]; //rx buffer
extern HAL_StatusTypeDef ret;

//This should work with a normal encoder, for this project, a AS5600 I2C encoder is used
/**
 * @brief Construct a new Encoder:: Encoder object
 * 
 * @param htim 
 * @param portA
 * @param pinA 
 * @param portB 
 * @param pinB 
 */
Encoder::Encoder(TIM_HandleTypeDef* htim, GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB)
{
    this->_htim = htim;
    this->_portA = portA;
    this->_pinA = pinA;
    this->_portB = portB;
    this->_pinB = pinB;
}

/**
 * @brief I2CEncoder
 * @return position
 */
float Encoder:: get_positionI2C() {
    data[0] = angleAdress;
    ret = HAL_I2C_Master_Transmit(&hi2c1, i2c_adress, data, 1, 10);
    if (ret == HAL_OK) {
        ret = HAL_I2C_Master_Receive(&hi2c1, i2c_adress, data, 2, 10);
        if (ret == HAL_OK) {
            rawData = data[0] << 8;
            rawData |= data[1];
            angle = (float) rawData * convertValue;
        } else {
            //HAL_GPIO_WritePin(LED_RED_PORT,LED_RED_PIN,GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(LED_RED_PORT,LED_RED_PIN,GPIO_PIN_SET);
    }
    return angle;
}

/**
 * @brief analogEncoder init
 * 
 */
void Encoder::init()
{
    // Set up the timer for the Encoder
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure Pin A
    GPIO_InitStruct.Pin = _pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ENCODER_M1_A_ALTERNATE;
    HAL_GPIO_Init(_portA, &GPIO_InitStruct);

    // Configure Pin B
    GPIO_InitStruct.Pin = _pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ENCODER_M1_B_ALTERNATE;
    HAL_GPIO_Init(_portB, &GPIO_InitStruct);

    HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
}
/**
 * @brief analogEncoder counter
 * @return int32_t counts
 **/

int32_t Encoder::get_count()
{
    return (int32_t) __HAL_TIM_GET_COUNTER(_htim);
}

/**
 * @brief analogEncoder resetcounter
 *
 **/
void Encoder::reset_count()
{
    __HAL_TIM_SET_COUNTER(_htim, 0);
}

/**
 * @brief analogEncoder getpos
 * @return int32_t position
 **/

float Encoder::get_position()
{
    int32_t count = get_count();
    float position = (float)count / ENCODER_RESOLUTION;
    return position;
}

