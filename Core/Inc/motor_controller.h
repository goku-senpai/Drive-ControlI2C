#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "constants.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "pid_controller.h" // Include the PID controller header

class MotorController {
public:
    MotorController(TIM_HandleTypeDef* htim_pwm, uint32_t channel_pwm,
                    GPIO_TypeDef* gpio_pwm, uint32_t pin_pwm,
                    float max_output, float max_integral, float target_start,
                    bool is_speed_controller);

    void set_direction(bool direction_fwd, bool direction_bwd);
    void set_target(float dSetpoint);
    void update(float sample_time, int32_t encoder_value, uint8_t bFreewheel, float dKp, float dKi, float dKd);

private:
    TIM_HandleTypeDef* htim_pwm_;
    uint32_t channel_pwm_;
    GPIO_TypeDef* gpio_pwm_;
    uint32_t pin_pwm_;
    GPIO_TypeDef* gpio_dir_fwd_;
    uint32_t pin_direction_fwd_;
    GPIO_TypeDef* gpio_dir_bwd_;
    uint32_t pin_direction_bwd_;

    bool is_speed_controller_;
    uint32_t current_position_;
    float current_output_;
    uint32_t prev_encoder_value_;

    // Member variables for control mode, PID gains, and PID controller
    uint8_t control_mode_; // Store control mode (bFreewheel or bPosition)
    float dKp_;           // Store PID proportional gain
    float dKi_;           // Store PID integral gain
    float dKd_;           // Store PID derivative gain
    PIDController pid_controller_; // The PID controller instance
};

#endif
