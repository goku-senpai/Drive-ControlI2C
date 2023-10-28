/**
 * @file motor_controller.cpp
 * @brief Motor controller
 */

#include "motor_controller.h"
#include <cmath>
#include <sstream>
#include "string.h"

// Define constants for control mode
const uint8_t CONTROL_MODE_FREEWHEEL = 1;
const uint8_t CONTROL_MODE_POSITION = 0;
uint8_t uartdebugBuffer[UART_RX_BUFFER_SIZE];

extern UART_HandleTypeDef huart3;


bool Motor_Simulation=false;

// Initialize the motor controller
MotorController::MotorController(TIM_HandleTypeDef* htim_pwm, uint32_t channel_pwm,
                                 GPIO_TypeDef* gpio_pwm, uint32_t pin_pwm,
                                 float max_output, float max_integral, float target_start,
                                 bool is_speed_controller)
        : htim_pwm_(htim_pwm), channel_pwm_(channel_pwm),
          gpio_pwm_(gpio_pwm), pin_pwm_(pin_pwm),  // Use the provided gpio_pwm and pin_pwm
          pid_controller_(0.0, 0.0, 0.0, max_output, max_integral, target_start),
          current_position_(0), current_output_(0),
          is_speed_controller_(is_speed_controller),
          control_mode_(CONTROL_MODE_FREEWHEEL) {
}


/**
 * @brief setter direction
 * @param direction_fwd
 * @param direction_bwd
 */

void MotorController::set_direction(bool direction_fwd, bool direction_bwd) {
    if (direction_fwd) {
        // Set IN1 and IN2 for forward motion
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    } else if (direction_bwd) {
        // Set IN1 and IN2 for backward motion
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    } else if (direction_bwd && direction_fwd) {
        // Set IN1 and IN2 STOP
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
    } else {
        // Both IN1 and IN2 should be off for off
        HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
    }
}
/**
 * @brief update function for Motor controller
 * @param sample_time
 * @param encoder_value
 * @param bFreewheel
 * @param dKp
 * @param dKi
 * @param dKd
 */
void MotorController::update(float sample_time, int32_t encoder_value, uint8_t bFreewheel, float dKp, float dKi, float dKd) {
    // Set the control mode based on received data
    control_mode_ = bFreewheel ? CONTROL_MODE_FREEWHEEL : CONTROL_MODE_POSITION;

    // Set PID gains based on received data
    pid_controller_.set_kp(dKp);
    pid_controller_.set_ki(dKi);
    pid_controller_.set_kd(dKd);

    if (is_speed_controller_) {
        // Compute motor output based on PID controller output
        current_output_ = pid_controller_.get_output();

        // Update PID controller with current speed and time delta
        float current_speed = (encoder_value - prev_encoder_value_) / (sample_time * ENCODER_RESOLUTION);
        pid_controller_.update(current_speed, sample_time);

        // Update motor direction based on output sign
        if (current_output_ >= pid_controller_.get_target()) {
            set_direction(true, false); // Forward
        } else {
            set_direction(false, true); // Backward
        }

        // Check if the target is reached
        if (fabs(current_output_) >= fabs(pid_controller_.get_target())) {
            // Stop the motor
            current_output_ = 0.0;
        }

        // Compute motor output based on PID controller output and current speed
        current_output_ = pid_controller_.get_output() + current_speed * SPEED_KF;
    } else {
        // Update PID controller with current position and time delta
        if (sample_time <= 0.0) {
            return;
        } else {
            if (current_output_ >= pid_controller_.get_target()) {
                set_direction(true, false); // Forward
                pid_controller_.update(current_position_, sample_time);
            } else {
                set_direction(false, true); // Forward
                pid_controller_.update(current_position_, sample_time);
            }
        }
    }

    // Check if the target is reached
    if (fabs(current_position_ - pid_controller_.get_target()) <= POSITION_TOLERANCE) {
        // Stop the motor
        current_output_ = 0.0;
        set_direction(true, true); // Forward
    }

    // Convert output magnitude to PWM duty cycle and write to PWM pin
    uint32_t duty_cycle = (uint32_t) (fabs(current_output_) * MAX_OUTPUT / 100);
    if (duty_cycle >= DUTYCYCLE_MAX) {
        TIM4->CCR1 = DUTYCYCLE_MAX;
        HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    }
    else if (duty_cycle <= DUTYCYCLE_MIN){
        TIM4->CCR1 = DUTYCYCLE_MIN;
        HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    }
    else {
        TIM4->CCR1 = duty_cycle;
    }
    // Update previous encoder value for speed calculation
    prev_encoder_value_ = encoder_value;
}

/**
 * setter for Setpoint
 * @param dSetpoint
 */
void MotorController::set_target(float dSetpoint) {
    pid_controller_.set_target(dSetpoint);
}