/**
 * @file pid_controller.cpp
 * @brief PID controller
 */

#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target)
        : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), max_integral_(max_integral), target_(target),
          integral_(0), prev_error_(0), output_(0) {
}

/**
 * @brief update function
 * @param current_value
 * @param sample_time
 */

void PIDController::update(float current_value, float sample_time) {
    // Calculate the error
    float error = target_ - current_value;

    // Calculate the integral term
    integral_ += error * sample_time;

    // Limit the integral term
    if (integral_ > max_integral_) {
        integral_ = max_integral_;
    } else if (integral_ < -max_integral_) {
        integral_ = -max_integral_;
    }

    // Calculate the derivative term
    float derivative = (error - prev_error_) / sample_time;

    // Calculate the PID output
    output_ = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Limit the PID output
    if (output_ > max_output_) {
        output_ = max_output_;
    } else if (output_ < -max_output_) {
        output_ = -max_output_;
    }

    // Update the previous error
    prev_error_ = error;
}

void PIDController::set_kp(float kp) {
    kp_ = kp;
}

void PIDController::set_ki(float ki) {
    ki_ = ki;
}

void PIDController::set_kd(float kd) {
    kd_ = kd;
}

void PIDController::set_target(float target) {
    target_ = target;
}

float PIDController::get_output() const {
    return output_;
}

float PIDController::get_target() const {
    return target_;
}
