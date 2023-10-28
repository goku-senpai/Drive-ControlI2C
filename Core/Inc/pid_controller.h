#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_output, float max_integral, float target);

    // Update the PID controller with the current value and sample time
    void update(float current_value, float sample_time);

    // Setters for PID gains
    void set_kp(float kp);
    void set_ki(float ki);
    void set_kd(float kd);
    void set_target(float target);

    // Getter for PID output
    float get_output() const;
    float get_target() const;

private:
    float kp_;
    float ki_;
    float kd_;
    float max_output_;
    float max_integral_;
    float target_;

    float integral_;
    float prev_error_;
    float output_;
};

#endif
