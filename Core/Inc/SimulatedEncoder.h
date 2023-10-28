#ifndef SIMULATED_ENCODER_H
#define SIMULATED_ENCODER_H

#include <cstdint>

class SimulatedEncoder {
public:
    SimulatedEncoder(int32_t initialCount = 0);

    // Simulate incrementing the count as if the encoder were rotating
    void simulateRotation(int32_t increment = 1);

    // Get the current count value
    int32_t get_count() const;

    // Reset the count to a specific value
    void reset_count(int32_t newValue = 0);

    // Simulate getting the position based on count and resolution
    float simulateGetPosition(int32_t resolution = 1000) const;

private:
    int32_t count;
};

#endif // SIMULATED_ENCODER_H
