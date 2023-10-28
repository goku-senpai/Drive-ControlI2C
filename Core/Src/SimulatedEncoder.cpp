#include "SimulatedEncoder.h"

SimulatedEncoder::SimulatedEncoder(int32_t initialCount) : count(initialCount) {}

// Simulate incrementing the count as if the encoder were rotating
void SimulatedEncoder::simulateRotation(int32_t increment) {
    count += increment;
}

// Get the current count value
int32_t SimulatedEncoder::get_count() const {
    return count;
}

// Reset the count to a specific value
void SimulatedEncoder::reset_count(int32_t newValue) {
    count = newValue;
}

// Simulate getting the position based on count and resolution
float SimulatedEncoder::simulateGetPosition(int32_t resolution) const {
    return static_cast<float>(count) / resolution;
}
