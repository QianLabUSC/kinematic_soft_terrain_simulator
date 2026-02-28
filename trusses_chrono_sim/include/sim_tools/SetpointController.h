#include <iostream>
#include <algorithm>  // For std::clamp

class SetpointController {
private:
    double setpoint;
    double velocity;
    double min_limit;
    double max_limit;

public:
    // Constructor to initialize setpoint and velocity
    SetpointController(double initial_setpoint, double initial_velocity = 0.0, 
                       double min_limit = -1e9, double max_limit = 1e9)
        : setpoint(initial_setpoint), velocity(initial_velocity), 
          min_limit(min_limit), max_limit(max_limit) {}

    // Function to adjust the setpoint based on velocity
    void adjustSetpointWithVelocity(double delta_time) {
        // Adjust setpoint based on velocity
        setpoint += velocity * delta_time;
        // Clamp the setpoint to its min and max limits
        setpoint = std::clamp(setpoint, min_limit, max_limit);
    }

    // Function to directly adjust the setpoint
    void adjustSetpoint(double delta) {
        setpoint += delta;
        // Optionally clamp the setpoint to its min and max limits
        setpoint = std::clamp(setpoint, min_limit, max_limit);
    }

    // Function to set the velocity directly
    void setVelocity(double new_velocity) {
        velocity = new_velocity;
    }

    // Function to set the setpoint directly
    void setSetpoint(double new_setpoint) {
        setpoint = std::clamp(new_setpoint, min_limit, max_limit);
    }

    // Getter for the current setpoint
    double getSetpoint() const {
        return setpoint;
    }

    // Getter for the current velocity
    double getVelocity() const {
        return velocity;
    }

    // Getter for the max limit
    double getMaxLimit() const {
        return max_limit;
    }

    // Getter for the min limit
    double getMinLimit() const {
        return min_limit;
    }

    // Optionally, you can provide functions to set limits
    void setLimits(double new_min_limit, double new_max_limit) {
        min_limit = new_min_limit;
        max_limit = new_max_limit;
        setpoint = std::clamp(setpoint, min_limit, max_limit);
    }
};