#include "sim_tools/PIDController.h"
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd, double maxOut, double maxSpd) 
    : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0), maxOutput(maxOut), maxSpeed(maxSpd) {}

double PIDController::calculate(double setpoint, double processVariable, double dt){
    double error = setpoint - processVariable;
    integral += error * dt;
    double derivative = (error-prevError) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    
    // Apply speed limit (limit the rate of change)
    double speed = derivative;
    if (std::abs(speed) > maxSpeed) {
        speed = (speed > 0) ? maxSpeed : -maxSpeed;
        output = Kp * error + Ki * integral + Kd * speed;
    }
    
    // Apply torque limit
    if (std::abs(output) > maxOutput) {
        output = (output > 0) ? maxOutput : -maxOutput;
    }
    
    prevError = error;
    return output;
};

void PIDController::setGains(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setMaxOutput(double maxOut) {
    maxOutput = maxOut;
}

void PIDController::setMaxSpeed(double maxSpd) {
    maxSpeed = maxSpd;
}