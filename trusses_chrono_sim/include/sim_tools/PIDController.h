#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
class PIDController{
    public:
        double Kp, Ki, Kd;
        double prevError, integral;
        double maxOutput; // Maximum output limit (torque)
        double maxSpeed;  // Maximum speed limit (rad/s)
        PIDController(double kp, double ki, double kd, double maxOut = 1000.0, double maxSpd = 10.0);
        double calculate(double setpoint, double processVariable, double dt);
        void setGains(double kp, double ki, double kd);
        void setMaxOutput(double maxOut);
        void setMaxSpeed(double maxSpd);
    private:

};

#endif