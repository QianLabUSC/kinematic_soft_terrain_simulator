#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "sim_tools/PIDController.h"

using namespace chrono;

#ifndef TRUSS_H
#define TRUSS_H

class Truss {
public:
    /// @brief Create a truss object and put it in the simulation.
    /// @param sys Chrono system
    /// @param body Robot body
    /// @param relative_truss_pos Truss base position relative to the body
    Truss(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, const ChVector3d& relative_truss_pos);
    
    /// @brief Visually & inertially update the length of the arm. Must be called continuously inside the sim loop. Also vis->BindAll() must be called after this function.
    void UpdateArmLength();

    /// @brief Set the desired length of the arm.
    /// @param length desired length of the arm
    void SetArmDesiredLength(double length);

    /// @brief Choose PID values for the arm joints.
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetArmPIDGains(double kp, double ki, double kd);

    /// @brief Get the length of the truss.
    double GetLength();

    void ConstructTruss(const ChVector3d& desired_coupler_pos);

    // Create a material for the truss
    std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();

    std::shared_ptr<chrono::ChBodyEasyBox> base = chrono_types::make_shared<ChBodyEasyBox>(.1, .1, .1, 10, true, false, material);

    std::shared_ptr<chrono::ChBodyEasyCylinder> arm = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.02, 1, .1, material);
    
    std::shared_ptr<chrono::ChBodyEasyBox> coupler = chrono_types::make_shared<ChBodyEasyBox>(.1, .1, .1, 10, true, false, material);
    std::shared_ptr<chrono::ChLinkMotorLinearSpeed> motor;

    double desired_length = -1;
    
private:
    ChSystem* m_sys;
    bool once = false;

    PIDController pidController = PIDController(250,0,0);

    std::shared_ptr<chrono::ChBody> m_body;
    ChVector3d m_truss_pos;

    std::shared_ptr<chrono::ChLinkLockLock> base_to_arm_link;
    std::shared_ptr<chrono::ChLinkLockSpherical> base_to_body_link;


   

};

#endif // TRUSS_H