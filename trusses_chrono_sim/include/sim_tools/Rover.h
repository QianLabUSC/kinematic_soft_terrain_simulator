#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChBodyEasy.h"

#include "sim_tools/PIDController.h"
#include "sim_tools/Truss.h"
#include "sim_tools/ROSRover.h"

using namespace chrono;

#ifndef ROVER_H
#define ROVER_H

class Rover {
public:
    std::shared_ptr<ROSRover> node;

    ChSystem* m_sys;
    std::shared_ptr<chrono::ChBody> m_body;
    std::vector<std::shared_ptr<chrono::ChBody>> m_wheels;
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> m_wheel_motors;
    std::vector<Truss> m_trusses;

    std::vector<PIDController> pidControllers;

    /// @brief Given the absolute path to the ROS package, number of trusses on the rover, and the step_size for the simulation integration, this function initializes the rover object.
    Rover(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> wheels, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> wheel_motors, std::vector<Truss> trusses, int roverNum);
    
    /// @brief Enables collisions for all of the collision bodies of the robot (body, and legs), and creates SCM moving patches for each of them.
    /// @param mterrain The SCM terrain
    void AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain);

    

    /// @brief Fixes the red box on the end of the truss to a provided location in space. This allows the truss to push and pull as if one end was fixed. 
    /// @param truss_to_lock Truss being referenced using 0-index.
    /// @param position Position of the red box. Global frame
    void LockTrussEndPosition(int truss_to_lock, ChVector3d position);

    /// @brief Connect the truss to the provided robot body at a specified location.
    /// @param truss_num Truss being referenced using 0-index.
    /// @param docked_robot_body Robot body to dock to.
    /// @param relative_position Position of the truss connection relative to the robot body.
    void DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_position);

    /// @brief Choose PID values for the wheel joints.
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetWheelGains(double kp, double ki, double kd);

    /// @brief Joint position you want the wheel to try to reach using PID.
    /// @param wheel Wheel being referenced using 0-index.
    /// @param setpoint Desired position (rad)
    void SetWheelDesiredPosition(int wheel, double setpoint);

    /// @brief Joint velocity you want the wheel to try to reach using PID.
    /// @param wheel Wheel being referenced using 0-index.
    /// @param setpoint Desired velocity (rad/s)
    void SetWheelDesiredVelocity(int wheel, double setpoint);
    
    /// @brief Call at the top of every sim loop timestep to update values from subscribers and to publishers, and handle service calls.
    void HandleROS();

private:

};

#endif