#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChBodyEasy.h"

#include "sim_tools/PIDController.h"
#include "sim_tools/Truss.h"
#include "sim_tools/Probe.h"
#include "sim_tools/ROSSpirit.h"

using namespace chrono;

#ifndef SPIRIT_H
#define SPIRIT_H

class Spirit {
public:

    std::shared_ptr<ROSSpirit> node;

    ChSystem* m_sys;
    std::shared_ptr<chrono::ChBody> m_body;
    std::vector<std::vector<std::shared_ptr<chrono::ChBody>>> m_leg_bodies;
    std::vector<std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> m_leg_motors;
    std::vector<Truss> m_trusses;
    std::vector<Probe> m_probes;
    std::vector<ChVector3d> m_probe_forces;


    std::vector<PIDController> pidControllers;


    /// @brief Given the absolute path to the ROS package, number of trusses on the spirit, and the step_size for the simulation integration, this function initializes the spirit object.
    //Spirit(ChSystem* sys, std::string package_filepath, int robot_num, int truss_num, double dt);
Spirit(ChSystem* sys, 
       std::shared_ptr<chrono::ChBody> body, 
       std::vector<std::vector<std::shared_ptr<chrono::ChBody>>> leg_bodies, 
       std::vector<std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> leg_motors, 
       std::vector<Truss> trusses, 
       std::vector<Probe> probes, 
       int spiritNum);

    std::vector<ChVector3d> GetProbeForces(chrono::vehicle::SCMTerrain mterrain);

    void MakeProbe(ChVector3d xpos, ChVector3d ypos);

    /// @brief Enables collisions for all of the collision bodies of the robot (body, and toes), and creates SCM moving patches for each of them.
    /// @param mterrain The SCM terrain
    void AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain);

    /// @brief Fixes the red box on the end of the truss to a provided location in space. This allows the truss to push and pull as if one end was fixed.
    /// @param truss_to_lock Truss being referenced using 0-index.
    /// @param position Position of the red box. Global frame
    void LockTrussEndPosition(int truss_to_lock, ChVector3d position);

    /// @brief Choose PID values for the ab joints.
    /// @param leg Leg being referenced using 0-index
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetAbGains(int leg, double kp, double ki, double kd);

    /// @brief Choose PID values for the shoulder joints.
    /// @param leg Leg being referenced using 0-index
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetShoulderGains(int leg, double kp, double ki, double kd);

    /// @brief Choose PID values for the knee joints.
    /// @param leg Leg being referenced using 0-index
    /// @param kp proportional gain
    /// @param ki integral gain
    /// @param kd derivative gain
    void SetKneeGains(int leg, double kp, double ki, double kd);

    /// @brief Joint frame position you want the ab to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetAbsDesiredPosition(int leg, double setpoint);

    /// @brief Joint frame position you want the shoulder to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetShoulderDesiredPosition(int leg, double setpoint);

    /// @brief Joint frame position you want the knee to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetKneeDesiredPosition(int leg, double setpoint);

    void DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_position);

    //void Probe_ground();

    /// @brief Call at the top of every sim loop timestep to update values from subscribers and to publishers, and handle service calls.
    void HandleROS();
/*
    void Populate();

    /// @brief Given the desired pose to spawn the robot, this function loads the correct URDF file based on the truss number given, and instantiates the motors, and their corresponding PID controllers. It also assigns the collision types for the contact bodies.
    /// @param position Position describing initial robot position. X is forward, Y is up, Z is right.
    /// @param orientation Quaternion describing initial robot orientation Y is up.
    void SpawnRobot(ChVector3d position, ChQuaternion<> orientation);
    
    /// @brief Changes the joint actuation type from position to FORCE and sets the PID gains for the controllers for all the knee joints.
    /// @param kp P gain for PID on the knees
    /// @param ki I gain for PID on the knees
    /// @param kd D gain for PID on the knees
    void EnableKnees(double kp, double ki, double kd);

    /// @brief Changes the joint actuation type from position to SPEED and sets the PID gains for the controllers for all the shoulder joints.
    /// @param kp P gain for PID on the shoulders
    /// @param ki I gain for PID on the shoulders
    /// @param kd D gain for PID on the shoulders
    void EnableShoulders(double kp, double ki, double kd);
    
    /// @brief Changes the joint actuation type from position to FORCE and sets the PID gains for the controllers for all the ab joints.
    /// @param kp P gain for PID on the abs
    /// @param ki I gain for PID on the abs
    /// @param kd D gain for PID on the abs
    void EnableAbs(double kp, double ki, double kd);

    /// @brief Changes the joint actuation type from position to FORCE on all the corresponding joints of the trusses and sets the PID gains for the prismatic joints on each truss.
    /// @param kp P gain for PID on the truss
    /// @param ki I gain for PID on the truss
    /// @param kd D gain for PID on the truss
    void EnableTrusses(double kp, double ki, double kd);

    /// @brief Changes the joint actuation type from position to FORCE on all the joints of the truss and sets the PID gains for the prismatic joint.
    /// @param truss_to_enable Truss being referenced using 0-index.
    /// @param kp P gain for PID on the truss
    /// @param ki I gain for PID on the truss
    /// @param kd D gain for PID on the truss
    void EnableTruss(int truss_to_enable, double kp, double ki, double kd);
    
    /// @brief Enables collisions for all of the collision bodies of the robot (body, and toes), and creates SCM moving patches for each of them.
    /// @param mterrain The SCM terrain
    void AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain);
   
    /// @brief Fixes the red box on the end of the truss to a provided location in space. This allows the truss to push and pull as if one end was fixed. 
    /// @param truss_to_set Truss being referenced using 0-index.
    /// @param position Position of the red box. Global frame
    void SetTrussEndPosition(int truss_to_set, ChVector3d position);

    //TODO Make this system type agnostic
    void DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChSystemSMC& sys);

    /// @brief Joint frame position you want the ab to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetAbsPositionPID(int leg, double setpoint);

    /// @brief Joint frame position you want the shoulder to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetShoulderPositionPID(int leg, double setpoint);

    /// @brief Joint frame position you want the knee to try to reach using PID.
    /// @param leg Leg being referenced using 0-index
    /// @param setpoint Desired setpoint in joint frame.
    void SetKneePositionPID(int leg, double setpoint);

    /// @brief Get the motor position in joint frame.
    /// @param motor Motor object that you want the motor position of
    /// @return Motor position in joint frame (radians).
    double getMotorRotationAngle(const std::shared_ptr<chrono::ChLinkMotor> motor);
    
    /// @brief Get the length of the truss.
    /// @param trussNum Truss being referenced using 0-index.
    /// @return Truss length in meters.
    double getTrussLength(int trussNum);
*/
private:

};

#endif