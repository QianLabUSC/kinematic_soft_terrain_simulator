#include <string>
#include <stdexcept>
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"
#include "chrono/physics/ChSystemSMC.h"

#include "sim_tools/PIDController.h"
#include "sim_tools/Truss.h"
#include "sim_tools/ROSRHex.h"

using namespace chrono;

#ifndef RHEX_H
#define RHEX_H

class RHex {
    public:
        std::shared_ptr<ROSRHex> node;

        ChSystem* m_sys;
        std::shared_ptr<chrono::ChBody> m_body;
        std::vector<std::shared_ptr<chrono::ChBody>> m_legs;
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> m_leg_motors;
        std::vector<Truss> m_trusses;

        std::vector<PIDController> pidControllers;

        RHex(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> legs, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors, std::vector<Truss> trusses, int rhexNum);
        
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
        /// @param relative_docking_pos Position of the truss connection relative to the robot body.
        void DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_docking_pos);

        /// @brief Choose PID values for the leg joints.
        /// @param kp proportional gain
        /// @param ki integral gain
        /// @param kd derivative gain
        void SetLegGains(double kp, double ki, double kd);

        /// @brief Joint position you want the leg to try to reach using PID.
        /// @param leg Leg being referenced using 0-index.
        /// @param setpoint Desired position (rad)
        void SetLegDesiredPosition(int leg, double setpoint);

        /// @brief Joint velocity you want the leg to try to reach using PID.
        /// @param leg Leg being referenced using 0-index.
        /// @param setpoint Desired velocity (rad/s)
        void SetLegDesiredVelocity(int leg, double setpoint);

        /// @brief Call at the top of every sim loop timestep to update values from subscribers and to publishers, and handle service calls.
        void HandleROS();


    private:

        
};

#endif // RHEX_H