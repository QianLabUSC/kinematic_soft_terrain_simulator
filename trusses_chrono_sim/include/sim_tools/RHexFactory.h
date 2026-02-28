#include <string>
#include <filesystem>

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "sim_tools/RHex.h"
#include "sim_tools/Truss.h"

using namespace chrono;

#ifndef RHEXFACTORY_H
#define RHEXFACTORY_H

class RHexFactory {
    public:
        RHexFactory(ChSystem* sys);
        std::unique_ptr<RHex> BuildRHex(const ChVector3d& RHex_pos, const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions, double bodymass, double legmass, double counterweight_mass = 0.0);

    private:
        ChSystem* m_sys;
        int RHexsBuilt = 0;
        ChVector3d m_RHex_pos;

        double body_mass = 20; //kg, needs calibration.
        ChVector3d body_inertia = ChVector3d(1.84,2.456,.77097); //kg*m^2, needs calibration. & changed to 3x3 matrix.
        std::string body_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "RHex_body.stl";
        //std::string body_mesh_path = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/RHex_body.stl";
        std::shared_ptr<chrono::ChContactMaterialSMC> body_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material

        std::shared_ptr<chrono::ChTriangleMeshConnected> body_trimesh = ChTriangleMeshConnected::CreateFromSTLFile(body_mesh_path);
        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> body_collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(body_material, body_trimesh, false, false, 0.01);

        std::shared_ptr<chrono::ChTriangleMeshConnected> leg_trimesh;
        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> leg_collision_shape;
        std::shared_ptr<chrono::ChVisualShapeTriangleMesh> leg_vis_shape;

        // Define leg positions and orientations
        float z_offset = 0.00;
        std::vector<ChVector3d> leg_positions = {
            ChVector3d(0, .36, z_offset), ChVector3d(0, -.36, z_offset), ChVector3d(0.40365, .28, z_offset),
            ChVector3d(0.40365, -.28, z_offset), ChVector3d(-0.40365, .28, z_offset), ChVector3d(-0.40365, -.28, z_offset)
        };

        std::vector<ChQuaternion<>> leg_orientations = {
            QuatFromAngleY(CH_PI), QuatFromAngleY(CH_PI), QuatFromAngleY(CH_PI),
            QuatFromAngleY(CH_PI), QuatFromAngleY(CH_PI), QuatFromAngleY(CH_PI)
        };


        double leg_mass = .39; //kg, needs calibration.
        chrono::ChMatrix33<double> leg_inertia = ChMatrix33(ChVector3d(.00828,0.0,.0037192), ChVector3d(0.0,.0118,0.0), ChVector3d(.0037192,0.0,.003709)); //kg*m^2, needs calibration.
        std::string leg_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "RHex_leg.stl";
        std::shared_ptr<chrono::ChContactMaterialSMC> leg_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material
        void SetMass(double bodym, double legm);
        void RotateRHex(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>>& legs, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation); // New function
        std::shared_ptr<chrono::ChBody> BuildBody();
        std::shared_ptr<chrono::ChBody> BuildSimpleBody(double counterweight_mass = 0.0);
        std::shared_ptr<chrono::ChBody> BuildLeg(const ChVector3d& pos, const ChQuaternion<>& rot, int leg_num);
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> AttachLegs(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& legs);
        
};

#endif // RHEXFACTORY_H