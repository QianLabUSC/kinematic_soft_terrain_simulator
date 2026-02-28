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

#include "sim_tools/Rover.h"
#include "sim_tools/Truss.h"

using namespace chrono;


#ifndef ROVERFACTORY_H
#define ROVERFACTORY_H

class RoverFactory {
    public:
        RoverFactory(ChSystem* sys);
        std::unique_ptr<Rover> BuildRover(const ChVector3d& Rover_pos, const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions);

    private:
        ChSystem* m_sys;
        int RoversBuilt = 0;
        ChVector3d m_Rover_pos;

        double body_mass = 20; //kg, needs calibration.
        ChVector3d body_inertia = ChVector3d(1.84,2.456,.77097); //kg*m^2, needs calibration. & changed to 3x3 matrix.
        std::string body_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "Rover_body.stl";
        std::string texture_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "black_carbon";
        std::shared_ptr<chrono::ChContactMaterialSMC> body_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material

        std::shared_ptr<ChTriangleMeshConnected> body_trimesh = ChTriangleMeshConnected::CreateFromSTLFile(body_mesh_path);
        

        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> body_collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(body_material, body_trimesh, false, false, 0.01);

        std::shared_ptr<chrono::ChTriangleMeshConnected> wheel_trimesh;
        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> wheel_collision_shape;
        std::shared_ptr<chrono::ChVisualShapeTriangleMesh> wheel_vis_shape;

        // Define leg positions and orientations
        std::vector<ChVector3d> wheel_positions = {
            ChVector3d(0.40365, .28, 0), ChVector3d(0.40365, -.28, 0), 
            ChVector3d(-0.40365, .28, 0), ChVector3d(-0.40365, -.28, 0)
        };

        std::vector<ChQuaternion<>> wheel_orientations = {
            QuatFromAngleY(-CH_PI_2), QuatFromAngleY(-CH_PI_2), 
            QuatFromAngleY(-CH_PI_2), QuatFromAngleY(-CH_PI_2)
        };

        double wheel_mass = .39; //kg, needs calibration.
        chrono::ChMatrix33<double> wheel_inertia = ChMatrix33(ChVector3d(.00828,0.0,.0037192), ChVector3d(0.0,.0118,0.0), ChVector3d(.0037192,0.0,.003709)); //kg*m^2, needs calibration.
        std::string wheel_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "Rover_wheel.obj";
        std::shared_ptr<chrono::ChContactMaterialSMC> wheel_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material

        void RotateRover(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>>& wheels, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation);
        std::shared_ptr<chrono::ChBody> BuildBody();
        std::shared_ptr<chrono::ChBody> BuildSimpleBody();
        std::shared_ptr<chrono::ChBody> BuildWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num);
        std::shared_ptr<chrono::ChBody> BuildSimpleWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num);
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> AttachWheels(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& wheels);
        
};

#endif // RHEXFACTORY_H