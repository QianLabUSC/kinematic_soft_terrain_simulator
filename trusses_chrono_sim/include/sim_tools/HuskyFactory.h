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

#ifndef HUSKYFACTORY_H
#define HUSKYFACTORY_H

class HuskyFactory {
    public:
        HuskyFactory(ChSystem* sys);
        std::unique_ptr<Rover> BuildHusky(const ChVector3d& Husky_pos, const std::vector<ChVector3d>& truss_positions);

    private:
        ChSystem* m_sys;
        int HuskiesBuilt = 0;
        ChVector3d m_Husky_pos;
        std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();

        std::shared_ptr<chrono::ChTriangleMeshConnected> wheel_trimesh;
        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> wheel_collision_shape;
        std::shared_ptr<chrono::ChVisualShapeTriangleMesh> wheel_vis_shape;

        // Define leg spositions and orientations
        std::vector<ChVector3d> wheel_positions = {
            ChVector3d(.165225, 0, .153545), ChVector3d(-.165225, 0, 0.153545), 
            ChVector3d(.165225, 0, -.153545), ChVector3d(-.165225, 0, -.153545)
        };

        std::vector<ChQuaternion<>> wheel_orientations = {
            QuatFromAngleZ(-CH_PI_2), QuatFromAngleZ(-CH_PI_2), 
            QuatFromAngleZ(-CH_PI_2), QuatFromAngleZ(-CH_PI_2)
        };

        double wheel_mass = .39; //kg, needs calibration.
        chrono::ChMatrix33<double> wheel_inertia = ChMatrix33(ChVector3d(.00828,0.0,.0037192), ChVector3d(0.0,.0118,0.0), ChVector3d(.0037192,0.0,.003709)); //kg*m^2, needs calibration.
        std::string wheel_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "Rover_wheel.obj";
        std::shared_ptr<chrono::ChContactMaterialSMC> wheel_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material

        std::shared_ptr<chrono::ChBody> BuildBody();
        std::shared_ptr<chrono::ChBody> BuildWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num);
        std::shared_ptr<chrono::ChBody> BuildSimpleWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num);
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> AttachWheels(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& wheels);
        ChColor darkgrey = ChColor(.1f, .1f, .1f);
        double body_mass = 5.75;
        ChVector3d body_inertia = ChVector3d(.05,.1,.1);

};

#endif // RHEXFACTORY_H