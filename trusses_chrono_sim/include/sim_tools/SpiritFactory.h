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

#include "sim_tools/Spirit.h"
#include "sim_tools/Truss.h"

using namespace chrono;

#ifndef SPIRITFACTORY_H
#define SPIRITFACTORY_H

class SpiritFactory {
    public:
        SpiritFactory(ChSystem* sys);
        std::unique_ptr<Spirit> BuildSpirit(const ChVector3d& Spirit_pos, const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions, const std::vector<ChVector3d>& probe_positions);

    private:
        std::shared_ptr<chrono::ChBody> BuildBody();
        std::pair<std::vector<std::shared_ptr<chrono::ChBody>>, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> BuildLeg(ChVector3d leg_pos, bool mirrored);
        std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> AttachLeg(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> leg_bodies, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors, bool mirrored);
        void RotateSpirit(std::shared_ptr<chrono::ChBody> body, std::vector<std::vector<std::shared_ptr<chrono::ChBody>>>& leg_bodies, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation);

        ChSystem* m_sys;
        int SpiritsBuilt = 0;
        ChVector3d m_Spirit_pos;

        // Define leg positions and orientations
        std::vector<ChVector3d> leg_positions = {
            ChVector3d(.2263, -0.098, 0), ChVector3d(.2263, 0.098, 0), 
            ChVector3d(-.2263, -0.098, 0), ChVector3d(-.2263, 0.098, 0)
        };

        ChVector3d m_abs_motor_offset = ChVector3d(0,-0.028,0);
        
        std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();

        ChColor darkgrey = ChColor(.1f, .1f, .1f);
        ChColor silver = ChColor(.3f, .3f, .3f);
        ChColor red = ChColor(.8f, .0f, .2f);

        //ChVector3d body_inertia = ChVector3d(.05,.1,.1);
        ChVector3d body_inertia = ChVector3d(.033,.059,.081);
        double body_mass = 5.75;

        //ChVector3d hip_inertia = ChVector3d(.00066963541,.0006963541,.0008696875);
        ChVector3d hip_inertia = ChVector3d(0.00074,0.00074,0.000145);
        double hip_mass = .575;

        ChVector3d m_upper_pos = ChVector3d(-0.103, 0.05098,0);
        //ChVector3d upper_inertia = ChVector3d(.00005,.001,.001);
        ChVector3d upper_inertia = ChVector3d(.000226,.0029,.0028);
        double upper_mass = .775;

        /*

        std::shared_ptr<chrono::ChTriangleMeshConnected> leg_trimesh;
        std::shared_ptr<chrono::ChCollisionShapeTriangleMesh> leg_collision_shape;
        std::shared_ptr<chrono::ChVisualShapeTriangleMesh> leg_vis_shape;

        std::string leg_mesh_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "Spirit_leg.stl";
        std::shared_ptr<chrono::ChContactMaterialSMC> leg_material = chrono_types::make_shared<ChContactMaterialSMC>(); //Standard Material

        ChVector3d m_leg_pos = ChVector3d(-0.206, 0.07298, 0);
        //ChVector3d leg_inertia = ChVector3d(.000417,.000417,.00000934);
        ChVector3d leg_inertia = ChVector3d(.0029,.0029,.000028);
        double leg_mass = 1;
        */

        ChVector3d m_lower_pos = ChVector3d(-0.103, 0.07298, 0);
        //ChVector3d lower_inertia = ChVector3d(.000005,.0001,.0001);
        //ChVector3d lower_inertia = ChVector3d(.000268,.000268,.00000105);
        ChVector3d lower_inertia = ChVector3d(.0029,.0029,.000028);
        double lower_mass = .2; 

        ChVector3d m_toe_pos = ChVector3d(0, 0.07298,0);
        //ChVector3d toe_inertia = ChVector3d(.00005,.00005,.00005);
        //ChVector3d toe_inertia = ChVector3d(.000008,.000008,.000008);
        ChVector3d toe_inertia = ChVector3d(.00029,.00029,.00029);
        double toe_mass = .1; 
};

#endif // RHEXFACTORY_H