#include "chrono/core/ChRealtimeStep.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"

#include <eigen3/Eigen/Core>
#include "Eigen/Dense"
#include "Eigen/Sparse"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::ros;

// =============================================================================

int main(int argc, char* argv[]) {
    SetChronoDataPath("/usr/local/share/chrono/data/");
    const std::string out_dir = "/home/parallels/ros2_ws/src/chrono_sim/probe_output";

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0,-9.81,0));
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    //Choose # of threads to use for chrono, collisions, and eigen.
    sys.SetNumThreads(4, 8, 1);
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys);

    //PSOR is a simplistic solver, while BARZILABORWEIN is second order precise but doesn't make much difference in performance and is slower.
    sys.SetSolverType(ChSolver::Type::PSOR);
    //sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
    double step_size = .001; 

    //Setting gravity to zero helps ensure no external forces show themselves in the probe test
    sys.SetGravitationalAcceleration(ChVector3d(0,0,0));

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));

    // Initialize the geometry of the soil
    double width = .2;
    double length = .2;
    double mesh_resolution = 0.001;
    mterrain.Initialize(length, width, mesh_resolution);
    // Constant soil properties
    mterrain.SetSoilParameters(1898680.010933398,  // Bekker Kphi
                                   .1,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   31.5,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   300e6,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                   3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    
    //Visualize a mesh for the terrain
    mterrain.GetMesh()->SetWireframe(true);

    //Create the probing mechanism
    std::shared_ptr<ChBody> guide;
    std::shared_ptr<ChBody> probe;
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();

    //Create guide for probe
    guide = chrono_types::make_shared<ChBodyEasyBox>(.1, 2, 0.15, 1000, true, false, material);
    guide->SetPos(ChVector3d(-.11, 0, 0));
    guide->SetFixed(true);
    guide->GetVisualShape(0)->SetColor(ChColor(.25f,.25f,.25f));
    sys.Add(guide);

    //Create probe
    probe = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Y,.006, .3, 1000, material);
    probe->SetPos(ChVector3d(0, .15, 0));
    probe->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
    sys.Add(probe);

    // Create the linear motor
    auto motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

    //Initialize frame to describe the motor direction
    Eigen::Vector3d motorAxis(1, 0, 0);
    Eigen::Quaterniond motorQuat(Eigen::AngleAxisd(3.1415926/2, motorAxis));
    float qw = motorQuat.w();
    float qx = motorQuat.x();
    float qy = motorQuat.y();
    float qz = motorQuat.z();

    // Connect the guide and the slider and add the motor to the system:
    motor->Initialize(probe,                                // body A (slave)
                      guide,                       // body B (master)
                      ChFrame(ChVector3d(0,0,0),ChQuaternion<>(qw,qx,qy,qz))  // motor frame, in abs. coords
    );
    sys.Add(motor);

    // Create a ChFunction to be used for F(t) in ChLinkMotorLinearForce.
    auto mF = chrono_types::make_shared<ChFunctionConst>(.1);

    // Let the motor use this motion function:
    motor->SetSpeedFunction(mF);

    // Initialize logging output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
    }
    utils::ChWriterCSV csv(" ");

    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystem> vis;
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1440, 1080);
    vis_irr->SetWindowTitle("Probe Test");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(.2, 0, 0.0), ChVector3d(0, 0, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);
    vis = vis_irr;

    bool probing = true;
    double probe_position;
    
    while (probing && vis->Run()) {
        double time = sys.GetChTime();
        
        //Set camera to follow the probe
        vis->SetCameraPosition(ChVector3d(.1, 0-probe_position, 0.0));
        vis->SetCameraTarget(ChVector3d(0, 0-probe_position, 0.0));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        //Get the force of the terrain on the probe
        ChVector3d force;
        ChVector3d torque;
        mterrain.GetContactForceBody(probe, force, torque);

        //TODO Make this match the other data solutions

        //Probe displacement increases as it goes down, and is 0 when it contacts the ground.
        probe_position = -(probe->GetPos())[1]+.15;

        //Once the probe has penetrated 10cm
        if(probe_position > .1){

            //Stop the motor
            motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));

            probing = false;
        }


        std::cout << "Time: " << time << " Force: "<< force[1] << " Position: " <<probe_position << std::endl;

        //Save data to CSV
        csv << time << force << probe_position << std::endl;

        sys.DoStepDynamics(step_size);

    }

    csv.WriteToFile(out_dir + "/output.dat");

    return 0;
}