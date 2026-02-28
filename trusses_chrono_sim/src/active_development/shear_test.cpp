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
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));

    //Initialize SCM Terrain
    double length = .2;
    double width = .2;
    double mesh_resolution = 0.001; //.001
    mterrain.Initialize(length, width, mesh_resolution);

    mterrain.SetSoilParameters(3.9e6,  // Bekker Kphi
                                   0.1,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   31.5,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   300e6,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                   3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );

    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            40,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            50,   // number of erosion refinements per timestep
            20);  // number of concentric vertex selections subject to erosion

    //Create Probe
    std::shared_ptr<ChBody> sideways_guide;
    std::shared_ptr<ChBody> vertical_guide;
    std::shared_ptr<ChBody> probe;
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();

    vertical_guide = chrono_types::make_shared<ChBodyEasyBox>(.1, 2, 0.15, 1000, true, false, material);
    vertical_guide->SetPos(ChVector3d(-.11, 0, 0));
    vertical_guide->GetVisualShape(0)->SetColor(ChColor(.25f,.25f,.25f));
    sys.Add(vertical_guide);

    sideways_guide = chrono_types::make_shared<ChBodyEasyBox>(.1,.1,2,1000,true,false,material);
    sideways_guide->SetPos(ChVector3d(-.25, 0, 0));
    sideways_guide->SetFixed(true);
    sideways_guide->GetVisualShape(0)->SetColor(ChColor(.1f,.25f,.25f));
    sys.Add(sideways_guide);

    Eigen::Vector3d sideMotorAxis(0, 0, 1);
    Eigen::Quaterniond sideMotorQuat(Eigen::AngleAxisd(3.1415926/2, sideMotorAxis));

    auto sideways_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

    // Connect the guide and the slider and add the motor to the system:
    sideways_motor->Initialize(vertical_guide,                       // body A (slave)
                       sideways_guide,                       // body B (master)
                       ChFrame(ChVector3d(0,0,0),ChQuaternion<>(sideMotorQuat.w(),sideMotorQuat.x(),sideMotorQuat.y(),sideMotorQuat.z()))  // motor frame, in abs. coords
    );
    sys.Add(sideways_motor);

    // Let the motor use this motion function:
    sideways_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));


    probe = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Y,.006, .3, 1000, material);
    probe->SetPos(ChVector3d(0, .15, 0));
    probe->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.6f, 0.0f));
    sys.Add(probe);

    // Create the linear motor
    auto vertical_motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

    Eigen::Vector3d motorAxis(1, 0, 0);
    Eigen::Quaterniond motorQuat(Eigen::AngleAxisd(3.1415926/2, motorAxis));
    float qw = motorQuat.w();
    float qx = motorQuat.x();
    float qy = motorQuat.y();
    float qz = motorQuat.z();

    // Connect the guide and the slider and add the motor to the system:
    vertical_motor->Initialize(probe,                                // body A (slave)
                       vertical_guide,                       // body B (master)
                       ChFrame(ChVector3d(0,0,0),ChQuaternion<>(qw,qx,qy,qz))  // motor frame, in abs. coords
    );
    sys.Add(vertical_motor);

    // Create a ChFunction to be used for F(t) in ChLinkMotorLinearForce.
    auto mF = chrono_types::make_shared<ChFunctionConst>(.1);
    // Let the motor use this motion function:
    vertical_motor->SetSpeedFunction(mF);

    // Initialize logging output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
    }
    utils::ChWriterCSV csv(" ");
    

    std::shared_ptr<ChVisualSystem> vis;
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(800, 600);
    vis_irr->SetWindowTitle("Probe Test");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(.2, 0, 0.0), ChVector3d(0, 0, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);
    vis = vis_irr;

    bool probing = true;
    double probe_ypos;
    
    while (probing && vis->Run()) {
        double time = sys.GetChTime();

        //Set camera to follow the probe
        //vis->SetCameraPosition(ChVector3d(.2, 0, probe->GetPos()[2]));
        vis->SetCameraTarget(ChVector3d(0, 0, probe->GetPos()[2]));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        ChVector3d force;
        ChVector3d torque;
        mterrain.GetContactForceBody(probe, force, torque);

        //Probe displacement increases as it goes down, and is 0 when it contacts the ground.
        probe_ypos = -(probe->GetPos())[1]+.15;

        //If probe has reached 10cm
        if(probe_ypos > .1){

            //Stop vertical motor
            vertical_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
            //Start going sideways
            sideways_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(-.1));

            //Until probe has moved 5cm sideways
            if(probe->GetPos()[2] < -.05){

                //Stop sideways motor
                sideways_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
                probing=false;
            }
        }

        std::cout << "\nTime: " << time << std::endl;
        std::cout << "Force: " << force << std::endl;
        std::cout << "Position: " << probe->GetPos()[0] <<" "<< probe_ypos << " " << probe->GetPos()[2] << std::endl;


        csv << time << force << probe_ypos << -probe->GetPos()[2] << std::endl;

        sys.DoStepDynamics(step_size);
    }

    csv.WriteToFile(out_dir + "/output.dat");

    return 0;
}