//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_parsers/ChParserURDF.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

//Standard Packages
#include <cstdlib>

//Sim Tools
#include "sim_tools/HuskyFactory.h"
#include "sim_tools/Rover.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

// Global variable to track if termination signal (CTRL-c) was received
std::atomic_bool g_terminate_requested(false);

// Signal handler function
void signal_handler(int signum) {
  if (signum == SIGINT) {
    g_terminate_requested.store(true);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Termination signal (CTRL-c) received. Shutting down...");
  }
}

int main(int argc, char* argv[]) {
    //Initialize ROS
    rclcpp::init(argc,argv);

    // Set up signal handler for termination signals (CTRL-c)
    std::signal(SIGINT, signal_handler);

    SetChronoDataPath("/usr/local/share/chrono/data/");

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0,-9.81,0));
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    //Choose # of threads to use for chrono, collisions, and eigen.
    sys.SetNumThreads(4, 8, 1);
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys);

    //PSOR is a simplistic solver, while BARZILABORWEIN is second order precise but doesn't make much difference in performance and is slower.
    //sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
    double step_size = .0005; 

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));

    // Initialize the geometry of the soil
    double width = 5;
    double length = 5;
    double mesh_resolution = 0.02;
    mterrain.Initialize(length, width, mesh_resolution);
    // Constant soil properties
    mterrain.SetSoilParameters(0.2e8,  // Bekker Kphi
                                   0,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   30,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   4e9,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
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

    //Rover_position_W
    //p_WRh0_W or p_WRhset_W
    ChVector3d Husky_position = ChVector3d(0, 0.6, 0);

    //truss_positions_B
    //p_Rh0Tset_Rh0
    std::vector<ChVector3d> truss_positions = {
            ChVector3d(0, .2, -.2), ChVector3d(0, .2, .2), ChVector3d(0, .2, .5),
    };
    // Create a Rover robot
    HuskyFactory huskyFactory(&sys);

    auto husky0 = huskyFactory.BuildHusky(Husky_position, truss_positions);
    //std::vector<ChVector3d>{});
    husky0->AddMovingPatchesAndCollisions(mterrain);

    //auto rover1 = RoverFactory.BuildRover(ChVector3d(1,0.6,0), truss_positions);
    //rover1->AddMovingPatchesAndCollisions(mterrain);

    std::shared_ptr<ChVisualSystem> vis;
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("Load Husky Example");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(2.0, 1.4, 0.0), ChVector3d(0, .8, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);
    vis = vis_irr;

    std::cout << "Starting simulation loop" << std::endl;
    double time = 0;
    while (rclcpp::ok() && vis->Run()) {
        time = sys.GetChTime();
        std::cout << "Time: " << time << std::endl;
        //rover0->HandleROS();
        //rover1->HandleROS();
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        if(time > .4){
  
        //rover0->m_wheel_motors[0]->SetTorqueFunction(chrono_types::make_shared<ChFunctionConst>(10));
        husky0->SetWheelDesiredVelocity(0,0);
        husky0->SetWheelDesiredVelocity(1,0);
        husky0->SetWheelDesiredVelocity(2,0);
        husky0->SetWheelDesiredVelocity(3,0);
        }
        std::cout << "Torque: " << husky0->m_wheel_motors[0]->GetMotorTorque() << std::endl;
        std::cout << "Angle: " << husky0->m_wheel_motors[0]->GetMotorAngle() << std::endl;
        std::cout << "Speed: " << husky0->m_wheel_motors[0]->GetMotorAngleDt() << std::endl;
       
        vis->BindAll();
        sys.DoStepDynamics(step_size);

        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }
    rclcpp::shutdown();

    std::cout << "Simulation loop ended" << std::endl;
    return 0;
}