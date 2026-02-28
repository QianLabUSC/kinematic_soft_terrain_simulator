//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

//ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

//Standard Packages
#include <cstdlib>

//Custom Classes
#include "sim_tools/SpiritFactory.h"
#include "sim_tools/Spirit.h"
#include "sim_tools/Truss.h"
#include "sim_tools/SimpleMovingAverage.h"

using namespace chrono;
using namespace chrono::irrlicht;

// =============================================================================
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
    sys.SetSolverType(ChSolver::Type::PSOR);
    //sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
    double step_size = .001; 

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));

    // Initialize the geometry of the soil
    double width = 1.5;
    double length = 1.5;
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

    ChVector3d Spirit_position = ChVector3d(0, 0.05, -.3);

    std::vector<ChVector3d> probe_positions = {ChVector3d(0.3, 0.1, -0.1)};


    //Leave empty if you want no trusses
    std::vector<ChVector3d> truss_positions = {
        ChVector3d(0, .1, 0.1)
    };

    // Create a Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    //Build a spirit robot
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, Q_ROTATE_Z_TO_Y, truss_positions, probe_positions);

    //Turn on collisions with the terrain and create moving patches for the robot
    spirit0->AddMovingPatchesAndCollisions(mterrain);
    spirit0->LockTrussEndPosition(0, ChVector3d(0, 1, 0));

    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystemIrrlicht> vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("Load Spirit Example");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(2.0, 1.4, 0.0), ChVector3d(0, .8, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);

    while (rclcpp::ok() && vis_irr->Run()) {
        double sim_time = sys.GetChTime();

        vis_irr->BeginScene();
        vis_irr->Render();
        vis_irr->EndScene();

        spirit0->SetAbsDesiredPosition(0,0);
        spirit0->SetAbsDesiredPosition(1,0);
        spirit0->SetAbsDesiredPosition(2,0);
        spirit0->SetAbsDesiredPosition(3,0);

        //Slowly increment knee setpoint from 0 to a final setpoint of -1.
        double setpoint_knee = -(sim_time)/.5;
        if(setpoint_knee < -1){
            setpoint_knee = -1;
        }

        //Slowly increment shoulder setpoint from 0 to a final setpoint of -.4.
        double setpoint_shoulder = -(sim_time)/.5;
        if(setpoint_shoulder < -.4){
            setpoint_shoulder = -.4;
        }


        //Update PID and send new controlSignal to shoulders
        spirit0->SetShoulderDesiredPosition(0,setpoint_shoulder);
        spirit0->SetShoulderDesiredPosition(1,setpoint_shoulder);
        spirit0->SetShoulderDesiredPosition(2,setpoint_shoulder);
        spirit0->SetShoulderDesiredPosition(3,setpoint_shoulder);

        //Updte PID and send new controlSignal to knees
        spirit0->SetKneeDesiredPosition(0,setpoint_knee);
        spirit0->SetKneeDesiredPosition(1,setpoint_knee);
        spirit0->SetKneeDesiredPosition(2,setpoint_knee);
        spirit0->SetKneeDesiredPosition(3,setpoint_knee);

        

        //This is what moves us to the next timestep, call it last
        sys.DoStepDynamics(step_size);

        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }

    //Cleanly shut down ROS after the while loop
    rclcpp::shutdown();
    return 0;
}