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
#include "sim_tools/RoverFactory.h"
#include "sim_tools/Rover.h"
#include "sim_tools/VariableSoilParams.h"

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
    double width = 10;
    double length = 10;
    double mesh_resolution = 0.02;
    mterrain.Initialize(length, width, mesh_resolution);

    //Variable soil properties
    std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/map.csv";
    std::string texture_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/Map.png";

    auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath);
    mterrain.RegisterSoilParametersCallback(my_params);

    //Showing map texture
    mterrain.SetTexture(texture_map_filepath);

    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    
    //Visualize a mesh for the terrain
    mterrain.GetMesh()->SetWireframe(true);
    //add rover?
    //Rover_position_W
    //p_WRh0_W or p_WRhset_W
    ChVector3d Rover_position = ChVector3d(0, 0, 0);

    //truss_positions_B
    //p_Rh0Tset_Rh0
    std::vector<ChVector3d> truss_positions = {
            ChVector3d(0, .2, -.2), ChVector3d(0, .2, .2), ChVector3d(0, .2, .5),
    };
    // Create a Rover robot
    RoverFactory RoverFactory(&sys);

    auto rover0 = RoverFactory.BuildRover(Rover_position, QUNIT, std::vector<ChVector3d>{});
    rover0->AddMovingPatchesAndCollisions(mterrain);


    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystem> vis;
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(800, 600);
    vis_irr->SetWindowTitle("Load Spirit Example");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(2.0, 1.4, 0.0), ChVector3d(0, .8, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);
    vis = vis_irr;

    while (rclcpp::ok() && vis->Run()) {
        double time = sys.GetChTime();

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        if(time > .4){
  
        //rover0->m_wheel_motors[0]->SetTorqueFunction(chrono_types::make_shared<ChFunctionConst>(10));
        rover0->SetWheelDesiredVelocity(0,0);
        rover0->SetWheelDesiredVelocity(1,0);
        rover0->SetWheelDesiredVelocity(2,0);
        rover0->SetWheelDesiredVelocity(3,0);
        }
        std::cout << "Torque: " << rover0->m_wheel_motors[0]->GetMotorTorque() << std::endl;
        std::cout << "Angle: " << rover0->m_wheel_motors[0]->GetMotorAngle() << std::endl;
        std::cout << "Speed: " << rover0->m_wheel_motors[0]->GetMotorAngleDt() << std::endl;
       
        vis->BindAll();
        //sys.DoStepDynamics(step_size);

        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }

        //This is what moves us to the next timestep, call it last.
        sys.DoStepDynamics(step_size);
    }
    rclcpp::shutdown();
    return 0;
}