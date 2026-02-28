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
#include "sim_tools/VariableSoilParams.h"
//Standard Packages
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
//Sim Tools
#include "sim_tools/RHexFactory.h"
#include "sim_tools/RHex.h"
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
double test(int argc, char* argv[], double bodymass, double legmass, std::string stiffness, double stiffness_scale) {
 //Initialize ROS
 rclcpp::init(argc,argv);

 // Set up signal handler for termination signals (CTRL-c)
 std::signal(SIGINT, signal_handler);


 SetChronoDataPath("/usr/local/share/chrono/data/");

 // Create a Chrono::Engine physical system
 ChSystemSMC sys;
 sys.SetGravitationalAcceleration(chrono::ChVector3d(0,0,-9.81));
 auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
 //Choose # of threads to use for chrono, collisions, and eigen.
 sys.SetNumThreads(4, 8, 1);
 collsys->SetNumThreads(4);
 sys.SetCollisionSystem(collsys);

 //PSOR is a simplistic solver, while BARZILABORWEIN is second order precise but doesn't make much difference in performance and is slower.
 //sys.SetSolverType(ChSolver::Type::PSOR);
 sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

 //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
 double step_size = .004; 

 // Create the 'deformable terrain' object
 vehicle::SCMTerrain mterrain(&sys);

 // Initialize the geometry of the soil
 double width = 4;
 double length = 6;
 double mesh_resolution = 0.02;

 mterrain.Initialize(length, width, mesh_resolution);
 mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
 mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    
    //Visualize a mesh for the terrain
 std::cout << "loading in stiffness" << std::endl;
 //Variable soil properties
 std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/output_stiffness.csv";
 //std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/constant_stiffness_1.csv";

 //std::string texture_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/Map.png";
 std::cout << "making params" << std::endl;
 double scalefactor = stiffness_scale;
 auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath,scalefactor);
 std::cout << "registering soil parameters" << std::endl;
 mterrain.RegisterSoilParametersCallback(my_params);

 //Showing map texture
 //mterrain.SetTexture(texture_map_filepath);

 // Constant soil properties
 /*
 mterrain.SetSoilParameters(0.2e7,  // Bekker Kphi
                                0,      // Bekker Kc
                                1.1,    // Bekker n exponent
                                0,      // Mohr cohesive limit (Pa)
                                30,     // Mohr friction limit (degrees)
                                0.01,   // Janosi shear coefficient (m)
                                0.3e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
 );
*/
 mterrain.EnableBulldozing(true);  // inflate soil at the border of the rut
 mterrain.SetBulldozingParameters(
         55,  // angle of friction for erosion of displaced material at the border of the rut
         1,   // displaced material vs downward pressed material.
         5,   // number of erosion refinements per timestep
         6);  // number of concentric vertex selections subject to erosion
 
 //Visualize a mesh for the terrain
 mterrain.GetMesh()->SetWireframe(true);

 //Rhex_position_W
 //p_WRh0_W or p_WRhset_W
 ChVector3d RHex_position = ChVector3d(-2, 0, 0.075);

 //truss_positions_B
 //p_Rh0Tset_Rh0
 //std::vector<ChVector3d> truss_positions = {
 //        ChVector3d(0, -.2, .2), ChVector3d(0, .2, .2), ChVector3d(0, .5, .2),
 //};
 std::vector<ChVector3d> truss_positions = {};
 // Create a RHex robot
 RHexFactory RHexFactory(&sys);
 //double bodymass=20;
 //double legmass=.39;

 auto Rhex = RHexFactory.BuildRHex(RHex_position, QUNIT, truss_positions,bodymass,legmass);

 // Set initial leg positions with 0.5π phase difference
 Rhex->SetLegDesiredPosition(0, 0.0);           // Tripod 1 leg 0
 Rhex->SetLegDesiredPosition(1, 0.0);           // Tripod 1 leg 1  
 Rhex->SetLegDesiredPosition(3, 0.0);           // Tripod 1 leg 3
 Rhex->SetLegDesiredPosition(2, 0.0);    // Tripod 2 leg 2 (0.5π phase difference)
 Rhex->SetLegDesiredPosition(5, 0.0);    // Tripod 2 leg 5 (0.5π phase difference)
 Rhex->SetLegDesiredPosition(4, 0.0);    // Tripod 2 leg 4 (0.5π phase difference)
 
 Rhex->AddMovingPatchesAndCollisions(mterrain);

 // Compute and log body COG and whole-robot COM
 auto computeRobotCOM = [&]() {
     double total_m = 0.0;
     ChVector3d weighted_sum(0, 0, 0);

     const ChVector3d body_pos = Rhex->m_body->GetPos();
     const double body_m = Rhex->m_body->GetMass();
     weighted_sum += body_pos * body_m;
     total_m += body_m;

     for (const auto& leg : Rhex->m_legs) {
         const ChVector3d p = leg->GetPos();
         const double m = leg->GetMass();
         weighted_sum += p * m;
         total_m += m;
     }
     return weighted_sum / total_m;
 };
 const ChVector3d body_cog_init = Rhex->m_body->GetPos();
 const ChVector3d robot_com_init = computeRobotCOM();
 std::cout << "[Init] Body COG: " << body_cog_init << "  Robot COM: " << robot_com_init << std::endl;

 //Rhex->LockTrussEndPosition(0, ChVector3d(0, 1, -.4));
 //Rhex->LockTrussEndPosition(1, ChVector3d(0, 1, .4));
 //Rhex->m_body->SetFixed(true);

 std::shared_ptr<ChVisualSystem> vis;
 auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
 vis_irr->SetWindowSize(1600, 1200);
 vis_irr->SetWindowTitle("Load Rhex Example");
 vis_irr->SetCameraVertical(CameraVerticalDir::Z);
 vis_irr->Initialize();
 vis_irr->AddSkyBox();
 vis_irr->AddTypicalLights();
 vis_irr->AddCamera(ChVector3d(0, -2, 1.4), ChVector3d(0, 0, .8));
 vis_irr->EnableShadows();
 vis_irr->AttachSystem(&sys);
 vis = vis_irr;

 std::cout << "Starting simulation loop" << std::endl;
 double simulation_time = sys.GetChTime();

 int index = 0;
 double leg_movement_start_time = 0.3; // Start leg movement after 0.3 seconds
 bool leg_movement_started = false;
 
 // Tripod gait parameters
 double gait_frequency = 2.0; // Hz - cycles per second (ω = 2π * gait_frequency rad/s)
 double gait_phase = 0.0; // Current phase of the gait cycle

 
 while (rclcpp::ok() && vis->Run()) {
     Rhex->HandleROS();
     vis->BeginScene();
     vis->Render();
     vis->EndScene();
     
     // Check if it's time to start leg movement
     if (simulation_time >= leg_movement_start_time && !leg_movement_started) {
         std::cout << "Starting leg movement at time: " << simulation_time << std::endl;
         leg_movement_started = true;
     }
     
     // Update gait phase - linear with time
     if (leg_movement_started) {
         gait_phase = (simulation_time - leg_movement_start_time) * gait_frequency * 2.0 * M_PI;
     }
     
     // Calculate desired leg positions based on gait phase
     if (!leg_movement_started) {
         // No leg movement - robot just sits there
         for (int i = 0; i < 6; i++) {
             Rhex->SetLegDesiredPosition(i, Rhex->m_leg_motors[i]->GetMotorAngle()); // Hold current position
         }
     } else {
         // Continuous rotation gait - forward motion
         // Tripod 1: legs 0, 1, 3
         // Tripod 2: legs 2, 5, 4
         
         // Calculate continuous rotation positions
         double time_since_start = simulation_time - leg_movement_start_time;
         
         // Tripod 1 legs (0, 1, 3) - continuous rotation
         double tripod1_pos = -gait_frequency * 2.0 * M_PI * time_since_start;
         double tripod1_initial = 0.0; // Initial position for tripod 1
         
         // Only move if calculated position is smaller (more negative) than initial
         if (tripod1_pos <= tripod1_initial) {
             Rhex->SetLegDesiredPosition(4, tripod1_pos);
             Rhex->SetLegDesiredPosition(1, tripod1_pos);
             Rhex->SetLegDesiredPosition(2, tripod1_pos);
         } else {
             // Hold at initial position
             Rhex->SetLegDesiredPosition(4, tripod1_initial);
             Rhex->SetLegDesiredPosition(1, tripod1_initial);
             Rhex->SetLegDesiredPosition(2, tripod1_initial);
         }
         
         // Tripod 2 legs (2, 5, 4) - continuous rotation with 0.5π phase difference
         double tripod2_pos = -gait_frequency * 2.0 * M_PI * time_since_start + 0.5 * M_PI;
         double tripod2_initial = 0; // Initial position for tripod 2 (with phase difference)
         
         // Only move if calculated position is smaller (more negative) than initial
         if (tripod2_pos <= tripod2_initial) {
             Rhex->SetLegDesiredPosition(0, tripod2_pos);
             Rhex->SetLegDesiredPosition(3, tripod2_pos);
             Rhex->SetLegDesiredPosition(5, tripod2_pos);
         } else {
             // Hold at initial position
             Rhex->SetLegDesiredPosition(0, tripod2_initial);
             Rhex->SetLegDesiredPosition(3, tripod2_initial);
             Rhex->SetLegDesiredPosition(5, tripod2_initial);
         }
     }

     std::cout << "Torque: " << Rhex->m_leg_motors[0]->GetMotorTorque() << std::endl;
     std::cout << "Angle: " << Rhex->m_leg_motors[0]->GetMotorAngle() << std::endl;
     std::cout << "Speed: " << Rhex->m_leg_motors[0]->GetMotorAngleDt() << std::endl;
     std::cout << "position: " << Rhex->m_body->GetPos() << std::endl;
     if (index % 50 == 0) {
         const ChVector3d body_cog = Rhex->m_body->GetPos();
         const ChVector3d robot_com = computeRobotCOM();
         std::cout << "[Step " << index << "] Body COG: " << body_cog
                   << "  Robot COM: " << robot_com << std::endl;
     }
     vis->BindAll();
     sys.DoStepDynamics(step_size);
     index++;
     simulation_time = sys.GetChTime();
     std::cout << "Total Simulation time: " << simulation_time << std::endl;
     if((Rhex->m_body->GetPos()[0])>=2) {
      simulation_time = sys.GetChTime();
      std::cout << "Total Simulation time: " << simulation_time << std::endl;
      std::cout << "average velocity: " << (Rhex->m_body->GetPos()[0])/(simulation_time) << std::endl;
      break;
      return ((Rhex->m_body->GetPos()[0])/(simulation_time));
      break;
     }
     //Handles CTRL-C quitting nicely
     if (g_terminate_requested.load()) {
     break;
     }
 }
 rclcpp::shutdown();
 return ((Rhex->m_body->GetPos()[0])/(simulation_time));
 std::cout << "Simulation loop ended" << std::endl;
 return 0;
}

int main(int argc, char* argv[]) {
    //this will run a test with the given body mass and leg mass. 
    //Stiffness is taken from the filepath specified in test, and scaled by the factor below
    //to change soil parameters besides stiffness, please see VariableSoilParams.cpp
     double bodymass=27.4 + 150;
    //  double legmass=3.9;
     double legmass=.39;
     double stiffness_scale= 1.0; //This is the scale factor for the stiffness, it is multiplied by the stiffness value in the csv file.
     std::string stiffness = "50";
     double velocity = test(argc, argv, bodymass, legmass, stiffness, stiffness_scale);
     return 0;
    

}