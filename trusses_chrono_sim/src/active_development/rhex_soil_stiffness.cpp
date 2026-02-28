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
#include <utility>
//Sim Tools
#include "sim_tools/RHexFactory.h"
#include "sim_tools/RHex.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;
//README:
//This file iterates through a variety of tests with varying simulation properties, including 
// Rhex weight and soil stiffness, recording valuable data for each test to the file "rhex_results"

// Global variable to track if termination signal (CTRL-c) was received
std::atomic_bool g_terminate_requested(false);

// Signal handler function
void signal_handler(int signum) {
  if (signum == SIGINT) {
    g_terminate_requested.store(true);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Termination signal (CTRL-c) received. Shutting down...");
  }
}
std::pair<double,double> test(int argc, char* argv[], double bodymass, double legmass, std::string stiffness) {
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
 double width = 10;
 double length = 10;
 double mesh_resolution = 0.02;

 mterrain.Initialize(length, width, mesh_resolution);

 std::cout << "loading in stiffness" << std::endl;
 //Variable soil properties
 //std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/output_stiffness.csv";
 std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/constant_stiffness_"+stiffness+".csv";

 //std::string texture_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/Map.png";
 std::cout << "making params" << std::endl;
 double scalefactor=10.0;
 auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath,1.0);
 std::cout << "registering soil parameters" << std::endl;
 mterrain.RegisterSoilParametersCallback(my_params);

 //Showing map texture
 //mterrain.SetTexture(texture_map_filepath);
// /*
 // Constant soil properties
 mterrain.SetSoilParameters( 0.2e7 * scalefactor,  // Bekker Kphi
                                0,      // Bekker Kc
                                1.1,    // Bekker n exponent
                                0,      // Mohr cohesive limit (Pa)
                                30 * scalefactor,     // Mohr friction limit (degrees)
                                0.001,   // Janosi shear coefficient (m)
                                0.3e7 * scalefactor,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
 );
// */
 mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
 mterrain.SetBulldozingParameters(
         55,  // angle of friction for erosion of displaced material at the border of the rut
         1,   // displaced material vs downward pressed material.
         5,   // number of erosion refinements per timestep
         6);  // number of concentric vertex selections subject to erosion
 
 //Visualize a mesh for the terrain
 mterrain.GetMesh()->SetWireframe(true);

 //Rhex_position_W
 //p_WRh0_W or p_WRhset_W
 ChVector3d RHex_position = ChVector3d(0, 0, 0.4);

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

 
 Rhex->AddMovingPatchesAndCollisions(mterrain);

 //Rhex->LockTrussEndPosition(0, ChVector3d(0, 1, -.4));
 //Rhex->LockTrussEndPosition(1, ChVector3d(0, 1, .4));
 //Rhex->m_body->SetFixed(true);

 std::shared_ptr<ChVisualSystem> vis;
 auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
 vis_irr->SetWindowSize(1600, 1200);
 vis_irr->SetWindowTitle("Load Spirit Example");
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
 double z_threshold = 0.2;
 bool was_touching = false;
 std::vector<double> step_positions;
 double last_step_x = 0.0;
 int index = 0;
 while (rclcpp::ok() && vis->Run()) {
     Rhex->HandleROS();
     vis->BeginScene();
     vis->Render();
     vis->EndScene();

     Rhex->SetLegDesiredVelocity(0,-2);
     Rhex->SetLegDesiredVelocity(1,-2);
     Rhex->SetLegDesiredVelocity(2,-2);
     Rhex->SetLegDesiredVelocity(3,-2);
     Rhex->SetLegDesiredVelocity(4,-2);
     Rhex->SetLegDesiredVelocity(5,-2);

     std::cout << "Torque: " << Rhex->m_leg_motors[0]->GetMotorTorque() << std::endl;
     std::cout << "Angle: " << Rhex->m_leg_motors[0]->GetMotorAngle() << std::endl;
     std::cout << "Speed: " << Rhex->m_leg_motors[0]->GetMotorAngleDt() << std::endl;
     std::cout << "position: " << Rhex->m_body->GetPos() << std::endl;
     vis->BindAll();
     sys.DoStepDynamics(step_size);
     index++;
     
     double z_now = Rhex->m_body->GetPos().z();
     double x_now = Rhex->m_body->GetPos().x();

// Detect belly contact event
     if (z_now < z_threshold && !was_touching) {
      step_positions.push_back(x_now);
      was_touching = true;
      } 
     else if (z_now >= z_threshold) {
      was_touching = false;
      }

     if((Rhex->m_body->GetPos()[0])>=1) {
      simulation_time = sys.GetChTime();
      std::cout << "Total Simulation time: " << simulation_time << std::endl;
      std::cout << "average velocity: " << (Rhex->m_body->GetPos()[0])/(simulation_time) << std::endl;
      // Compute average step length
      double total_step = 0.0;
      int step_count = 0;
      for (size_t i = 1; i < step_positions.size(); ++i) {
        total_step += step_positions[i] - step_positions[i - 1];
        step_count++;
      }
      double avg_step = (step_count > 0) ? (total_step / step_count) : 0.0;
      std::cout << "Average Step Size: " << avg_step << std::endl;
      double avg_speed = double(Rhex->m_body->GetPos()[0]) / (simulation_time);
      return std::make_pair(avg_speed,avg_step);
      break;
     }
     //Handles CTRL-C quitting nicely
     if (g_terminate_requested.load()) {
     break;
     }
 }
 rclcpp::shutdown();
 return std::make_pair(0.0,0.0);
 std::cout << "Simulation loop ended" << std::endl;
 return std::make_pair(0.0,0.0);
}
//n/m
//check step length
int main(int argc, char* argv[]) {
  /*
     double bodymass=20;
     double legmass=.39;
     std::string stiffness = "2000";
     double velocity = test(argc, argv, bodymass, legmass, stiffness);
     return 0;
    */
    // Vector of pairs: each pair contains (body_mass, leg_mass)
    std::vector<std::pair<double, double>> mass_pairs = {
      //{20.0, 0.35},
      //{25.0, 0.39},
      {30.0, 0.45}
  };
  
  

  // List of stiffness values as strings
  //std::vector<std::string> stiffness_values = {"1000", "1500", "2000"};
  std::vector<std::string> stiffness_values = {"1000"};
  // Open the output CSV file
  std::string outputfile = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/rhex_results.csv";
  std::ofstream output_file(outputfile);
  if (!output_file.is_open()) {
      std::cerr << "Failed to open results.csv for writing." << std::endl;
      return 1;
  }

  // Write the header row
  output_file << "BodyMass, LegMass, Stiffness, Velocity, Average Step Size\n";

  // Iterate through each mass pair and stiffness value
  for (const auto& mass_pair : mass_pairs) {
      double body_mass = mass_pair.first;
      double leg_mass = mass_pair.second;

      for (const auto& stiffness : stiffness_values) {
          auto [velocity, avg_step] = test(argc, argv, body_mass, leg_mass, stiffness);
          rclcpp::shutdown();
          output_file << body_mass << "," << leg_mass << "," << stiffness << "," << velocity << "," <<avg_step<<"\n";
      }
  }

  // Close the output file
  output_file.close();
  return 0;
}