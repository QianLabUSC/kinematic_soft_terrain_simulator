//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

//ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>
#include "std_srvs/srv/trigger.hpp"

//Standard Packages
#include <cstdlib>
#include <chrono>
#include <thread>
#include <memory>

//Custom Classes
#include "sim_tools/SpiritFactory.h"
#include "sim_tools/Spirit.h"
#include "sim_tools/Truss.h"
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

std::atomic_bool g_advance_simulation(false);

// Service callback function
void advance_simulation_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    g_advance_simulation.store(true);
    while(g_advance_simulation.load() && rclcpp::ok() && !g_terminate_requested.load()){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    response->success = true;
    response->message = "Simulation advanced";
    std::cout << "Simulation advanced" << std::endl;
}


// Function to run the single-threaded executor
void run_executor(std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor) {
    while (rclcpp::ok() && !g_terminate_requested.load()) {
        executor->spin_some();  // Use spin_some to avoid blocking
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Adjust as needed
    }
}

ChVector3d previous_spirit0_position;
ChQuaternion<> previous_spirit0_orientation;
float position_epsilon = 0.00001;
float orientation_epsilon = 0.0001;

int main(int argc, char* argv[]) {
    //Initialize ROS
    rclcpp::init(argc,argv);

    // Set up signal handler for termination signals (CTRL-c)
    std::signal(SIGINT, signal_handler);

    // Create the ROS node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("chrono_simulation_node");

    // Create the service
    auto service = node->create_service<std_srvs::srv::Trigger>(
    "advance_simulation", &advance_simulation_callback);

    // Create a single-threaded executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Add the node to the executor
    executor->add_node(node);

    // Launch the executor in its own thread
    std::thread executor_thread(run_executor, executor);

    SetChronoDataPath("/usr/local/share/chrono/data/");

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(chrono::ChVector3d(0, 0, -9.81));
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    //Choose # of threads to use for chrono, collisions, and eigen.
    sys.SetNumThreads(4, 8, 1);
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys); 

    //PSOR is a simplistic solver, while BARZILABORWEIN is second order precise but doesn't make much difference in performance and is slower.
    //sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
    double step_size = .0025; 

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Initialize the geometry of the soil
    double width = 20;
    double length = 20;
    double mesh_resolution = 0.02;
    mterrain.Initialize(length, width, mesh_resolution);

    std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/map.csv";
    auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath);
    mterrain.RegisterSoilParametersCallback(my_params);

    //Showing map texture
    mterrain.SetTexture(std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/Map_flipped.png");
    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    
    //Visualize a mesh for the terrain
    mterrain.GetMesh()->SetWireframe(false);

    ChVector3d Spirit_position = ChVector3d(5, 5, .2);

    //Leave empty if you want no trusses
    std::vector<ChVector3d> truss_positions = {
        ChVector3d(-.25,0,.025),ChVector3d(0,0,.025),ChVector3d(.25,0,.025)
    };

    // Create a Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    //Build a spirit robot
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, QUNIT, truss_positions);

    //Turn on collisions with the terrain and create moving patches for the robot
    spirit0->AddMovingPatchesAndCollisions(mterrain);
    spirit0->LockTrussEndPosition(0, ChVector3d(5-.5, 5-.5, .25));
    spirit0->LockTrussEndPosition(1, ChVector3d(5-0, 5+.5, .25));
    spirit0->LockTrussEndPosition(2, ChVector3d(5+.5, 5-.5, .25));

    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystemIrrlicht> vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("Load Spirit Example");
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(5.0, 3.0, 1.4), ChVector3d(5, 5, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);

    RCLCPP_INFO(node->get_logger(), "Chrono simulation node started. Waiting for service calls to advance simulation.");
    
    int loop_size = 100;
    int loop_index = 0;

    previous_spirit0_position = spirit0->m_body->GetPos();
    previous_spirit0_orientation = spirit0->m_body->GetRot();

    float position_diff = 0;
    float orientation_diff = 0;

    while (rclcpp::ok() && vis_irr->Run()) {
        double sim_time = sys.GetChTime();

        if(sim_time < 1){
            spirit0->m_body->SetFixed(true);
        }else{
            spirit0->m_body->SetFixed(false);
        }

        spirit0->HandleROS();

        std::cout << sim_time << std::endl;
        
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

        //std::cout << spirit0->m_trusses[0].motor->GetMotorForce() << std::endl;
        //std::cout << spirit0->m_trusses[0].GetLength() << std::endl;
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


        position_diff = (previous_spirit0_position - spirit0->m_body->GetPos()).Length();
        orientation_diff = (previous_spirit0_orientation - spirit0->m_body->GetRot()).Length();
        
        // If the service was called, proceed to the next timestep
        if(g_advance_simulation.load() || sim_time < 1){
            loop_index++;
            previous_spirit0_position = spirit0->m_body->GetPos();
            previous_spirit0_orientation = spirit0->m_body->GetRot();
            sys.DoStepDynamics(step_size);
        }

        std::cout << previous_spirit0_position[0] << " " << previous_spirit0_position[1] << " " << previous_spirit0_position[2] << std::endl;
        std::cout << spirit0->m_body->GetPos()[0] << " " << spirit0->m_body->GetPos()[1] << " " << spirit0->m_body->GetPos()[2] << std::endl;
        std::cout << position_diff << std::endl;
        std::cout << previous_spirit0_orientation.e0() << " " << previous_spirit0_orientation.e1() << " " << previous_spirit0_orientation.e2() << " " << previous_spirit0_orientation.e3() << std::endl;
        std::cout << spirit0->m_body->GetRot().e0() << " " << spirit0->m_body->GetRot().e1() << " " << spirit0->m_body->GetRot().e2() << " " << spirit0->m_body->GetRot().e3() << std::endl;
        std::cout << orientation_diff << std::endl;
    
        if(loop_index > 0 && position_diff < position_epsilon && orientation_diff < orientation_epsilon){
            loop_index = 0;
            g_advance_simulation.store(false);  // Reset the flag
        }
        
        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }

    // Ensure the executor thread is joined before shutting down
    if (executor_thread.joinable()) {
        executor_thread.join();
    }


    //Cleanly shut down ROS after the while loop
    rclcpp::shutdown();
    return 0;
}