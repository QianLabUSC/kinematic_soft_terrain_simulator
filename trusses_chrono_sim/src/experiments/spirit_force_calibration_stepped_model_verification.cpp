//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_thirdparty/filesystem/path.h"

//ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>
#include "std_srvs/srv/trigger.hpp"

//Standard Packages
#include <cstdlib>
#include <chrono>
#include <thread>
#include <memory>
#include <filesystem>

//Custom Classes
#include "sim_tools/SpiritFactory.h"
#include "sim_tools/Spirit.h"
#include "sim_tools/Truss.h"
#include "sim_tools/VariableSoilParams.h"
#include "sim_tools/SimpleMovingAverage.h"
#include "sim_tools/SetpointController.h"


#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrSkyBoxSceneNode.h"
#include <irrlicht.h>


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

void AddCustomSkyBox(std::shared_ptr<ChVisualSystemIrrlicht> vis_irr, std::string texture_dir) {
    // create sky box
    std::string str_left = texture_dir + "sky_west.jpg";
    std::string str_right = texture_dir + "sky_east.jpg";
    std::string str_front = texture_dir + "sky_north.jpg";
    std::string str_back = texture_dir + "sky_south.jpg";
    std::string str_up = texture_dir + "sky_up.jpg";
    std::string str_dn = texture_dir + "sky_down.jpg";


    // Load textures for the skybox sides
    irr::video::ITexture* left_texture = vis_irr->GetVideoDriver()->getTexture(str_left.c_str());
    irr::video::ITexture* right_texture = vis_irr->GetVideoDriver()->getTexture(str_right.c_str());
    irr::video::ITexture* front_texture = vis_irr->GetVideoDriver()->getTexture(str_front.c_str());
    irr::video::ITexture* back_texture = vis_irr->GetVideoDriver()->getTexture(str_back.c_str());
    irr::video::ITexture* up_texture = vis_irr->GetVideoDriver()->getTexture(str_up.c_str());
    irr::video::ITexture* dn_texture = vis_irr->GetVideoDriver()->getTexture(str_dn.c_str());


    // Create a skybox scene node
    auto skybox = new irr::scene::CSkyBoxSceneNode(up_texture, dn_texture,
                                        right_texture, left_texture,
                                        front_texture, back_texture,
                                        vis_irr->GetSceneManager()->getRootSceneNode(),
                                        vis_irr->GetSceneManager(), -1);
    skybox->drop();

    skybox->setRotation(irr::core::vector3df(90, 0, 0));
}

ChVector3d previous_spirit0_position;
ChQuaternion<> previous_spirit0_orientation;
float position_epsilon = 0.000005*2.5;
float orientation_epsilon = 0.00005*2.5;

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
    double width = 2;
    double length = 2;
    double mesh_resolution = 0.02;
    mterrain.Initialize(length, width, mesh_resolution);

    std::string stiffness_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/map.csv";
    auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath);
    mterrain.RegisterSoilParametersCallback(my_params);
    // Constant soil properties
    /*
    mterrain.SetSoilParameters(0.1e8,  // Bekker Kphi
                                   0,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   30,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   .25e8,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                   3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    */

    //Showing map texture
    //mterrain.SetTexture("/home/parallels/ros2_ws/src/chrono_sim/scripts/Map_flipped.png");
    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    
    //Visualize a mesh for the terrain
    //mterrain.GetMesh()->SetWireframe(false);

    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE,0, .025);

    // Set the texture or apply colors as needed
    auto terrain_material = chrono_types::make_shared<ChVisualMaterial>();
    // Diffuse color for a sandy texture - light, warm tones
    terrain_material->SetDiffuseColor(ChColor(0.9f, 0.8f, 0.6f));  // Light brown/yellowish for sand

    // Specular color - very low for sand, to avoid shiny reflections
    terrain_material->SetSpecularColor(ChColor(0.02f, 0.02f, 0.02f));  // Very low reflectivity

    // Ambient color - subtle, warm reflection from environmental light
    terrain_material->SetAmbientColor(ChColor(0.6f, 0.55f, 0.45f));  // Slightly darker and warm tones
    std::string texture_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/scripts/Map_flipped.png";
    terrain_material->SetKdTexture(texture_map_filepath); // Add texture if needed

    mterrain.GetMesh()->SetMaterial(0, terrain_material);
    mterrain.GetMesh()->SetWireframe(false);

    ChVector3d Spirit_position = ChVector3d(0, 0, .19);

    //Leave empty if you want no trusses
    std::vector<ChVector3d> truss_positions = {
        ChVector3d(0,0,.025)
    };

    // Create a Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    //Build a spirit robot
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, QUNIT, truss_positions);

    //Turn on collisions with the terrain and create moving patches for the robot
    spirit0->AddMovingPatchesAndCollisions(mterrain);
    spirit0->LockTrussEndPosition(0, ChVector3d(0, .5, .25));


    const std::string out_dir = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim";
    // Initialize logging output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
    }
    utils::ChWriterCSV csv(" ");


    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystemIrrlicht> vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1440, 1080);
    vis_irr->SetWindowTitle("Load Spirit Example");
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);
    vis_irr->SetShadows(true);
    vis_irr->Initialize();
    //vis_irr->AddSkyBox();
    AddCustomSkyBox(vis_irr, std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/");
    vis_irr->AddLightWithShadow(ChVector3d(-75, 37.5, 40), ChVector3d(0,0,0), 750, 300, 500, 180, 512, ChColor(1, .95f, .85f), true, false); //Angle, res, color, directional, clipborder
    vis_irr->AddLightWithShadow(ChVector3d(75, -37.5, 40), ChVector3d(0,0,0), 200, 300, 500, 180, 512, ChColor(1, .95f, .85f), true, false); //Angle, res, color, directional, clipborder
    vis_irr->AddCamera(ChVector3d(.8, -.4, .6), spirit0->m_body->GetPos());
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);
    vis_irr->EnableShadows();

    RCLCPP_INFO(node->get_logger(), "Chrono simulation node started. Waiting for service calls to advance simulation.");
    
    previous_spirit0_position = spirit0->m_body->GetPos();
    previous_spirit0_orientation = spirit0->m_body->GetRot();

    float position_diff = 0;
    float orientation_diff = 0;
    int loop_index = 0;

    float init_truss_length = spirit0->m_trusses[0].GetLength();
    float truss_length_increment = 0.05;
    float truss_length_max = 1.0;

    int window_size = 10;
    SimpleMovingAverage position_sma(window_size);
    SimpleMovingAverage orientation_sma(window_size);

    SetpointController shoulderController(0.0, -.4, -.4, 0.0); //Initial position, init velocity, min pos, max pos 
    SetpointController kneeController(0.0, -1, -1, 0.0); //Initial position. init velocity, min pos, max pos

    SetpointController trussController(init_truss_length, .1, 0, .5); //Initial position, init velocity, min pos, max pos

    double sim_time = sys.GetChTime();

    double next_frame_time = 0.0;  // When the next frame should be saved
    double frame_interval = 0.01;  // Interval between frames (0.01s)
    int index = 0;
    std::string folder_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "output_frames";
    std::string filename = folder_path + "/frame_" + std::to_string(index) + ".png";

    spirit0->m_body->SetFixed(true);

    while(rclcpp::ok() && vis_irr->Run() && sim_time < 1.5){
        sim_time = sys.GetChTime();
        std::cout << "Time: " << sim_time << std::endl;
        vis_irr->BeginScene();
        vis_irr->Render();
        vis_irr->EndScene();

        shoulderController.adjustSetpointWithVelocity(step_size);
        kneeController.adjustSetpointWithVelocity(step_size);

        spirit0->SetAbsDesiredPosition(0,0);
        spirit0->SetAbsDesiredPosition(1,0);
        spirit0->SetAbsDesiredPosition(2,0);
        spirit0->SetAbsDesiredPosition(3,0);

        //Update PID and send new controlSignal to shoulders
        spirit0->SetShoulderDesiredPosition(0,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(1,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(2,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(3,shoulderController.getSetpoint());

        //Update PID and send new controlSignal to knees
        spirit0->SetKneeDesiredPosition(0,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(1,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(2,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(3,kneeController.getSetpoint());

        sys.DoStepDynamics(step_size);

        if (sim_time >= next_frame_time) {
            std::string filename = folder_path + "/frame_" + std::to_string(index) + ".png";
            vis_irr->WriteImageToFile(filename);
            index++;
            
            // Update next_frame_time to schedule the next frame capture
            next_frame_time += frame_interval;
        }

        if (g_terminate_requested.load()) {
        break;
        }
    }
    spirit0->m_body->SetFixed(false);

    while (rclcpp::ok() && vis_irr->Run()) {

        spirit0->HandleROS();
        sim_time = sys.GetChTime();
        std::cout << "Time: " << sim_time << std::endl;
        
        vis_irr->BeginScene();
        vis_irr->Render();
        vis_irr->EndScene();

        spirit0->SetAbsDesiredPosition(0,0);
        spirit0->SetAbsDesiredPosition(1,0);
        spirit0->SetAbsDesiredPosition(2,0);
        spirit0->SetAbsDesiredPosition(3,0);

        //Update PID and send new controlSignal to shoulders
        spirit0->SetShoulderDesiredPosition(0,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(1,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(2,shoulderController.getSetpoint());
        spirit0->SetShoulderDesiredPosition(3,shoulderController.getSetpoint());

        //Update PID and send new controlSignal to knees
        spirit0->SetKneeDesiredPosition(0,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(1,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(2,kneeController.getSetpoint());
        spirit0->SetKneeDesiredPosition(3,kneeController.getSetpoint());
        
        if(g_advance_simulation.load()){
            position_diff = (previous_spirit0_position - spirit0->m_body->GetPos()).Length();
            position_sma.add(position_diff);

            orientation_diff = (previous_spirit0_orientation - spirit0->m_body->GetRot()).Length();
            orientation_sma.add(orientation_diff);

            std::cout << position_sma.getAverage() << " " << orientation_sma.getAverage() << std::endl;

            loop_index++;
            previous_spirit0_position = spirit0->m_body->GetPos();
            previous_spirit0_orientation = spirit0->m_body->GetRot();
            sys.DoStepDynamics(step_size);

            if (sim_time >= next_frame_time) {
            std::string filename = folder_path + "/frame_" + std::to_string(index) + ".png";
            vis_irr->WriteImageToFile(filename);
            index++;
            
            // Update next_frame_time to schedule the next frame capture
            next_frame_time += frame_interval;
            }
      
            trussController.adjustSetpointWithVelocity(step_size);
            spirit0->m_trusses[0].SetArmDesiredLength(trussController.getSetpoint());
            csv << sim_time << spirit0->m_body->GetPos() << spirit0->m_body->GetRot() << spirit0->m_leg_bodies[0][3]->GetPos()[0] << spirit0->m_leg_bodies[0][3]->GetPos()[1] << spirit0->m_leg_bodies[0][3]->GetPos()[2] << spirit0->m_leg_bodies[1][3]->GetPos()[0] << spirit0->m_leg_bodies[1][3]->GetPos()[1] << spirit0->m_leg_bodies[1][3]->GetPos()[2] << spirit0->m_leg_bodies[2][3]->GetPos()[0] << spirit0->m_leg_bodies[2][3]->GetPos()[1] << spirit0->m_leg_bodies[2][3]->GetPos()[2] << spirit0->m_leg_bodies[3][3]->GetPos()[0] << spirit0->m_leg_bodies[3][3]->GetPos()[1] << spirit0->m_leg_bodies[3][3]->GetPos()[2] << spirit0->m_trusses[0].GetLength() << trussController.getSetpoint() << spirit0->m_trusses[0].motor->GetMotorForce() << std::endl;

            if(loop_index > 10 && position_sma.getAverage() < position_epsilon && orientation_sma.getAverage() < orientation_epsilon){
                loop_index = 0;
                trussController.setLimits(0.0, trussController.getMaxLimit()+truss_length_increment);
                if(trussController.getMaxLimit() > truss_length_max+.04){
                    g_advance_simulation.store(false);
                    break;
                }
                g_advance_simulation.store(false);  // Reset the flag
            }

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

    csv.WriteToFile(out_dir + "/1spirit_1truss.dat");
    //Cleanly shut down ROS after the while loop
    rclcpp::shutdown();
    return 0;
}