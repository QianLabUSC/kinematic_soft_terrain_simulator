//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChBodyEasy.h"

//ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

//Standard Packages
#include <cstdlib>

//Custom Classes
#include "sim_tools/SpiritFactory.h"
#include "sim_tools/Spirit.h"
#include "sim_tools/Truss.h"

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

    std::shared_ptr<chrono::ChBodyEasyBox> body;
    std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();
    body = chrono_types::make_shared<ChBodyEasyBox>(5, .100, 5, 30, true, true, material);
    body->SetPos(ChVector3d(0, -.050, 0));
    body->EnableCollision(true);
    body->SetFixed(true);
    sys.AddBody(body);

    ChVector3d Spirit_position = ChVector3d(0, 0.05, 0);

    //Leave empty if you want no trusses
    std::vector<ChVector3d> truss_positions = {};

    // Create a Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    //Build a spirit robot
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, Q_ROTATE_Z_TO_Y, truss_positions);
    spirit0->m_body->EnableCollision(true);
    spirit0->m_leg_bodies[0][3]->EnableCollision(true);
    spirit0->m_leg_bodies[1][3]->EnableCollision(true);
    spirit0->m_leg_bodies[2][3]->EnableCollision(true);
    spirit0->m_leg_bodies[3][3]->EnableCollision(true);

    //Create visualization of chrono simulation
    std::shared_ptr<ChVisualSystemIrrlicht> vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("Load Spirit Example");
    vis_irr->SetShadows(true);
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    //vis_irr->AddTypicalLights();
    vis_irr->AddLightWithShadow(ChVector3d(30, 80, +30), ChVector3d(0,0,0), 280, 100, 200, 45);
    vis_irr->AddLightWithShadow(ChVector3d(30, 80, -30), ChVector3d(0,0,0), 280, 100,200 ,45);
    vis_irr->AddCamera(ChVector3d(2.0, 1.4, 0.0), ChVector3d(0, .8, 0));
    //vis_irr->SetShadowMapSize(2048);  // Larger size might give better results
    vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);

    utils::ChWriterCSV csv(" ");

    bool unstable = true;

    while (unstable && rclcpp::ok() && vis_irr->Run()) {
        double sim_time = sys.GetChTime();

        vis_irr->BeginScene();
        vis_irr->Render();
        vis_irr->SetCameraTarget(spirit0->m_body->GetPos());

        vis_irr->EndScene();

        spirit0->SetAbsDesiredPosition(0,0);
        spirit0->SetAbsDesiredPosition(1,0);
        spirit0->SetAbsDesiredPosition(2,0);
        spirit0->SetAbsDesiredPosition(3,0);

        //Slowly increment knee setpoint from 0 to a final setpoint of -1.
        double setpoint_knee = -(sim_time)/1;
        if(setpoint_knee < -1){
            setpoint_knee = -1;
        }

        //Slowly increment shoulder setpoint from 0 to a final setpoint of -.4.
        double setpoint_shoulder = -(sim_time)/1;
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
        std::cout << sim_time << std::endl;
        csv << spirit0->m_leg_bodies[0][3]->GetPos()[1] << spirit0->m_body->GetPos()[1] << spirit0->m_leg_motors[0][1]->GetMotorAngle() << spirit0->m_leg_motors[0][0]->GetMotorAngle() << std::endl;

        if(sim_time > 3){
            unstable = false;
        }
        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }
    const std::string out_dir = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/src/experiments";
    csv.WriteToFile(out_dir + "/spirit_height.dat");

    //Cleanly shut down ROS after the while loop
    rclcpp::shutdown();
    return 0;
}