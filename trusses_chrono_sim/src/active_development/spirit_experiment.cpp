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
    mterrain.SetSoilParameters(0.2e7,  // Bekker Kphi
                                   0,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   30,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   0.3e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
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

    ChVector3d Spirit_position = ChVector3d(0, 0.19, 0);

    //Leave empty if you want no trusses
    std::vector<ChVector3d> truss_positions = {
        ChVector3d(-.25, 0.025, 0),ChVector3d(0, 0.025, 0),ChVector3d(.25, 0.025, 0)
    };

    // Create a Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    //Build a spirit robot
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, Q_ROTATE_Z_TO_Y, truss_positions);

    //Turn on collisions with the terrain and create moving patches for the robot
    spirit0->AddMovingPatchesAndCollisions(mterrain);
    spirit0->LockTrussEndPosition(0, ChVector3d(-.5, .25, -.5));
    spirit0->LockTrussEndPosition(1, ChVector3d(0, .25, .50));
    spirit0->LockTrussEndPosition(2, ChVector3d(.5, .25, -.5));
    spirit0->m_body->SetFixed(true);
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

    struct ArmLength {
    double arm0;
    double arm1;
    double arm2;};

    std::vector<ArmLength> arm_lengths;

    std::ifstream file(std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/armies.csv");
    std::string temp_arm0;
    std::string temp_arm1;
    std::string temp_arm2;
    
    double index = 0;
    while (file.good()) {
        getline(file, temp_arm0, ',');
        getline(file, temp_arm1, ',');
        getline(file, temp_arm2);

        ArmLength temp;

        temp.arm0 = std::stod(temp_arm0);
        temp.arm1= std::stod(temp_arm1);
        temp.arm2 =std::stod(temp_arm2);

        arm_lengths.emplace_back(temp);
        
        index++;
    }

    int arm_index = 0;
    int loop_index = 0;
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

        if(sim_time > 1){
            spirit0->m_body->SetFixed(false);
      

            spirit0->m_trusses[0].SetArmDesiredLength(arm_lengths[arm_index].arm0);
            spirit0->m_trusses[0].SetArmPIDGains(10000,0,0);

            spirit0->m_trusses[1].SetArmDesiredLength(arm_lengths[arm_index].arm1);
            spirit0->m_trusses[1].SetArmPIDGains(10000,0,0);

            spirit0->m_trusses[2].SetArmDesiredLength(arm_lengths[arm_index].arm2);
            spirit0->m_trusses[2].SetArmPIDGains(10000,0,0);
            /*
            if(spirit0->m_trusses[0].motor->GetMotorPos()+1 - arm_lengths[arm_index].arm0 < .01 && spirit0->m_trusses[1].motor->GetMotorPos()+1 - arm_lengths[arm_index].arm1 < .01 && spirit0->m_trusses[2].motor->GetMotorPos()+1 - arm_lengths[arm_index].arm2 < .01){
                arm_index++;
            }
            */
            if(loop_index % 10 == 0){
                arm_index++;
            }
            
        }


        spirit0->m_trusses[0].UpdateArmLength();
        spirit0->m_trusses[1].UpdateArmLength();
        spirit0->m_trusses[2].UpdateArmLength();
        


        //std::cout << spirit0->m_body->GetPos().y() << std::endl;
        std::cout << "Time: " << sim_time << std::endl;
        std::cout << "Arm Index: " << arm_index << std::endl;
        std::cout << "Arm0 Length: " << spirit0->m_trusses[0].motor->GetMotorPos()+1 << std::endl;
        std::cout << "Arm1 Length: " << spirit0->m_trusses[1].motor->GetMotorPos()+1 << std::endl;
        std::cout << "Arm2 Length: " << spirit0->m_trusses[2].motor->GetMotorPos()+1 << std::endl;

        std::cout << "Arm0 Force: " << spirit0->m_trusses[0].motor->GetMotorForce() << std::endl;
        std::cout << "Arm1 Force: " << spirit0->m_trusses[1].motor->GetMotorForce() << std::endl;
        std::cout << "Arm2 Force: " << spirit0->m_trusses[2].motor->GetMotorForce() << std::endl;
        std::cout << "Spirit body: " << spirit0->m_body->GetPos() << std::endl;
        //This is what moves us to the next timestep, call it last
        vis_irr->BindAll();
        sys.DoStepDynamics(step_size);
        loop_index++;
        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }

    //Cleanly shut down ROS after the while loop
    rclcpp::shutdown();
    return 0;
}