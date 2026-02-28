// Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

// ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

// Standard Packages
#include <cstdlib>
#include <vector>

// Custom Classes
#include "sim_tools/SpiritFactory.h"
#include "sim_tools/Spirit.h"

using namespace chrono;
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
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Set up signal handler for termination signals (CTRL-c)
    std::signal(SIGINT, signal_handler);

    // Chrono data path
    SetChronoDataPath("/usr/local/share/chrono/data/");

    // Create Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));
    sys.SetNumThreads(4, 8, 1);
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys);
    sys.SetSolverType(ChSolver::Type::PSOR);

    // Step size
    double step_size = 0.001;

    // Initialize terrain
    vehicle::SCMTerrain mterrain(&sys);
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));
    mterrain.Initialize(1.5, 1.5, 0.02);
    mterrain.SetSoilParameters(0.2e7, 0, 1.1, 0, 30, 0.01, 0.3e7, 3e4);
    mterrain.EnableBulldozing(false);
    mterrain.GetMesh()->SetWireframe(true);

    // Define Spirit position
    ChVector3d Spirit_position(0, 0.19, 0);

    // No trusses
    std::vector<ChVector3d> truss_positions = {};

    // Single probe position
    std::vector<ChVector3d> probe_positions = {ChVector3d(0.3, 0.1, -0.1)};

    // Create Spirit Factory
    SpiritFactory Spiritfactory(&sys);

    // Build Spirit with one probe
    std::unique_ptr<Spirit> spirit0 = Spiritfactory.BuildSpirit(Spirit_position, Q_ROTATE_Z_TO_Y, truss_positions, probe_positions);

    // Enable collisions and moving patches
    spirit0->AddMovingPatchesAndCollisions(mterrain);
    spirit0->m_body->SetFixed(true);

    // Visualization
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("Single Probe Example");
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(2.0, 1.4, 0.0), ChVector3d(0, 0.8, 0));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);

    // Main simulation loop
    while (rclcpp::ok() && vis_irr->Run()) {
        double sim_time = sys.GetChTime();

        vis_irr->BeginScene();
        vis_irr->Render();
        vis_irr->EndScene();

        // Change probe length at sim_time > 2.0
        if (sim_time > 2.0) {
            // Unfix the robot's body
            spirit0->m_body->SetFixed(false);

            // Extend the probe's arms
            spirit0->m_probes[0].SetArmDesiredLengths(10, -10); // Example lengths
            spirit0->m_probes[0].UpdateArmLength(); // Example lengths

            // Retrieve and log probe forces
            auto probe_forces = spirit0->GetProbeForces(mterrain);
            std::cout << "Time: " << sim_time << " Probe Force: " << probe_forces[0] << std::endl;
        }

        // Step forward in simulation
        vis_irr->BindAll();
        sys.DoStepDynamics(step_size);

        // Handle CTRL-C signal
        if (g_terminate_requested.load()) {
            break;
        }
    }

    // Cleanly shut down ROS
    rclcpp::shutdown();
    return 0;
}
