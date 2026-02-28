// Chrono Packages
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
// Standard Packages
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <csignal>
#include <atomic>
#include <filesystem>
// Sim Tools
#include "sim_tools/RHexFactory.h"
#include "sim_tools/RHex.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

std::atomic_bool g_terminate_requested(false);

void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_terminate_requested.store(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Termination signal (CTRL-c) received. Shutting down...");
    }
}

ChVector3d GetNextWaypointTarget(
    const ChVector3d& current_pos,
    const std::vector<std::pair<double, double>>& waypoints,
    int& current_index,
    double tolerance = 0.1
) {
    if (current_index >= (int)waypoints.size()) return current_pos;

    double dx = waypoints[current_index].first - current_pos.x();
    double dy = waypoints[current_index].second - current_pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < tolerance && current_index < (int)waypoints.size() - 1) {
        current_index++;
    }

    return ChVector3d(waypoints[current_index].first, waypoints[current_index].second, 0);
}

double test(int argc, char* argv[], double bodymass, double legmass,
            std::string stiffness, double stiffness_scale, double counterweight_mass) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    SetChronoDataPath("/usr/local/share/chrono/data/");

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
    sys.SetNumThreads(4, 8, 1);
    collsys->SetNumThreads(4);
    sys.SetCollisionSystem(collsys);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    double step_size = 0.004;

    vehicle::SCMTerrain mterrain(&sys);
    double width = 6.0 * 3;
    double length = 4.0 * 3;
    double mesh_resolution = 0.03;
    mterrain.Initialize(length, width, mesh_resolution);

    mterrain.EnableBulldozing(true);
    mterrain.SetBulldozingParameters(55, 1, 5, 6);

    // Visualization
    mterrain.SetMeshWireframe(true);
    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.05);
    mterrain.SetTexture(std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/meshes/output.png", 1.0, -1.0);
    mterrain.SetColor(ChColor(0.5, 0.5, 0.5));

    // Variable soil params
    std::string stiffness_map_filepath =
        std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/scripts/output_stiffness.csv";
    auto my_params = chrono_types::make_shared<VariableSoilParams>(stiffness_map_filepath, stiffness_scale);
    mterrain.RegisterSoilParametersCallback(my_params);

    std::string test_name = "exp3_success_safe";

    // Load waypoints
    std::string csv_file =
        std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/scripts/tests_to_run/" + test_name + ".csv";
    std::ifstream file(csv_file);
    std::vector<std::pair<double, double>> waypoints;
    std::string line;
    std::getline(file, line); // skip header
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string cell;
        std::getline(iss, cell, ','); // skip first column
        std::getline(iss, cell, ',');
        double x_cm = std::stod(cell);
        std::getline(iss, cell, ',');
        double y_cm = std::stod(cell);
        waypoints.emplace_back(x_cm / 100.0, y_cm / 100.0);
    }

    // Starting pose
    ChVector3d RHex_position(waypoints.front().first, waypoints.front().second, 0.09);
    const ChVector3d start_location_xyz = RHex_position;
    waypoints.erase(waypoints.begin());

    RHexFactory RHexFactory(&sys);
    ChVector3d dir_vec = ChVector3d(waypoints[0].first - RHex_position.x(),
                                    waypoints[0].second - RHex_position.y(),
                                    0);
    double yaw = std::atan2(dir_vec.y(), dir_vec.x());
    auto Rhex = RHexFactory.BuildRHex(RHex_position, QuatFromAngleZ(yaw),
                                      {}, bodymass, legmass, counterweight_mass);
    Rhex->AddMovingPatchesAndCollisions(mterrain);
    
    // Set RHex PID gains (Kp, Ki, Kd)
    Rhex->SetLegGains(500.0, 0.0, 10.0);  // Higher Kp for more responsive control

    // Visualization system
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(1600, 1200);
    vis_irr->SetWindowTitle("RHex Waypoint Navigation");
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);
    vis_irr->Initialize();
    vis_irr->AddSkyBox();
    vis_irr->AddTypicalLights();
    vis_irr->AddCamera(ChVector3d(0, -12, 6.0), ChVector3d(0, 0, 0.8));
    vis_irr->EnableShadows();
    vis_irr->AttachSystem(&sys);

    if (mterrain.GetMesh()) {
        vis_irr->AddVisualModel(mterrain.GetMesh(), ChFrame<>());
    }
    std::shared_ptr<ChVisualSystem> vis = vis_irr;

    int current_index = 0;
    double simulation_time = sys.GetChTime();

    // CSV output
    std::string output_file_path =
        std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/scripts/" + test_name + "_output_data.csv";
    std::ofstream output_file(output_file_path);
    output_file << "time_s,body_mass,leg_mass,"
                << "pos_x,pos_y,pos_z,"
                << "rot_e0,rot_e1,rot_e2,rot_e3,"
                << "vel_x,vel_y,vel_z,"
                << "prev_wp_x,prev_wp_y,prev_wp_z,"
                << "next_wp_x,next_wp_y,next_wp_z,"
                << "torque_0,torque_1,torque_2,torque_3,torque_4,torque_5\n";

    // Control parameters
    double gait_frequency = 1.0;
    double leg_movement_start_time = 0.1;
    bool leg_movement_started = false;
    double turn_threshold = 0.261799 * 2; // 15 deg
    double turn_speed = 1.0 * M_PI * gait_frequency;

    // Frame saving setup
    static int frame_number = 0;

    while (rclcpp::ok() && vis->Run()) {
        Rhex->HandleROS();
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Save frame to PNG
        std::ostringstream filename;
        std::string frames_dir = std::string(getenv("HOME")) + "/ros2_ws/src/trusses_chrono_sim/scripts/frames/" + test_name;
        std::filesystem::create_directories(frames_dir);
        filename << std::string(getenv("HOME")) << "/ros2_ws/src/trusses_chrono_sim/scripts/frames/"
                 << test_name << "/frame" << std::setw(5) << std::setfill('0') << frame_number++ << ".png";
        vis_irr->WriteImageToFile(filename.str());

        if (simulation_time >= leg_movement_start_time) {
            leg_movement_started = true;
        }

        ChVector3d current_pos = Rhex->m_body->GetPos();
        ChQuaternion<> current_rot = Rhex->m_body->GetRot();

        // // Update camera to follow the robot: offset in robot frame, look slightly above robot
        // {
        //     ChVector3d cam_offset_robot(0.0, -6.0, 2.5); // back and above relative to robot
        //     ChVector3d cam_pos_world = current_rot.Rotate(cam_offset_robot) + current_pos;
        //     ChVector3d cam_target_world = current_pos + ChVector3d(0, 0, 0.8);
        //     vis_irr->SetCameraPosition(cam_pos_world);
        //     vis_irr->SetCameraTarget(cam_target_world);
        // }
        ChVector3d target = GetNextWaypointTarget(current_pos, waypoints, current_index);
        ChVector3d to_target = target - current_pos;
        double heading = std::atan2(to_target.y(), to_target.x());
        ChVector3d forward = current_rot.Rotate(ChVector3d(1, 0, 0));
        double current_yaw = std::atan2(forward.y(), forward.x());
        double angle_diff = heading - current_yaw;
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        if (!leg_movement_started) {
            for (int i = 0; i < 6; i++)
                Rhex->SetLegDesiredPosition(i, Rhex->m_leg_motors[i]->GetMotorAngle());
        } else if (std::abs(angle_diff) > turn_threshold) {
            double left = (angle_diff > 0) ? turn_speed : -turn_speed;
            double right = -left;
            Rhex->SetLegDesiredVelocity(0, left);
            Rhex->SetLegDesiredVelocity(2, left);
            Rhex->SetLegDesiredVelocity(4, left);
            Rhex->SetLegDesiredVelocity(1, right);
            Rhex->SetLegDesiredVelocity(3, right);
            Rhex->SetLegDesiredVelocity(5, right);
            std::cout << "Left speed: " << left << std::endl;
        } else {
            double forward_speed = gait_frequency * 2.0 * M_PI;
            for (int leg = 0; leg < 6; ++leg)
                Rhex->SetLegDesiredVelocity(leg, -forward_speed);
            std::cout << "Forward speed: " << forward_speed << std::endl;
        }

        // Logging
        if (output_file.is_open()) {
            ChVector3d vel = Rhex->m_body->GetPosDt();
            ChVector3d prev_wp, next_wp;
            if (current_index <= 0) {
                prev_wp = start_location_xyz;
            } else {
                prev_wp = ChVector3d(waypoints[current_index - 1].first,
                                     waypoints[current_index - 1].second, 0.0);
            }
            if (current_index < (int)waypoints.size()) {
                next_wp = ChVector3d(waypoints[current_index].first,
                                     waypoints[current_index].second, 0.0);
            } else {
                next_wp = waypoints.empty() ? start_location_xyz
                                            : ChVector3d(waypoints.back().first,
                                                         waypoints.back().second, 0.0);
            }
            output_file << simulation_time << ","
                        << bodymass << "," << legmass << ","
                        << current_pos.x() << "," << current_pos.y() << "," << current_pos.z() << ","
                        << current_rot.e0() << "," << current_rot.e1() << "," << current_rot.e2() << "," << current_rot.e3() << ","
                        << vel.x() << "," << vel.y() << "," << vel.z() << ","
                        << prev_wp.x() << "," << prev_wp.y() << "," << prev_wp.z() << ","
                        << next_wp.x() << "," << next_wp.y() << "," << next_wp.z();
            for (int i = 0; i < 6; ++i)
                output_file << "," << Rhex->m_leg_motors[i]->GetMotorTorque();
            output_file << "\n";
        }

        vis->BindAll();
        sys.DoStepDynamics(step_size);
        simulation_time = sys.GetChTime();

        if (current_index == (int)waypoints.size() - 1 &&
            (current_pos - target).Length() < 0.1) {
            std::cout << "Reached final waypoint." << std::endl;
            break;
        }
        if (g_terminate_requested.load()) break;
    }

    output_file.close();
    rclcpp::shutdown();
    return (Rhex->m_body->GetPos().x()) / simulation_time;
}

int main(int argc, char* argv[]) {
    double bodymass = 80.0;
    double counterweight_mass = 0.0;
    double legmass = 0.39;
    double stiffness_scale = 1.0;
    std::string stiffness = "50";
    double velocity =
        test(argc, argv, bodymass, legmass, stiffness, stiffness_scale, counterweight_mass);
    return 0;
}
