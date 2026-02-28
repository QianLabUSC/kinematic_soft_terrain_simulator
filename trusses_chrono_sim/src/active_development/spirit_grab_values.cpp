//Chrono Packages
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
//ROS Packages
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>

//Standard Packages
#include <cstdlib>

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

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

std::pair<std::vector<std::shared_ptr<chrono::ChBody>>, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> CreateLeg(ChSystem* m_sys, ChVector3d leg_pos, bool mirrored){
    std::vector<std::shared_ptr<chrono::ChBody>> leg_bodies;
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors;

    std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();

    ChColor darkgrey = ChColor(.1f, .1f, .1f);
    ChColor silver = ChColor(.3f, .3f, .3f);
    ChColor red = ChColor(.8f, .0f, .2f);

    ChVector3d hip_inertia = ChVector3d(.00066963541,.0006963541,.0008696875);
    double hip_mass = .575;

    ChVector3d m_upper_pos = ChVector3d(-0.103, 0, 0.05098);
    ChVector3d upper_inertia = ChVector3d(.00005,.001,.001);
    double upper_mass = .775;


    ChVector3d m_lower_pos = ChVector3d(-0.103, 0, 0.07298);
    ChVector3d lower_inertia = ChVector3d(.000005,.0001,.0001);
    double lower_mass = .075;

    ChVector3d m_toe_pos = ChVector3d(0, 0, 0.07298);
    ChVector3d toe_inertia = ChVector3d(.00005,.00005,.00005);
    double toe_mass = .05;

    ChVector3d upper_pos = m_upper_pos;
    ChVector3d lower_pos = m_lower_pos;
    ChVector3d toe_pos = m_toe_pos;

    if(mirrored){
        upper_pos = ChVector3d(upper_pos.x(), upper_pos.y(), -upper_pos.z());
        lower_pos = ChVector3d(lower_pos.x(), lower_pos.y(), -lower_pos.z());
        toe_pos = ChVector3d(toe_pos.x(), toe_pos.y(), -toe_pos.z());
    }

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.055, .08, .1, material);
    hip->SetPos(leg_pos);
    hip->SetMass(hip_mass);
    hip->SetInertia(hip_inertia);
    hip->EnableCollision(false);
    hip->GetVisualShape(0)->SetColor(darkgrey);
    hip->SetFixed(true);
    m_sys->Add(hip);
    leg_bodies.push_back(hip);

    std::shared_ptr<chrono::ChBodyEasyBox> upper = chrono_types::make_shared<ChBodyEasyBox>(.206, .055, .022, 10, true, false, material);
    upper->SetPos(leg_pos+upper_pos);
    upper->SetMass(upper_mass);
    upper->SetInertia(upper_inertia);
    upper->GetVisualShape(0)->SetColor(silver);
    m_sys->Add(upper);
    leg_bodies.push_back(upper);

    std::shared_ptr<chrono::ChBodyEasyCylinder> lower = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, material);
    lower->SetPos(leg_pos+lower_pos);
    lower->SetMass(lower_mass);
    lower->SetInertia(lower_inertia);
    lower->EnableCollision(false);
    lower->GetVisualShape(0)->SetColor(darkgrey);
    m_sys->Add(lower);
    leg_bodies.push_back(lower);

    std::shared_ptr<chrono::ChBodyEasySphere> toe = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, false, material);
    toe->SetPos(leg_pos+toe_pos);
    toe->SetMass(toe_mass);
    toe->SetInertia(toe_inertia);
    toe->EnableCollision(false);
    toe->GetVisualShape(0)->SetColor(darkgrey);
    m_sys->Add(toe);
    leg_bodies.push_back(toe);

    std::shared_ptr<chrono::ChLinkLockLock> toe_to_lower_link = std::make_shared<ChLinkLockLock>();
    toe_to_lower_link->Initialize(lower, toe, ChFrame<>(toe->GetPos(), QUNIT));
    m_sys->AddLink(toe_to_lower_link);

    std::shared_ptr<chrono::ChLinkMotorRotationTorque> shoulder_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    shoulder_motor->Initialize(hip, upper, ChFrame<>(hip->GetPos(), QUNIT));
    m_sys->Add(shoulder_motor);

    std::shared_ptr<chrono::ChLinkMotorRotationTorque> knee_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    //Frame is half the length of the upper box in the x direction plus it's position
    knee_motor->Initialize(upper, lower, ChFrame<>(ChVector3d(-.103,0,0)+upper->GetPos(), QUNIT));
    m_sys->Add(knee_motor);

    leg_motors.push_back(knee_motor);
    leg_motors.push_back(shoulder_motor);

    return std::make_pair(leg_bodies, leg_motors);
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

    //Load Spirit Robot
    std::string ros_package_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/";
    int robot_num = 0;
    int truss_num = 0;
  /*
    Spirit spirit0(&sys, ros_package_filepath,robot_num,truss_num,step_size);


    //Spawn the robot in the world at the given position and orientation
    //spirit0.SpawnRobot(ChVector3d(0,0,0),ChQuaternion<>(.707107,.707107,0,0));
    //spirit0.body->SetFixed(true);
    //spirit0.EnableAbs(0,0,0);
    spirit0.EnableKnees(0,0,0);
    spirit0.EnableShoulders(0,0,0);
    spirit0.Populate();
    spirit0.m_robot->GetChBody("body_spirit0")->SetFixed(true);

    for (auto body : sys.GetBodies()) {
        std::cout << "Body ID: " << body->GetName() << "\n";
        std::cout << "Position: " << body->GetPos() << "\n";
        std::cout << "Orientation: " << body->GetRot() << "\n";
        std::cout << "Inertia: " << body->GetInertia() << "\n";
        std::cout << "Mass: " << body->GetMass() << "\n";
        std::cout << "----------------------\n";
    }

    for(auto link : sys.GetLinks()){
        std::cout << "Link ID: " << link->GetName() << "\n";
        std::cout << "----------------------\n";
    }
    */

    //Rhex_position_W
    //p_WRh0_W or p_WRhset_W
    ChVector3d Spirit_position = ChVector3d(0, 0.2, 0);

    //truss_positions_B
    //p_Rh0Tset_Rh0
    std::vector<ChVector3d> truss_positions = {};
    // Create a RHex robot
    SpiritFactory Spiritfactory(&sys);

    Spiritfactory.BuildSpirit(Spirit_position,  Q_ROTATE_Z_TO_Y, truss_positions);

/*

    CreateLeg(&sys, ChVector3d(0,1,0), true);
    std::shared_ptr<chrono::ChContactMaterialSMC> material = chrono_types::make_shared<ChContactMaterialSMC>();
    
    ChColor darkgrey = ChColor(.1f, .1f, .1f);
    ChColor silver = ChColor(.3f, .3f, .3f);
    ChColor red = ChColor(.8f, .0f, .2f);
    
    //Y & Z swapped
    std::shared_ptr<chrono::ChBodyEasyBox> body = chrono_types::make_shared<ChBodyEasyBox>(.335, .104, .24, 10, true, false, material);
    auto offset = ChVector3d(0,0,0);
    body->SetPos(offset+ChVector3d(0, 0, 0));
    body->SetFixed(true);
    body->SetMass(5.75);
    body->GetVisualShape(0)->SetColor(darkgrey);
    ChVector3d body_inertia = ChVector3d(.05,.1,.1);
    body->SetInertia(body_inertia);
    sys.Add(body);

    //Radius & Height
    auto hip_link_origin = ChVector3d(0, 0, .028);
    ChVector3d hip_inertia = ChVector3d(.00066963541,.0006963541,.0008696875);
    double hip_mass = .575;

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip0 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.055, .08, .1, material);
    hip0->SetPos(offset+hip_link_origin+ChVector3d(.2263, 0, .07));
    hip0->SetMass(hip_mass);
    hip0->SetInertia(hip_inertia);
    hip0->EnableCollision(false);
    hip0->GetVisualShape(0)->SetColor(darkgrey);
    hip0->SetFixed(true);
    sys.Add(hip0);

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip1 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.055, .08, .1, material);
    hip1->SetPos(offset+hip_link_origin+ChVector3d(-.2263, 0, .07));
    hip1->SetMass(hip_mass);
    hip1->SetInertia(hip_inertia);
    hip1->EnableCollision(false);
    hip1->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(hip1);

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip2 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.055, .08, .1, material);
    hip2->SetPos(offset-hip_link_origin+ChVector3d(.2263, 0, -.07));
    hip2->SetMass(hip_mass);
    hip2->SetInertia(hip_inertia);
    hip2->EnableCollision(false);
    hip2->GetVisualShape(0)->SetColor(red);
    sys.Add(hip2);

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip3 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z,.055, .08, .1, material);
    hip3->SetPos(offset-hip_link_origin+ChVector3d(-.2263, 0, -.07));
    hip3->SetMass(hip_mass);
    hip3->SetInertia(hip_inertia);
    hip3->EnableCollision(false);
    hip3->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(hip3);

    std::cout << hip0->GetPos() << std::endl;
    std::cout << hip1->GetPos() << std::endl;
    std::cout << hip2->GetPos() << std::endl;
    std::cout << hip3->GetPos() << std::endl;

    auto upper_link_origin_right = ChVector3d(-.103, 0, -0.022);
    auto upper_link_origin_left = ChVector3d(-.103, 0, 0.022);
    ChVector3d upper_inertia = ChVector3d(.00005,.001,.001);
    double upper_mass = .775;
    std::shared_ptr<chrono::ChBodyEasyBox> upper0 = chrono_types::make_shared<ChBodyEasyBox>(.206, .055, .022, 10, true, false, material);
    upper0->SetPos(offset+upper_link_origin_right+ChVector3d(.2263,0,.17098));
    upper0->SetMass(upper_mass);
    upper0->SetInertia(upper_inertia);
    upper0->GetVisualShape(0)->SetColor(silver);
    sys.Add(upper0);

    std::shared_ptr<chrono::ChBodyEasyBox> upper1 = chrono_types::make_shared<ChBodyEasyBox>(.206, .055, .022, 10, true, false, material);
    upper1->SetPos(offset+upper_link_origin_right+ChVector3d(-.2263,0,.17098));
    upper1->SetMass(upper_mass);
    upper1->SetInertia(upper_inertia);
    upper1->GetVisualShape(0)->SetColor(silver);
    sys.Add(upper1);

    std::shared_ptr<chrono::ChBodyEasyBox> upper2 = chrono_types::make_shared<ChBodyEasyBox>(.206, .055, .022, 10, true, false, material);
    upper2->SetPos(offset+upper_link_origin_left+ChVector3d(.2263,0,-.17098));
    upper2->SetMass(upper_mass);
    upper2->SetInertia(upper_inertia);
    upper2->GetVisualShape(0)->SetColor(silver);
    sys.Add(upper2);

    std::shared_ptr<chrono::ChBodyEasyBox> upper3 = chrono_types::make_shared<ChBodyEasyBox>(.206, .055, .022, 10, true, false, material);
    upper3->SetPos(offset+upper_link_origin_left+ChVector3d(-.2263,0,-.17098));
    upper3->SetMass(upper_mass);
    upper3->SetInertia(upper_inertia);
    upper3->GetVisualShape(0)->SetColor(silver);
    sys.Add(upper3);

    auto lower_link_origin = ChVector3d(.103, 0, 0);
    ChVector3d lower_inertia = ChVector3d(.000005,.0001,.0001);
    double lower_mass = .075;
    std::shared_ptr<chrono::ChBodyEasyCylinder> lower0 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, material);
    lower0->SetPos(offset+lower_link_origin+ChVector3d(0.0203,0,0.17098));
    lower0->SetMass(lower_mass);
    lower0->SetInertia(lower_inertia);
    lower0->EnableCollision(false);
    lower0->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(lower0);

    std::shared_ptr<chrono::ChBodyEasyCylinder> lower1 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, material);
    lower1->SetPos(offset+lower_link_origin+ChVector3d(-0.432301, 0, 0.17098));
    lower1->SetMass(lower_mass);
    lower1->SetInertia(lower_inertia);
    lower1->EnableCollision(false);
    lower1->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(lower1);

    std::shared_ptr<chrono::ChBodyEasyCylinder> lower2 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, material);
    lower2->SetPos(offset+lower_link_origin+ChVector3d(0.0203, 0, -0.17098));
    lower2->SetMass(lower_mass);
    lower2->SetInertia(lower_inertia);
    lower2->EnableCollision(false);
    lower2->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(lower2);

    std::shared_ptr<chrono::ChBodyEasyCylinder> lower3 = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, material);
    lower3->SetPos(offset+lower_link_origin+ChVector3d(-0.432301, 0, -0.17098));
    lower3->SetMass(lower_mass);
    lower3->SetInertia(lower_inertia);
    lower3->EnableCollision(false);
    lower3->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(lower3);

    auto toe_link_origin = ChVector3d(0, 0, 0);
    ChVector3d toe_inertia = ChVector3d(.00005,.00005,.00005);
    double toe_mass = .05;

    std::shared_ptr<chrono::ChBodyEasySphere> toe0 = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, false, material);
    toe0->SetPos(offset+toe_link_origin+ChVector3d(.2263,0,.17098));
    toe0->SetMass(toe_mass);
    toe0->SetInertia(toe_inertia);
    toe0->EnableCollision(false);
    toe0->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(toe0);

    std::shared_ptr<chrono::ChBodyEasySphere> toe1 = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, false, material);
    toe1->SetPos(offset+toe_link_origin+ChVector3d(-.2263,0,.17098));
    toe1->SetMass(toe_mass);
    toe1->SetInertia(toe_inertia);
    toe1->EnableCollision(false);
    toe1->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(toe1);

    std::shared_ptr<chrono::ChBodyEasySphere> toe2 = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, false, material);
    toe2->SetPos(offset+toe_link_origin+ChVector3d(.2263,0,-.17098));
    toe2->SetMass(toe_mass);
    toe2->SetInertia(toe_inertia);
    toe2->EnableCollision(false);
    toe2->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(toe2);

    std::shared_ptr<chrono::ChBodyEasySphere> toe3 = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, false, material);
    toe3->SetPos(offset+toe_link_origin+ChVector3d(-.2263,0,-.17098));
    toe3->SetMass(toe_mass);
    toe3->SetInertia(toe_inertia);
    toe3->EnableCollision(false);
    toe3->GetVisualShape(0)->SetColor(darkgrey);
    sys.Add(toe3);

    auto abs0_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    abs0_motor->Initialize(body, hip0, ChFrame<>(body->GetPos()+ChVector3d(.2263,0,.07), QuatFromAngleY(-CH_PI_2)));
    sys.Add(abs0_motor);

    auto abs1_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    abs1_motor->Initialize(body, hip1, ChFrame<>(body->GetPos()+ChVector3d(-.2263,0,.07), QuatFromAngleY(-CH_PI_2)));
    sys.Add(abs1_motor);

    auto abs2_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    abs2_motor->Initialize(body, hip2, ChFrame<>(body->GetPos()+ChVector3d(.2263,0,-.07), QuatFromAngleY(-CH_PI_2)));
    sys.Add(abs2_motor);

    auto abs3_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    abs3_motor->Initialize(body, hip3, ChFrame<>(body->GetPos()+ChVector3d(-.2263,0,-.07), QuatFromAngleY(-CH_PI_2)));
    sys.Add(abs3_motor);

    auto shoulder0_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    shoulder0_motor->Initialize(hip0, upper0, ChFrame<>(hip0->GetPos(), QUNIT));
    sys.Add(shoulder0_motor);

    auto shoulder1_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    shoulder1_motor->Initialize(hip1, upper1, ChFrame<>(hip1->GetPos(), QUNIT));
    sys.Add(shoulder1_motor);

    auto shoulder2_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    shoulder2_motor->Initialize(hip2, upper2, ChFrame<>(hip2->GetPos(), QUNIT));
    sys.Add(shoulder2_motor);

    auto shoulder3_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    shoulder3_motor->Initialize(hip3, upper3, ChFrame<>(hip3->GetPos(), QUNIT));
    sys.Add(shoulder3_motor);

    auto knee0_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    //Frame is half the length of the upper0 box in the x direction plus it's position
    knee0_motor->Initialize(upper0, lower0, ChFrame<>(ChVector3d(-.103,0,0)+upper0->GetPos(), QUNIT));
    sys.Add(knee0_motor);

    auto knee1_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    knee1_motor->Initialize(upper1, lower1, ChFrame<>(ChVector3d(-.103,0,0)+upper1->GetPos(), QUNIT));
    sys.Add(knee1_motor);

    auto knee2_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    knee2_motor->Initialize(upper2, lower2, ChFrame<>(ChVector3d(-.103,0,0)+upper2->GetPos(), QUNIT));
    sys.Add(knee2_motor);

    auto knee3_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    knee3_motor->Initialize(upper3, lower3, ChFrame<>(ChVector3d(-.103,0,0)+upper3->GetPos(), QUNIT));
    sys.Add(knee3_motor);

    auto toe0_to_lower0_link = std::make_shared<ChLinkLockLock>();
    toe0_to_lower0_link->Initialize(lower0, toe0, ChFrame<>(toe0->GetPos(), QUNIT));
    sys.AddLink(toe0_to_lower0_link);

    auto toe1_to_lower1_link = std::make_shared<ChLinkLockLock>();
    toe1_to_lower1_link->Initialize(lower1, toe1, ChFrame<>(toe1->GetPos(), QUNIT));
    sys.AddLink(toe1_to_lower1_link);

    auto toe2_to_lower2_link = std::make_shared<ChLinkLockLock>();
    toe2_to_lower2_link->Initialize(lower2, toe2, ChFrame<>(toe2->GetPos(), QUNIT));
    sys.AddLink(toe2_to_lower2_link);

    auto toe3_to_lower3_link = std::make_shared<ChLinkLockLock>();
    toe3_to_lower3_link->Initialize(lower3, toe3, ChFrame<>(toe3->GetPos(), QUNIT));
    sys.AddLink(toe3_to_lower3_link);

*/
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