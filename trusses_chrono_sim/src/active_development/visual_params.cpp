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

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrSkyBoxSceneNode.h"
#include <irrlicht.h>
//Standard Packages
#include <cstdlib>

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
int main(int argc, char* argv[]) {
    //Initialize ROS
    rclcpp::init(argc,argv);
    std::string ground_texture_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "meshes" / "sand";
    std::cout<<ground_texture_path << std::endl;
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

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    //mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(-CH_PI_2)));

    // Initialize the geometry of the soil
    double width = 30;
    double length = 30;
    double mesh_resolution = 0.05;
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
    mterrain.GetMesh()->SetWireframe(false);

    std::string texture_map_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/sand3.png";
    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE,0, .025);
    mterrain.SetTexture(texture_map_filepath);

    // Set the texture or apply colors as needed
    auto terrain_material = chrono_types::make_shared<ChVisualMaterial>();
    terrain_material->SetDiffuseColor(ChColor(0.1f, 0.1f, 0.1f)); // Less reflective
    terrain_material->SetSpecularColor(ChColor(0.05f, 0.05f, 0.05f)); // Reduce specular reflection
    terrain_material->SetAmbientColor(ChColor(0.3f, 0.3f, 0.3f)); // Set ambient color
    terrain_material->SetKdTexture(std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/sand3.png"); // Add texture if needed

    mterrain.GetMesh()->SetMaterial(0, terrain_material);


    //Rhex_position_W
    //p_WRh0_W or p_WRhset_W
    ChVector3d RHex_position = ChVector3d(0, 0, 0.5);

    //truss_positions_B
    //p_Rh0Tset_Rh0
    //std::vector<ChVector3d> truss_positions = {
    //        ChVector3d(0, .2, -.2), ChVector3d(0, .2, .2), ChVector3d(0, .2, .5),
    //};
    std::vector<ChVector3d> truss_positions = {};
    // Create a RHex robot
    RHexFactory RHexFactory(&sys);

    auto Rhex = RHexFactory.BuildRHex(RHex_position, QUNIT, truss_positions);

    Rhex->AddMovingPatchesAndCollisions(mterrain);

    //Rhex->LockTrussEndPosition(0, ChVector3d(0, 1, -.4));
    //Rhex->LockTrussEndPosition(1, ChVector3d(0, 1, .4));
    //Rhex->m_body->SetFixed(true);

    std::shared_ptr<ChVisualSystem> vis;

    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(3840, 1600);
    vis_irr->SetWindowTitle("Visual Test");
    vis_irr->SetShadows(true);
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);

    vis_irr->Initialize();
    //vis_irr->AddSkyBox();
    AddCustomSkyBox(vis_irr, std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/");
    //vis_irr->AddTypicalLights();)
    vis_irr->AddLightWithShadow(ChVector3d(-75, 37.5, 40), ChVector3d(0,0,0), 500, 300, 500, 180, 512, ChColor(1, 1, 1), true, false); //Angle, res, color, directional, clipborder
    vis_irr->AddLightWithShadow(ChVector3d(75, -37.5, 40), ChVector3d(0,0,0), 100, 300, 500, 180, 512, ChColor(1, 1, 1), true, false); //Angle, res, color, directional, clipborder
    //vis_irr->AddLightWithShadow(ChVector3d(30, 30, 160), ChVector3d(0,0,0), 280, 100, 200, 45);
    //vis_irr->AddLightWithShadow(ChVector3d(30, -30, 160), ChVector3d(0,0,0), 280, 100,200 ,45);
    vis_irr->AddCamera(ChVector3d(0, -2, 1.40), ChVector3d(0, 0, 0.8));
    vis_irr->AttachSystem(&sys);
    vis_irr->EnableShadows();
   // vis_irr->SetShadows(true);
    vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
    vis_irr->ShowExplorer(true);
    vis = vis_irr;

    std::cout << "Starting simulation loop" << std::endl;

    int index = 0;
    while (rclcpp::ok() && vis->Run()) {
        //Rhex->HandleROS();
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        Rhex->SetLegDesiredVelocity(0,2);
        Rhex->SetLegDesiredVelocity(1,2);
        Rhex->SetLegDesiredVelocity(2,2);
        Rhex->SetLegDesiredVelocity(3,2);
        Rhex->SetLegDesiredVelocity(4,2);
        Rhex->SetLegDesiredVelocity(5,2);

        std::cout << "Torque: " << Rhex->m_leg_motors[0]->GetMotorTorque() << std::endl;
        std::cout << "Angle: " << Rhex->m_leg_motors[0]->GetMotorAngle() << std::endl;
        std::cout << "Speed: " << Rhex->m_leg_motors[0]->GetMotorAngleDt() << std::endl;
        vis->BindAll();
        sys.DoStepDynamics(step_size);
        index++;
        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }
    rclcpp::shutdown();

    std::cout << "Simulation loop ended" << std::endl;
    return 0;
}