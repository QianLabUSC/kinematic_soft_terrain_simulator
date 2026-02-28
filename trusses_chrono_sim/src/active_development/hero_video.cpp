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
#include <filesystem>

//Sim Tools
#include "sim_tools/RoverFactory.h"
#include "sim_tools/Rover.h"

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
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        if (loc.x() < 1) {
            Bekker_Kphi = 0.2e6;
            Bekker_Kc = 0;
            Bekker_n = 1.1;
            Mohr_cohesion = 0;
            Mohr_friction = 30;
            Janosi_shear = 0.01;
            elastic_K = .3e6;
            damping_R = 3e4;
        } else {
            Bekker_Kphi = .0001e6;
            Bekker_Kc = 0;
            Bekker_n = 1.1;
            Mohr_cohesion = 15000;
            Mohr_friction = 30;
            Janosi_shear = 0.01;
            elastic_K = .0002e6;
            damping_R = 3e4;
        }
    }
};

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
    sys.SetSolverType(ChSolver::Type::PSOR);

    //This will change how large the integration between timesteps is, but the larger the value the more unstable the sim gets. This is a good starting point.
    double step_size = .001; 

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

// Initialize the geometry of the soil
    double width = 30;
    double length = 30;
    double mesh_resolution = 0.02;
    mterrain.Initialize(std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/sand3.bmp", length, width, 0, .15, mesh_resolution);
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
    

    mterrain.EnableBulldozing(false);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    */
    //Visualize a mesh for the terrain
    auto my_params = chrono_types::make_shared<MySoilParams>();
    mterrain.RegisterSoilParametersCallback(my_params);
    mterrain.GetMesh()->SetWireframe(false);

    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE,0, .025);

    // Set the texture or apply colors as needed
    auto terrain_material = chrono_types::make_shared<ChVisualMaterial>();
    // Diffuse color for a sandy texture - light, warm tones
    terrain_material->SetDiffuseColor(ChColor(0.9f, 0.8f, 0.6f));  // Light brown/yellowish for sand

    // Specular color - very low for sand, to avoid shiny reflections
    terrain_material->SetSpecularColor(ChColor(0.02f, 0.02f, 0.02f));  // Very low reflectivity

    // Ambient color - subtle, warm reflection from environmental light
    terrain_material->SetAmbientColor(ChColor(0.6f, 0.55f, 0.45f));  // Slightly darker and warm tones
    terrain_material->SetKdTexture(std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/sand3_flipped.png"); // Add texture if needed

    mterrain.GetMesh()->SetMaterial(0, terrain_material);

    //Rover_position_W
    //p_WRh0_W or p_WRhset_W
    ChVector3d Rover_position = ChVector3d(0, 0, 0.4);
    ChQuaternion orientation= ChQuaternion(0,0,0,0);
    //truss_positions_B
    //p_Rh0Tset_Rh0
    std::vector<ChVector3d> truss_positions = {};
    std::vector<ChVector3d> rover1_truss_positions = {ChVector3d(0,0,.1)};
    // Create a Rover robot
    RoverFactory RoverFactory(&sys);

    auto rover0 = RoverFactory.BuildRover(Rover_position, QUNIT , std::vector<ChVector3d>{});
    rover0->AddMovingPatchesAndCollisions(mterrain);



    std::shared_ptr<ChVisualSystem> vis;
    auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->SetWindowSize(800, 600);
    vis_irr->SetWindowTitle("Hero Video");
    vis_irr->SetShadows(true);
    vis_irr->SetCameraVertical(CameraVerticalDir::Z);
    vis_irr->Initialize();
    //vis_irr->AddSkyBox();
    AddCustomSkyBox(vis_irr, std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/");
    //vis_irr->AddTypicalLights();)
    vis_irr->AddLightWithShadow(ChVector3d(-75, 37.5, 40), ChVector3d(0,0,0), 750, 300, 500, 180, 512, ChColor(1, .95f, .85f), true, false); //Angle, res, color, directional, clipborder
    vis_irr->AddLightWithShadow(ChVector3d(75, -37.5, 40), ChVector3d(0,0,0), 200, 300, 500, 180, 512, ChColor(1, .95f, .85f), true, false); //Angle, res, color, directional, clipborder
    //vis_irr->AddLightWithShadow(ChVector3d(30, 30, 160), ChVector3d(0,0,0), 280, 100, 200, 45);
    //vis_irr->AddLightWithShadow(ChVector3d(30, -30, 160), ChVector3d(0,0,0), 280, 100,200 ,45);
    vis_irr->AddCamera(ChVector3d(2.4, -1.2, 2.4), ChVector3d(0, 0, 0.8));
    vis_irr->AttachSystem(&sys);
    vis_irr->EnableShadows();
   // vis_irr->SetShadows(true);
    vis = vis_irr;

    std::cout << "Starting simulation loop" << std::endl;
    double time = 0;
    bool stuck = false;
    bool spawned_rover1 = false;
    std::unique_ptr<Rover> rover1;
    std::string folder_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "output_frames";
    int index = 0;
    while (rclcpp::ok() && vis->Run()) {
        time = sys.GetChTime();
        std::cout << "Time: " << time << std::endl;
        //rover0->HandleROS();
        //rover1->HandleROS();
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        vis->SetCameraTarget(rover0->m_body->GetPos());
        if(time > 0){
  
        //rover0->m_wheel_motors[0]->SetTorqueFunction(chrono_types::make_shared<ChFunctionConst>(10));
        rover0->SetWheelDesiredVelocity(0,1);
        rover0->SetWheelDesiredVelocity(1,1);
        rover0->SetWheelDesiredVelocity(2,1);
        rover0->SetWheelDesiredVelocity(3,1);
        if(rover0->m_wheel_motors[0]->GetMotorTorque() > 400 || stuck==true){
            rover0->SetWheelDesiredVelocity(0,0);
            rover0->SetWheelDesiredVelocity(1,0);
            rover0->SetWheelDesiredVelocity(2,0);
            rover0->SetWheelDesiredVelocity(3,0);
            stuck = true;
            if(spawned_rover1==false && time > 5){
                spawned_rover1 = true;
                rover1 = RoverFactory.BuildRover(ChVector3d(-1,0,.4), QUNIT, rover1_truss_positions);
                rover1->AddMovingPatchesAndCollisions(mterrain);
                rover1->DockTruss(0, rover0->m_body, ChVector3d(-.5,0,.1));
            }if(time > 5.1){
                rover0->SetWheelDesiredVelocity(0,-.5);
                rover0->SetWheelDesiredVelocity(1,-.5);
                rover0->SetWheelDesiredVelocity(2,-.5);
                rover0->SetWheelDesiredVelocity(3,-.5);

                rover1->SetWheelDesiredVelocity(0,-2);
                rover1->SetWheelDesiredVelocity(1,-2);
                rover1->SetWheelDesiredVelocity(2,-2);
                rover1->SetWheelDesiredVelocity(3,-2);
            }
        }
    }

        //std::string filename = folder_path + "/frame_" + std::to_string(index) + ".png";
        //vis_irr->WriteImageToFile(filename);
       
        vis->BindAll();
        sys.DoStepDynamics(step_size);
        index = index+1;
        //Handles CTRL-C quitting nicely
        if (g_terminate_requested.load()) {
        break;
        }
    }
    rclcpp::shutdown();

    std::cout << "Simulation loop ended" << std::endl;
    return 0;
}