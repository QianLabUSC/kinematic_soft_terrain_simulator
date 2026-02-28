#include "sim_tools/RoverFactory.h"

RoverFactory::RoverFactory(ChSystem* sys){
    m_sys = sys;
};
    

std::unique_ptr<Rover> RoverFactory::BuildRover(const ChVector3d& Rover_pos, const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions){
    m_Rover_pos = Rover_pos;
    
    std::shared_ptr<chrono::ChBody> body;
    body = BuildSimpleBody();

    //Build wheels
    std::vector<std::shared_ptr<chrono::ChBody>> wheels;
    for (size_t i = 0; i < wheel_positions.size(); ++i) {
        auto wheel = BuildSimpleWheel(wheel_positions[i], wheel_orientations[i], i);
        wheels.push_back(wheel);
    }

    //Attach wheels
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> wheel_motors;
    wheel_motors = AttachWheels(body, wheels);

    //Build & attach trusses
    std::vector<Truss> trusses;
    for(size_t i = 0; i < truss_positions.size(); ++i){
        auto truss = Truss(m_sys, body, truss_positions[i]);
        trusses.push_back(truss);
    }

    // Rotate the Rover to the given orientation
    RotateRover(body, wheels, trusses, orientation);

    //Build Rover
    std::unique_ptr<Rover> newRover = std::make_unique<Rover>(m_sys, body, wheels, wheel_motors, trusses, RoversBuilt); //body, wheels, wheel_motors, trusses

    RoversBuilt++;
    return std::move(newRover);
};

void RoverFactory::RotateRover(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>>& wheels, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation) {
    // Rotate the main body
    body->SetRot(orientation);

    // Rotate each wheel
    for (auto& wheel : wheels) {
        ChVector3d relative_pos = wheel->GetPos() - m_Rover_pos;
        ChVector3d rotated_pos = orientation.Rotate(relative_pos);
        wheel->SetPos(m_Rover_pos + rotated_pos);

        // Rotate the wheel itself
        wheel->SetRot(orientation * wheel->GetRot());
    }

    // Rotate each truss
    for (auto& truss : trusses) {
        auto truss_body = truss.base;
        ChVector3d relative_pos = truss_body->GetPos() - m_Rover_pos;
        ChVector3d rotated_pos = orientation.Rotate(relative_pos);
        truss_body->SetPos(m_Rover_pos + rotated_pos);
        truss_body->SetRot(orientation * truss_body->GetRot());
    }
}

std::shared_ptr<chrono::ChBody> RoverFactory::BuildSimpleBody(){
    std::shared_ptr<chrono::ChBodyEasyBox> body;
    body = chrono_types::make_shared<ChBodyEasyBox>(.967, .49, .152, 10, true, true, body_material);
    body->SetPos(m_Rover_pos+ChVector3d(0, 0, 0));
    body->SetMass(body_mass);
    std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/aluminium.png";
    body->GetVisualShape(0)->SetTexture(texture_body_filepath);
    body->SetInertia(body_inertia);
    body->EnableCollision(false);
    m_sys->Add(body);
    return body;
};

std::shared_ptr<chrono::ChBody> RoverFactory::BuildBody() {
    std::shared_ptr<chrono::ChBody> body;
    body = chrono_types::make_shared<ChBody>();

    m_sys->Add(body);
    body_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(1,0,0), ChVector3d(0,0,1), ChVector3d(0,-1,0)));
    body_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(0,1,0), ChVector3d(-1,0,0), ChVector3d(0,0,1)));

    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> body_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    body_vis_shape->SetMesh(body_trimesh);
    //body_vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/wheel.png";

    body_vis_shape->SetTexture(texture_body_filepath);
    body->AddVisualShape(body_vis_shape);
    body->SetMass(body_mass);
    //body->SetInertiaXX(body_inertia); //TODO: Change to full 3x3 inertia matrix
    body->SetPos(m_Rover_pos);
  
    body->AddCollisionShape(body_collision_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
    body->EnableCollision(false);

    // Set a unique name for the body
    body->SetName("body_Rover" + std::to_string(RoversBuilt));
    //body->SetFixed(true);
    return body;
};

std::shared_ptr<chrono::ChBody> RoverFactory::BuildSimpleWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num) {
    std::shared_ptr<chrono::ChBody> wheel;
    wheel = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Y,.305, .05, 100, true, true, wheel_material);
    m_sys->Add(wheel);
    std::string texture_wheel_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/wheel.png";
    //wheel->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    
    wheel->GetVisualShape(0)->SetTexture(texture_wheel_filepath);
    //wheel->SetMass(wheel_mass);
    //wheel->SetInertia(wheel_inertia);
    wheel->SetPos(m_Rover_pos+pos);
    wheel->EnableCollision(false);

    // Set a unique name for the leg
    wheel->SetName("leg"+std::to_string(wheel_num)+"_Rover" + std::to_string(RoversBuilt));

    return wheel;
};

std::shared_ptr<chrono::ChBody> RoverFactory::BuildWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num) {
    std::shared_ptr<chrono::ChBody> wheel;
    wheel = chrono_types::make_shared<ChBody>();

    m_sys->Add(wheel);

    wheel_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(wheel_mesh_path);
    wheel_collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(wheel_material, wheel_trimesh, false, false, 0.01);

    wheel_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    wheel_vis_shape->SetMesh(wheel_trimesh);
    wheel_vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));

    wheel->AddVisualShape(wheel_vis_shape);
    //wheel->SetMass(wheel_mass);
    //wheel->SetInertia(wheel_inertia);
    wheel->SetPos(m_Rover_pos+pos);
    wheel->SetRot(rot);
    wheel->AddCollisionShape(wheel_collision_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
    wheel->EnableCollision(false);

    // Set a unique name for the leg
    wheel->SetName("leg"+std::to_string(wheel_num)+"_Rover" + std::to_string(RoversBuilt));
    return wheel;
};

std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> RoverFactory::AttachWheels(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& wheels) {
    // Attach each wheel to the body
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> motors;
    int index = 0;
    for (const auto& wheel : wheels) {
        auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
        motor->Initialize(body, wheel, ChFrame<>(m_Rover_pos+wheel_positions[index], Q_ROTATE_Y_TO_Z));
        m_sys->Add(motor);
        motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0));
        motors.push_back(motor);
        index++;
    }
    return motors;
};