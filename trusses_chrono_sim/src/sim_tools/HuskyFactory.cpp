#include "sim_tools/HuskyFactory.h"
#include "sim_tools/RoverFactory.h"

HuskyFactory::HuskyFactory(ChSystem* sys){
    m_sys = sys;
};
    

std::unique_ptr<Rover> HuskyFactory::BuildHusky(const ChVector3d& Husky_pos, const std::vector<ChVector3d>& truss_positions){
    m_Husky_pos = Husky_pos;
    
    std::shared_ptr<chrono::ChBody> body;
    body = BuildBody();

    //Build wheels (same as rover)
    std::vector<std::shared_ptr<chrono::ChBody>> wheels;
    for (size_t i = 0; i < wheel_positions.size(); ++i) {
        auto wheel = BuildSimpleWheel(wheel_positions[i], wheel_orientations[i], i);
        wheels.push_back(wheel);
    }

    //Attach wheels (same as rover)
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> wheel_motors;
    wheel_motors = AttachWheels(body, wheels);

    //Build & attach trusses (same as rover)
    std::vector<Truss> trusses;
    for(size_t i = 0; i < truss_positions.size(); ++i){
        auto truss = Truss(m_sys, body, truss_positions[i]);
        trusses.push_back(truss);
    }

    //Build Husky
    std::unique_ptr<Rover> newHusky = std::make_unique<Rover>(m_sys, body, wheels, wheel_motors, trusses, HuskiesBuilt); //body, wheels, wheel_motors, trusses

    HuskiesBuilt++;
    return std::move(newHusky);
};

std::shared_ptr<chrono::ChBody> HuskyFactory::BuildBody() {
    std::shared_ptr<chrono::ChBodyEasyBox> body;
    body = chrono_types::make_shared<ChBodyEasyBox>(.30709, .0635, .413, 10, true, true, material);
    body->SetPos(m_Husky_pos+ChVector3d(0, 0, 0));
    body->SetMass(body_mass);
    body->GetVisualShape(0)->SetColor(darkgrey);
    body->SetInertia(body_inertia);
    body->EnableCollision(false);
    m_sys->Add(body);
    return body;
};

std::shared_ptr<chrono::ChBody> HuskyFactory::BuildSimpleWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num) {
    std::shared_ptr<chrono::ChBody> wheel;
    wheel = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.2399792/2, .05, 75, true, true, wheel_material);
    m_sys->Add(wheel);
    
    wheel->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.3f));

    //wheel->SetMass(wheel_mass);
    //wheel->SetInertia(wheel_inertia);
    wheel->SetPos(m_Husky_pos+pos);
    wheel->EnableCollision(false);

    // Set a unique name for the leg
    wheel->SetName("leg"+std::to_string(wheel_num)+"_Husky" + std::to_string(HuskiesBuilt));

    return wheel;
};


std::shared_ptr<chrono::ChBody> HuskyFactory::BuildWheel(const ChVector3d& pos, const ChQuaternion<>& rot, int wheel_num) {
    std::shared_ptr<chrono::ChBody> wheel;
    wheel = chrono_types::make_shared<ChBody>();

    m_sys->Add(wheel);

    wheel_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(wheel_mesh_path);
    wheel_collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(wheel_material, wheel_trimesh, false, false, 0.01);

    wheel_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    wheel_vis_shape->SetMesh(wheel_trimesh);
    wheel_vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));

    wheel->AddVisualShape(wheel_vis_shape);
    wheel->SetMass(wheel_mass);
    wheel->SetInertia(wheel_inertia);
    wheel->SetPos(m_Husky_pos+pos);
    wheel->SetRot(rot);
    wheel->AddCollisionShape(wheel_collision_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
    wheel->EnableCollision(false);

    // Set a unique name for the leg
    wheel->SetName("leg"+std::to_string(wheel_num)+"_Husky" + std::to_string(HuskiesBuilt));
    return wheel;
};

std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> HuskyFactory::AttachWheels(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& wheels) {
    // Attach each wheel to the body
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> motors;
    int index = 0;
    for (const auto& wheel : wheels) {
        auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
        motor->Initialize(body, wheel, ChFrame<>(m_Husky_pos+wheel_positions[index], QuatFromAngleY(-CH_PI_2)));
        m_sys->Add(motor);
        motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0));
        motors.push_back(motor);
        index++;
    }
    return motors;
};