#include "sim_tools/RHexFactory.h"

RHexFactory::RHexFactory(ChSystem* sys){
    m_sys = sys;
};

std::unique_ptr<RHex> RHexFactory::BuildRHex(const ChVector3d& RHex_pos, const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions, double bodymass, double legmass, double counterweight_mass){
    m_RHex_pos = RHex_pos;
    SetMass(bodymass,legmass);
    std::shared_ptr<chrono::ChBody> body;
    body = BuildSimpleBody(counterweight_mass);

    //Build legs
    std::vector<std::shared_ptr<chrono::ChBody>> legs;
    for (size_t i = 0; i < leg_positions.size(); ++i) {
        auto leg = BuildLeg(leg_positions[i], leg_orientations[i], i);
        legs.push_back(leg);
    }

    //Attach legs
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors;
    leg_motors = AttachLegs(body, legs);

    //Build & attach trusses
    std::vector<Truss> trusses;
    for(size_t i = 0; i < truss_positions.size(); ++i){
        auto truss = Truss(m_sys, body, truss_positions[i]);
        trusses.push_back(truss);
    }
    
    // Rotate the RHex to the given orientation
    RotateRHex(body, legs, trusses, orientation);

    //Build RHex
    //RHex Rhex = RHex(m_sys, body, legs, leg_motors, trusses); //body, legs, leg_motors, trusses
    std::unique_ptr<RHex> Rhex = std::make_unique<RHex>(m_sys, body, legs, leg_motors, trusses, RHexsBuilt); //body, legs, leg_motors, trusses

    RHexsBuilt++;
    
    return std::move(Rhex);
};
void RHexFactory::SetMass(double bodym, double legm){
    std::cout << "body mass "<<body_mass << std::endl;
    std::cout << "leg mass "<<leg_mass << std::endl;
    body_mass=bodym;
    leg_mass=legm;
    std::cout << "body mass "<<body_mass << std::endl;
    std::cout << "leg mass "<<leg_mass << std::endl;
}
void RHexFactory::RotateRHex(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>>& legs, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation) {
    // Rotate the main body
    body->SetRot(orientation);

    // Rotate each leg
    for (auto& leg : legs) {
        ChVector3d relative_pos = leg->GetPos() - m_RHex_pos;
        ChVector3d rotated_pos = orientation.Rotate(relative_pos);
        leg->SetPos(m_RHex_pos + rotated_pos);

        // Rotate the leg itself
        leg->SetRot(orientation * leg->GetRot());
        
    }

    // Rotate each truss
    for (auto& truss : trusses) {
        auto truss_body = truss.base;
        ChVector3d relative_pos = truss_body->GetPos() - m_RHex_pos;
        ChVector3d rotated_pos = orientation.Rotate(relative_pos);
        truss_body->SetPos(m_RHex_pos + rotated_pos);
        truss_body->SetRot(orientation * truss_body->GetRot());
    }
}

std::shared_ptr<chrono::ChBody> RHexFactory::BuildSimpleBody(double counterweight_mass){
    std::shared_ptr<chrono::ChBodyEasyBox> body;
    body = chrono_types::make_shared<ChBodyEasyBox>(.967, .49, .152, 10, true, true, body_material);
    body->SetPos(m_RHex_pos+ChVector3d(0, 0, 0));
    body->SetMass(body_mass);
    body->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.0f, 0.0f)); // USC Cardinal Red body color
    // std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/black_aluminium.png";
    // body->GetVisualShape(0)->SetTexture(texture_body_filepath);
    body->SetInertia(body_inertia);
    body->EnableCollision(false);
    
    m_sys->Add(body);
    
    // Add counterweight to shift center of mass backward (behind the robot)
    if (counterweight_mass > 0.0) {
        auto counterweight = chrono_types::make_shared<ChBodyEasyBox>(0.05, 0.05, 0.05, 2000, true, true, body_material);
        counterweight->SetPos(body->GetPos() + ChVector3d(0.1, 0, 0.076)); // Add weight 10cm forward and at body center height
        counterweight->SetMass(counterweight_mass); // Use passed counterweight mass
        counterweight->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.0f)); // Dark red counterweight
        m_sys->Add(counterweight);
        
        // Add rigid link to connect counterweight to body (both bodies now in system)
        auto rigid_link = chrono_types::make_shared<ChLinkLockLock>();
        rigid_link->Initialize(body, counterweight, ChFrame<>(body->GetPos() + ChVector3d(0.1, 0, 0.076)));
        m_sys->Add(rigid_link);
        
        std::cout << "Added counterweight with mass: " << counterweight_mass << " kg rigidly attached to robot" << std::endl;
    }
    return body;
};

std::shared_ptr<chrono::ChBody> RHexFactory::BuildBody(){
    std::shared_ptr<chrono::ChBody> body;
    body = chrono_types::make_shared<ChBody>();

    m_sys->Add(body);
    body_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(1,0,0), ChVector3d(0,0,1), ChVector3d(0,-1,0)));
    body_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(0,1,0), ChVector3d(-1,0,0), ChVector3d(0,0,1)));

    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> body_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    body_vis_shape->SetMesh(body_trimesh);
    body_vis_shape->SetColor(ChColor(0.6f, 0.0f, 0.0f)); // USC Cardinal Red body color
    // std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/black_aluminium.png";
    // body->GetVisualShape(0)->SetTexture(texture_body_filepath);
    body->AddVisualShape(body_vis_shape);
    body->SetMass(body_mass);
    body->SetInertiaXX(body_inertia); //TODO: Change to full 3x3 inertia matrix
    body->SetPos(m_RHex_pos);
    body->AddCollisionShape(body_collision_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
    body->EnableCollision(false);

    // Set a unique name for the body
    body->SetName("body_RHex" + std::to_string(RHexsBuilt));
    //body->SetFixed(true);
    return body;
};

std::shared_ptr<chrono::ChBody> RHexFactory::BuildLeg(const ChVector3d& pos, const ChQuaternion<>& rot, int leg_num){
    
    std::shared_ptr<chrono::ChBody> leg;
    leg = chrono_types::make_shared<ChBody>();

    m_sys->Add(leg);
 
    leg_trimesh = ChTriangleMeshConnected::CreateFromSTLFile(leg_mesh_path);
    leg_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(1,0,0), ChVector3d(0,-1,0), ChVector3d(0,0,-1)));
    leg_trimesh->Transform(ChVector3d(0,0,0), ChMatrix33(ChVector3d(-1,0,0), ChVector3d(0,-1,0), ChVector3d(0,0,1)));
    leg_collision_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(leg_material, leg_trimesh, false, false, 0.01);

    leg_vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    leg_vis_shape->SetMesh(leg_trimesh);
    leg_vis_shape->SetColor(ChColor(1.0f, 0.8f, 0.0f)); // USC Gold/Yellow leg color
    leg->AddVisualShape(leg_vis_shape);
    // std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/aluminium.png";
    // std::cout << "Texture file path: " << texture_body_filepath << std::endl;

    // auto visual_shape = leg->GetVisualShape(0);
    // if (!visual_shape) {
    //     std::cerr << "Error: No visual shape found for leg before setting texture!" << std::endl;
    // } else {
    //     visual_shape->SetTexture(texture_body_filepath);
    // }


    // leg->GetVisualShape(0)->SetTexture(texture_body_filepath);
    leg->AddVisualShape(leg_vis_shape);
    leg->SetMass(leg_mass);
    //leg->SetInertia(leg_inertia);
    leg->SetPos(m_RHex_pos+pos);
    leg->SetRot(rot);
    leg->AddCollisionShape(leg_collision_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
    leg->EnableCollision(false);

    // Set a unique name for the leg
    leg->SetName("leg"+std::to_string(leg_num)+"_RHex" + std::to_string(RHexsBuilt));

    return leg;
};
 
std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> RHexFactory::AttachLegs(const std::shared_ptr<chrono::ChBody>& body, const std::vector<std::shared_ptr<chrono::ChBody>>& legs) {
    // Attach each leg to the body
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> motors;
    int index = 0;
    for (const auto& leg : legs) {
        auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
        std::cout << Q_ROTATE_Y_TO_Z[0] << " " << Q_ROTATE_Y_TO_Z[1] << " " << Q_ROTATE_Y_TO_Z[2] << " " << Q_ROTATE_Y_TO_Z[3] << std::endl;

        motor->Initialize(body, leg, ChFrame<>(m_RHex_pos+leg_positions[index], ChQuaternion<>(0.707107,-0.707107,0,0)));
        m_sys->Add(motor);
        motor->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(0));
        motors.push_back(motor);
        index++;
    }
    return motors;
};
