#include "sim_tools/SpiritFactory.h"

SpiritFactory::SpiritFactory(ChSystem* sys){
    m_sys = sys;
};

std::unique_ptr<Spirit> SpiritFactory::BuildSpirit(const ChVector3d& Spirit_pos,  const ChQuaternion<>& orientation, const std::vector<ChVector3d>& truss_positions, const std::vector<ChVector3d>& probe_positions){
    m_Spirit_pos = Spirit_pos;
    
    std::shared_ptr<chrono::ChBody> body = BuildBody();

    // Build legs
    std::vector<std::vector<std::shared_ptr<chrono::ChBody>>> leg_bodies;
    std::vector<std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> leg_motors;
    bool mirror = false;
    for (int i = 0; i < leg_positions.size(); ++i) {
        if(leg_positions[i][1] < 0){mirror=true;}
        else{mirror=false;}
        std::pair<std::vector<std::shared_ptr<chrono::ChBody>>, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> leg = BuildLeg(leg_positions[i]+body->GetPos(), mirror);
        
        leg_bodies.push_back(leg.first);
        leg_motors.push_back(leg.second);
    }

    // Attach legs
    mirror = false;
    for (int i = 0; i < leg_bodies.size(); ++i) {
        if(leg_positions[i][1] < 0){mirror=true;}
        else{mirror=false;}
        leg_motors[i] = AttachLeg(body, leg_bodies[i], leg_motors[i], mirror);
    }

    // Build & attach trusses
    std::vector<Truss> trusses;
    for(size_t i = 0; i < truss_positions.size(); ++i){
        auto truss = Truss(m_sys, body, truss_positions[i]);
        trusses.push_back(truss);
    }

    // Build & attach probes
    std::vector<Probe> probes;
    for(size_t i = 0; i < probe_positions.size(); ++i){
        Probe probe(m_sys, body, probe_positions[i]);
        probe.ConstructProbe(); // Example position
        probes.push_back(probe);
    }

    // Rotate the Spirit to the given orientation
    RotateSpirit(body, leg_bodies, trusses, orientation);

    // Build Spirit
    std::unique_ptr<Spirit> SpiritBot = std::make_unique<Spirit>(m_sys, body, leg_bodies, leg_motors, trusses, probes, SpiritsBuilt);

    SpiritsBuilt++;
    return std::move(SpiritBot);
};

void SpiritFactory::RotateSpirit(std::shared_ptr<chrono::ChBody> body, std::vector<std::vector<std::shared_ptr<chrono::ChBody>>>& leg_bodies, const std::vector<Truss>& trusses, const ChQuaternion<>& orientation) {
    // Rotate the main body
    body->SetRot(orientation);

    // Rotate each leg
    for (auto& leg : leg_bodies) {
        for (auto& leg_part : leg) {
            // Update position according to the rotated body
            ChVector3d relative_pos = leg_part->GetPos() - body->GetPos();
            ChVector3d rotated_pos = orientation.Rotate(relative_pos);
            leg_part->SetPos(body->GetPos() + rotated_pos);

            // Rotate the part itself
            leg_part->SetRot(orientation * leg_part->GetRot());
        }
    }

    // Rotate each truss
    for (auto& truss : trusses) {
        auto truss_body = truss.base;
        ChVector3d relative_pos = truss_body->GetPos() - body->GetPos();
        ChVector3d rotated_pos = orientation.Rotate(relative_pos);
        truss_body->SetPos(body->GetPos() + rotated_pos);
        truss_body->SetRot(orientation * truss_body->GetRot());
    }
}

std::shared_ptr<chrono::ChBody> SpiritFactory::BuildBody(){
    std::shared_ptr<chrono::ChBodyEasyBox> body;
    body = chrono_types::make_shared<ChBodyEasyBox>(.335, .24, .104, 10, true, true, material); // Swapped Y and Z dimensions
    body->SetPos(m_Spirit_pos+ChVector3d(0, 0, 0));
    body->SetMass(body_mass);
    body->GetVisualShape(0)->SetColor(darkgrey);
    std::string texture_body_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/black_aluminium.png";
    
    auto body_material = chrono_types::make_shared<ChVisualMaterial>();
    // Diffuse color for a sandy texture - light, warm tones
    body_material->SetDiffuseColor(ChColor(0.1f, 0.1f, 0.1f));  // Light brown/yellowish for sand

    // Specular color - very low for sand, to avoid shiny reflections
    body_material->SetSpecularColor(ChColor(0.02f, 0.02f, 0.02f));  // Very low reflectivity

    // Ambient color - subtle, warm reflection from environmental light
    body_material->SetAmbientColor(ChColor(0.02f, 0.02f, 0.02f));  // Slightly darker and warm tones
    body_material->SetKdTexture(texture_body_filepath); // Add texture if needed
    
    //body->GetVisualShape(0)->SetTexture(texture_body_filepath);
    body->GetVisualShape(0)->SetMaterial(0,body_material);
    body->SetInertia(body_inertia);
    body->EnableCollision(false);
    m_sys->Add(body);
    return body;
};

std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> SpiritFactory::AttachLeg(std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> leg_bodies, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors, bool mirrored){
    ChVector3d abs_motor_offset = m_abs_motor_offset;
    if(mirrored){abs_motor_offset = -abs_motor_offset;}
    
    auto abs_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    abs_motor->Initialize(body, leg_bodies[0], ChFrame<>(leg_bodies[0]->GetPos()+abs_motor_offset, Q_ROTATE_Z_TO_Y)); // Changed from Y to Z
    m_sys->Add(abs_motor);
    leg_motors.push_back(abs_motor);
    return leg_motors;
};

std::pair<std::vector<std::shared_ptr<chrono::ChBody>>, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> SpiritFactory::BuildLeg(ChVector3d leg_pos, bool mirrored){
    std::vector<std::shared_ptr<chrono::ChBody>> leg_bodies;
    std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors;

    ChVector3d upper_pos = m_upper_pos;
    ChVector3d lower_pos = m_lower_pos;
    ChVector3d toe_pos = m_toe_pos;

    if(mirrored){
        upper_pos = ChVector3d(upper_pos.x(), -upper_pos.y(), upper_pos.z()); // Adjusted for Z-up
        lower_pos = ChVector3d(lower_pos.x(), -lower_pos.y(), lower_pos.z()); // Adjusted for Z-up
        toe_pos = ChVector3d(toe_pos.x(), -toe_pos.y(), toe_pos.z()); // Adjusted for Z-up
    }

    std::shared_ptr<chrono::ChBodyEasyCylinder> hip = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Y,.055, .08, .1, true, true, material); // Changed axis to Y
    hip->SetPos(leg_pos);
    hip->SetMass(hip_mass);
    hip->SetInertia(hip_inertia);
    hip->EnableCollision(false);
    std::string texture_hip_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/aluminium.png";
    hip->GetVisualShape(0)->SetTexture(texture_hip_filepath);
    //hip->GetVisualShape(0)->SetColor(darkgrey);
    m_sys->Add(hip);
    leg_bodies.push_back(hip);

    std::shared_ptr<chrono::ChBodyEasyBox> upper = chrono_types::make_shared<ChBodyEasyBox>(.206, .022, .055, 10, true, true, material); // Swapped Y and Z dimensions
    upper->SetPos(leg_pos+upper_pos);
    upper->SetMass(upper_mass);
    upper->SetInertia(upper_inertia);
    upper->EnableCollision(false);
    //std::string texture_hip_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/aluminium.png";
    upper->GetVisualShape(0)->SetTexture(texture_hip_filepath);
    //upper->GetVisualShape(0)->SetColor(silver);
    m_sys->Add(upper);
    leg_bodies.push_back(upper);

    std::shared_ptr<chrono::ChBodyEasyCylinder> lower = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::X,.013, .206, .1, true, true, material); // Changed axis to Z
    lower->SetPos(leg_pos+lower_pos);
    lower->SetMass(lower_mass);
    lower->SetInertia(lower_inertia);
    lower->EnableCollision(false);
    //std::string texture_hip_filepath = std::string(getenv("HOME"))+"/ros2_ws/src/chrono_sim/meshes/aluminium.png";
    lower->GetVisualShape(0)->SetTexture(texture_hip_filepath);
    //lower->GetVisualShape(0)->SetColor(darkgrey);
    m_sys->Add(lower);
    leg_bodies.push_back(lower);

    std::shared_ptr<chrono::ChBodyEasySphere> toe = chrono_types::make_shared<ChBodyEasySphere>(.02, 10, true, true, material);
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
    shoulder_motor->Initialize(hip, upper, ChFrame<>(hip->GetPos(), Q_ROTATE_Y_TO_Z));
    m_sys->Add(shoulder_motor);

    std::shared_ptr<chrono::ChLinkMotorRotationTorque> knee_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    // Frame is half the length of the upper box in the Z direction plus its position
    knee_motor->Initialize(upper, lower, ChFrame<>(ChVector3d(-.103, 0, 0)+upper->GetPos(), Q_ROTATE_Y_TO_Z));
    m_sys->Add(knee_motor);

    leg_motors.push_back(knee_motor);
    leg_motors.push_back(shoulder_motor);

    return std::make_pair(leg_bodies, leg_motors);
};