#include "sim_tools/Truss.h"
/*
Truss::Truss(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, const ChVector3d& relative_truss_pos) {

    m_sys = sys;
    m_body = body;
    m_truss_pos = relative_truss_pos;

    base->SetPos(m_body->GetPos()+m_truss_pos);
    base->GetVisualShape(0)->SetColor(ChColor(0.8f,0.0f,0.2f));
    m_sys->Add(base);

    arm->SetPos(m_body->GetPos()+m_truss_pos+ChVector3d(0,0,.5));
    arm->GetVisualShape(0)->SetColor(ChColor(0.945f,0.35f,0.133f));
    arm->EnableCollision(false);
    m_sys->Add(arm);

    coupler->SetPos(m_body->GetPos()+m_truss_pos+ChVector3d(0,0,1));
    coupler->GetVisualShape(0)->SetColor(ChColor(0.8f,0.0f,0.2f));
    m_sys->Add(coupler);

    base_to_arm_link = std::make_shared<ChLinkLockLock>();
    base_to_arm_link->Initialize(base, arm, ChFrame<>(base->GetPos(), QUNIT));
    m_sys->AddLink(base_to_arm_link);

    base_to_body_link = std::make_shared<ChLinkLockSpherical>();
    base_to_body_link->Initialize(m_body, base, ChFrame<>(base->GetPos(), QUNIT));
    m_sys->AddLink(base_to_body_link);

    motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

    ChFrame<> motor_frame(base->GetPos(), QUNIT);
    motor->Initialize(base, coupler, motor_frame);
    m_sys->Add(motor);
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));


};
*/

Truss::Truss(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, const ChVector3d& relative_truss_pos) 
    : m_sys(sys), m_body(body), m_truss_pos(relative_truss_pos) {
}

void Truss::ConstructTruss(const ChVector3d& desired_coupler_pos) {
    // Calculate the direction vector from base to coupler
    ChVector3d base_pos = m_body->GetPos() + m_body->GetRot().Rotate(m_truss_pos);
    ChVector3d direction = (desired_coupler_pos - base_pos).GetNormalized();
    double truss_length = (desired_coupler_pos - base_pos).Length();

    // Create a quaternion that rotates from the Z-axis to the desired direction
    ChQuaternion<> direction_rot = QuatFromVec2Vec(VECT_Z, direction);

    // Set the texture or apply colors as needed
    auto truss_material = chrono_types::make_shared<ChVisualMaterial>();
    // Diffuse color for a sandy texture - light, warm tones
    truss_material->SetDiffuseColor(ChColor(0.464f, 0.53f, 0.597f));  // Light brown/yellowish for sand

    // Specular color - very low for sand, to avoid shiny reflections
    truss_material->SetSpecularColor(ChColor(0.00f, 0.0f, 0.00f));  // Very low reflectivity

    // Ambient color - subtle, warm reflection from environmental light
    truss_material->SetAmbientColor(ChColor(0.0f, 0.0f, 0.00f));  // Slightly darker and warm tones

    // Set the base position and orientation
    base->SetPos(base_pos);
    base->SetRot(direction_rot);  // Orient the base to face along the direction of the arm
    base->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.2f));
    base->GetVisualShape(0)->SetMaterial(0,truss_material);
    m_sys->Add(base);

    // Set the coupler position and orientation to face the base
    coupler->SetPos(desired_coupler_pos);
    coupler->SetRot(direction_rot);  // Orient the coupler to face the base along the arm direction
    coupler->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.2f));
    coupler->GetVisualShape(0)->SetMaterial(0,truss_material);
    m_sys->Add(coupler);

    // Create the arm, positioned between the base and coupler
    arm = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z, .02, truss_length, .1, material);

    // Set the arm position at the midpoint between base and coupler
    ChVector3d arm_pos = base_pos + direction * (truss_length / 2);
    arm->SetPos(arm_pos);

    // Set the texture or apply colors as needed
    auto arm_material = chrono_types::make_shared<ChVisualMaterial>();

    // Diffuse color for a sandy texture - light, warm tones
    arm_material->SetDiffuseColor(ChColor(0.25f, 0.125f, 0.00f)); // Light brown/yellowish for sand

    // Specular color - very low for sand, to avoid shiny reflections
    arm_material->SetSpecularColor(ChColor(0.00f, 0.00f, 0.00f));  // Very low reflectivity

    // Ambient color - subtle, warm reflection from environmental light
    arm_material->SetAmbientColor(ChColor(0.0f, 0.0f, 0.0f));  // Slightly darker and warm tones

    // Align the arm along the direction vector using the calculated rotation
    arm->SetRot(direction_rot);
    arm->GetVisualShape(0)->SetColor(ChColor(0.945f, 0.35f, 0.133f));
    arm->EnableCollision(false);
    arm->GetVisualShape(0)->SetMaterial(0,arm_material);
    m_sys->Add(arm);

    // Create the links for base-to-arm and base-to-coupler
    base_to_arm_link = std::make_shared<ChLinkLockLock>();
    base_to_arm_link->Initialize(base, arm, ChFrame<>(base->GetPos(), direction_rot));
    m_sys->AddLink(base_to_arm_link);

    base_to_body_link = std::make_shared<ChLinkLockSpherical>();
    base_to_body_link->Initialize(m_body, base, ChFrame<>(base->GetPos(), direction_rot));
    m_sys->AddLink(base_to_body_link);

    // Motor setup remains the same, aligned with the base and coupler
    motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    ChFrame<> motor_frame(base->GetPos(), direction_rot);
    motor->Initialize(base, coupler, motor_frame);
    m_sys->Add(motor);
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    motor->SetAvoidPositionDrift(false);
}

void Truss::SetArmPIDGains(double kp, double ki, double kd){
    pidController.setGains(kp, ki, kd);
};

void Truss::SetArmDesiredLength(double length){
    desired_length = length;
    double controlSignal = pidController.calculate(length, GetLength(),m_sys->GetStep());
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(-controlSignal));
};


double Truss::GetLength(){
    ChVector3d base_pos = base->GetPos();
    ChVector3d coupler_pos = coupler->GetPos();
    double truss_length = (coupler_pos - base_pos).Length();
    return truss_length;
};

void Truss::UpdateArmLength(){

    ChVector3d old_vel = arm->GetLinVel();
    ChVector3d old_ang_vel = arm->GetAngVelLocal();

    ChVector3d old_accel = arm->GetLinAcc();
    ChVector3d old_ang_accel = arm->GetAngAccLocal();

    // Remove the old arm and link
    m_sys->RemoveLink(base_to_arm_link);
    m_sys->Remove(arm);
        
    // Calculate the new length for the arm based on the distance between base and coupler
    ChVector3d base_pos = base->GetPos();
    ChVector3d coupler_pos = coupler->GetPos();
    double truss_length = (coupler_pos - base_pos).Length();

    // Create a new arm with the updated length
    arm = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z, .02, truss_length, .1, material);

    // Determine the local offset position of the arm relative to the base
    ChVector3d local_offset(0, truss_length / 2, 0);

    // Rotate the local offset by the base's rotation to get the world position
    ChVector3d rotated_offset = base->GetRot().Rotate(local_offset);

    // Set the position for the new arm: base position plus rotated offset
    ChVector3d arm_pos = base_pos + rotated_offset;
    arm->SetPos(arm_pos);
    
    // Set the rotation for the new arm to align with the direction vector
    // The default axis for the cylinder is along the Y-axis (0, 1, 0)
    arm->SetRot(base->GetRot());

    // Set visual properties and collision settings
    arm->GetVisualShape(0)->SetColor(ChColor(0.945f, 0.35f, 0.133f));
    arm->EnableCollision(false);

    arm->SetAngVelLocal(old_ang_vel);
    arm->SetLinVel(old_vel);

    arm->SetLinAcc(old_accel);
    arm->SetAngAccLocal(old_ang_accel);

    m_sys->Add(arm);

    base_to_arm_link = std::make_shared<ChLinkLockLock>();
    base_to_arm_link->Initialize(base, arm, ChFrame<>(base->GetPos(), QUNIT));
    m_sys->Add(base_to_arm_link);
};