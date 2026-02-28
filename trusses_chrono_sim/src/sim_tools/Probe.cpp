#include "sim_tools/Probe.h"


Probe::Probe(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, const ChVector3d& relative_probe_pos) 
    : m_sys(sys), m_body(body), m_probe_pos(relative_probe_pos) {
}

void Probe::ConstructProbe() {
    ChVector3d& desired_x_pos = m_probe_pos;
    ChVector3d& desired_probe_tip_pos = m_probe_pos;
    // Calculate the direction vector from base to probe_tip
    ChVector3d base1_pos = m_body->GetPos() + m_body->GetRot().Rotate(m_probe_pos);
    //x pos and z pos for base2 of probe based on the final probe position
    //x component of final pos
    ChVector3d desired_x_position = desired_x_pos;
    //z component of final pos
    //ChVector3d desired_z_position = desired_probe_tip_pos;
    //horizontal direction and vertical direction should be constant relative to the robot position
    ChVector3d horizontal_direction = (desired_x_position - base1_pos).GetNormalized();
    ChVector3d vertical_direction = (desired_probe_tip_pos - desired_x_position).GetNormalized();
    //maybe set direction to just straight down.
    double horizontal_length = (desired_x_position - base1_pos).Length();
    double vertical_length = (desired_probe_tip_pos - desired_x_position).Length();

   

    // Create a quaternion that rotates from the Z-axis to the desired direction
    ChQuaternion<> horizontal_direction_rot = QuatFromVec2Vec(VECT_Z, horizontal_direction);
    ChQuaternion<> vertical_direction_rot = QuatFromVec2Vec(VECT_Z, vertical_direction);
    // Set the texture or apply colors as needed
    auto probe_material = chrono_types::make_shared<ChVisualMaterial>();

    // Ambient color - subtle, warm reflection from environmental light
    probe_material->SetAmbientColor(ChColor(0.0f, 0.0f, 0.00f));  // Slightly darker and warm tones

    // Set the base position and orientation
    base1->SetPos(base1_pos);
    base1->SetRot(horizontal_direction);  // Orient the base to face along the direction of the arm
    base1->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.2f));
    base1->GetVisualShape(0)->SetMaterial(0,probe_material);
    m_sys->Add(base1);


    // Set the base2 position and orientation to face base1
    base2->SetPos(desired_x_position);
    base2->SetRot(vertical_direction);  // Orient the probe_tip to face the base along the arm direction
    base2->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.2f));
    base2->GetVisualShape(0)->SetMaterial(0,probe_material);
    m_sys->Add(base2);

    // Set the probe_tip position and orientation to face base2
    probe_tip->SetPos(desired_probe_tip_pos);
    probe_tip->SetRot(vertical_direction_rot);  // Orient the probe_tip to face the base along the arm direction
    probe_tip->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.0f, 0.2f));
    probe_tip->GetVisualShape(0)->SetMaterial(0,probe_material);
    m_sys->Add(probe_tip);


    base_to_body_link = std::make_shared<ChLinkLockSpherical>();
    base_to_body_link->Initialize(m_body, base1, ChFrame<>(base1->GetPos(), horizontal_direction_rot));
    m_sys->AddLink(base_to_body_link);

    // Motor setup remains the same, alpigned with the base and probe_tip
    motor = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    ChFrame<> motor_frame(base1->GetPos(), horizontal_direction_rot);
    motor->Initialize(base1, base2, motor_frame);
    m_sys->Add(motor);
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    motor->SetAvoidPositionDrift(false);

    // Motor setup remains the same, aligned with the base and probe_tip
    motor2 = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
    ChFrame<> motor2_frame(base2->GetPos(), vertical_direction_rot);
    motor2->Initialize(base2, probe_tip, motor2_frame);
    m_sys->Add(motor2);
    motor2->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(0));
    motor2->SetAvoidPositionDrift(false);
}

void Probe::SetArmPIDGains(double kp, double ki, double kd){
    pidController.setGains(kp, ki, kd);
};

void Probe::SetArmDesiredLengths(double lengthhorizontal, double lengthvertical){
    double horizontal_desired_length = lengthhorizontal;
    double vertical_desired_length = lengthvertical;
    std::cout << "desired length" << lengthvertical << std:: endl;
    double horizontal_controlSignal = pidController.calculate(lengthhorizontal, GetHorizontalLength(),m_sys->GetStep());
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(-horizontal_controlSignal));
    double vertical_controlSignal = pidController.calculate(lengthvertical, GetVerticalLength(),m_sys->GetStep());
    motor2->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(-vertical_controlSignal));
    std::cout << vertical_controlSignal << std::endl;
    std::cout << probe_tip->GetPos() << std::endl;


};


double Probe::GetVerticalLength(){
    ChVector3d base2_pos = base2->GetPos();
    ChVector3d probe_tip_pos = probe_tip->GetPos();
    double probe_length = (probe_tip_pos - base2_pos).Length();
    std::cout << "actual length " << probe_length << std:: endl;
    return probe_length;
};

double Probe::GetHorizontalLength(){
    ChVector3d base1_pos = base1->GetPos();
    ChVector3d base2_pos = base2->GetPos();
    double horizontal_arm_length = (base2_pos-base1_pos).Length();
    return horizontal_arm_length;
};

//Get ground force
ChVector3d Probe::GetGroundForce(vehicle::SCMTerrain mterrain) {
    ChVector3d force, torque;
    mterrain.GetContactForceBody(probe_tip, force, torque);
    return force;
}


void Probe::UpdateArmLength(){

    // Remove the old arm and link
    //m_sys->RemoveLink(base_to_arm_link);
    //m_sys->RemoveLink(base2_to_arm_link);
    //m_sys->Remove(arm);
        
    // Calculate the new length for the arm based on the distance between base and probe_tip
    ChVector3d base1_pos = base1->GetPos();
    ChVector3d base2_pos = base2->GetPos();
    ChVector3d probe_tip_pos = probe_tip->GetPos();

    // Rotate the local offset by the base's rotation to get the world position
    //ChVector3d rotated_offset = base1->GetRot().Rotate(local_offset);
    
    //probe_arm = chrono_types::make_shared<ChBodyEasyCylinder>(chrono::ChAxis::Z, .02, probe_length, .1, material);
// Determine the local offset position of the arm relative to the base

    //just move probe body
    
    //ChVector3d probe_local_offset(0, probe_length / 2, 0);

    // Rotate the local offset by the base's rotation to get the world position
    //ChVector3d probe_rotated_offset = base2->GetRot().Rotate(probe_local_offset);

    // Set the position for the new arm: base position plus rotated offset

};