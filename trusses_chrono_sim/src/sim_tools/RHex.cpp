#include "sim_tools/RHex.h"

RHex::RHex(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> legs, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> leg_motors, std::vector<Truss> trusses, int rhexNum){
    m_sys = sys;
    m_body = body;
    m_legs = legs;
    m_leg_motors = leg_motors;
    m_trusses= trusses;
    node = std::make_shared<ROSRHex>("rhex"+std::to_string(rhexNum));

    //Instantiate PID controllers for RHex legs with moderate gains and limits to prevent overshooting
    for(int i = 0; i<6; i++){
        pidControllers.emplace_back(200, 1, 3, 500.0, M_PI); // Kp=500, Ki=0, Kd=0.5, maxTorque=100, maxSpeed=2π rad/s
    }
};


void RHex::AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain){

    m_body->EnableCollision(true);
    mterrain.AddMovingPatch(m_body, ChVector3d(0, 0, 0), ChVector3d(3, 3, 3));

    for(auto leg : m_legs){
        leg->EnableCollision(true);
    }
};

void RHex::LockTrussEndPosition(int truss_to_lock, ChVector3d position){

    m_trusses[truss_to_lock].coupler->SetPos(position);

    auto world = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(world);

    world->SetPos(position);
    world->SetFixed(true);

    auto link = std::make_shared<ChLinkLockSpherical>();
    link->Initialize(world, m_trusses[truss_to_lock].coupler, ChFrame<>(position, QUNIT));
    m_sys->AddLink(link);
};


void RHex::DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_docking_pos){

};

void RHex::SetLegGains(double kp, double ki, double kd){
    for(int i = 0; i<6; i++){
        pidControllers[i].setGains(kp, ki, kd);
    }
};

void RHex::SetLegDesiredPosition(int leg, double setpoint){
    double controlSignal = pidControllers[leg].calculate(setpoint, m_leg_motors[leg]->GetMotorAngle(),m_sys->GetStep());
    m_leg_motors[leg]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void RHex::SetLegDesiredVelocity(int leg, double setpoint){
    double controlSignal = pidControllers[leg].calculate(setpoint, m_leg_motors[leg]->GetMotorAngleDt(),m_sys->GetStep());
    m_leg_motors[leg]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void RHex::HandleROS(){
    rclcpp::spin_some(node);

    std::vector<double> joint_angles;
    for(size_t i = 0; i < 6; ++i){
        joint_angles.push_back(m_leg_motors[i]->GetMotorAngle());
    }

    std::vector<double> truss_lengths;
    for(size_t i = 0; i < m_trusses.size(); ++i){
        truss_lengths.push_back(m_trusses[i].GetLength());
    }

    node->publish_pose(m_body->GetPos(),m_body->GetRot());
    node->publish_joint_angles(joint_angles);
    node->publish_truss_lengths(truss_lengths);

    auto joint_pos = node->get_set_joint_pos();
    if (!joint_pos.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<6; i++){
            //Apply it
            //SetLegDesiredPosition(i, joint_pos.data[i]);
        }
    } else {
        //std::cout << "No joint position data received yet." << std::endl;
    }

    auto joint_PID = node->get_set_joint_PID();
    if (!joint_PID.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<6; i++){
            //Apply it
            //Fix to be different per leg
            //SetLegGains(joint_PID.data[0], joint_PID.data[1], joint_PID.data[2]);
        }
    } else {
        //std::cout << "No joint PID data received yet." << std::endl;
    }

    auto truss_lengths_msg = node->get_set_truss_lengths();
    if (!truss_lengths_msg.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<truss_lengths_msg.data.size(); i++){
            //m_trusses[i].SetArmDesiredLength(truss_lengths_msg.data[i]);
        }
    } else {
        //std::cout << "No truss length data received yet." << std::endl;
    }

}