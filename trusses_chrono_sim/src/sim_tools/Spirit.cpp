#include "sim_tools/Spirit.h"

Spirit::Spirit(ChSystem* sys, 
               std::shared_ptr<chrono::ChBody> body, 
               std::vector<std::vector<std::shared_ptr<chrono::ChBody>>> leg_bodies, 
               std::vector<std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>>> leg_motors, 
               std::vector<Truss> trusses, 
               std::vector<Probe> probes, 
               int spiritNum) 
    : m_sys(sys), m_body(body), m_leg_bodies(leg_bodies), 
      m_leg_motors(leg_motors), m_trusses(trusses), m_probes(probes) {
    node = std::make_shared<ROSSpirit>("spirit" + std::to_string(spiritNum));

    for (int i = 0; i < 12; i++) {
        pidControllers.emplace_back(50, 0, 0);
    }
};


void Spirit::AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain){

    m_body->EnableCollision(true);
    mterrain.AddMovingPatch(m_body, ChVector3d(0, 0, 0), ChVector3d(3, 3, 3));
    //probe->EnableCollision(true);
    m_leg_bodies[0][3]->EnableCollision(true);
    m_leg_bodies[1][3]->EnableCollision(true);
    m_leg_bodies[2][3]->EnableCollision(true);
    m_leg_bodies[3][3]->EnableCollision(true);

};

std::vector<ChVector3d> Spirit::GetProbeForces(chrono::vehicle::SCMTerrain mterrain) {
    m_probe_forces.clear();
    for (auto& probe : m_probes) {
        m_probe_forces.push_back(probe.GetGroundForce(mterrain));
    }
    return m_probe_forces;
}


void Spirit::LockTrussEndPosition(int truss_to_lock, ChVector3d position){

    m_trusses[truss_to_lock].ConstructTruss(position);
    //m_trusses[truss_to_lock].coupler->SetPos(position);

    auto world = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(world);

    world->SetPos(position);
    world->SetFixed(true);

    auto link = std::make_shared<ChLinkLockSpherical>();
    link->Initialize(world, m_trusses[truss_to_lock].coupler, ChFrame<>(position, QUNIT));
    m_sys->AddLink(link);

};

//move to spirit factory
void Spirit::MakeProbe(ChVector3d xpos, ChVector3d ypos){
    m_probes[0].ConstructProbe();
};


void Spirit::SetAbGains(int leg, double kp, double ki, double kd){
        pidControllers[(leg*3)].setGains(kp, ki, kd);
        std::cout << "Ab Gains: " << kp << std::endl;
};

void Spirit::SetShoulderGains(int leg, double kp, double ki, double kd){
        pidControllers[(leg*3)+1].setGains(kp, ki, kd);
        std::cout << "Shoulder Gains: " << kp << std::endl;
};

void Spirit::SetKneeGains(int leg, double kp, double ki, double kd){
        pidControllers[(leg*3)+2].setGains(kp, ki, kd);
        std::cout << "Knee Gains: " << kp << std::endl;
};

void Spirit::SetAbsDesiredPosition(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)].calculate(setpoint, m_leg_motors[leg][2]->GetMotorAngle(),m_sys->GetStep());
    m_leg_motors[leg][2]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void Spirit::SetShoulderDesiredPosition(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)+1].calculate(setpoint, m_leg_motors[leg][1]->GetMotorAngle(),m_sys->GetStep());
    m_leg_motors[leg][1]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void Spirit::SetKneeDesiredPosition(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)+2].calculate(-setpoint, m_leg_motors[leg][0]->GetMotorAngle(),m_sys->GetStep());
    m_leg_motors[leg][0]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void Spirit::DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_position){
    ChQuaternion<> relative_orientation = docked_robot_body->GetRot();
    
    std::cout << "Relative Position: "<<relative_position[0]  << ", " << relative_position[1] << ", "<< relative_position[2] << std::endl;
    std::cout  << "Relative Orientation: "<< relative_orientation.e0()  << ", " << relative_orientation.e1() << ", "<< relative_orientation.e2() << ", "<< relative_orientation.e3() << std::endl;
    ChVector3d rotated_position = relative_orientation.Rotate(relative_position);
    std::cout <<"Rotated Position: " <<rotated_position[0]  << ", " << rotated_position[1] << ", "<< rotated_position[2] << std::endl;
    //m_trusses[truss_num].ConstructTruss(ChVector3d(docked_robot_body->GetPos()[0]+relative_position[0],docked_robot_body->GetPos()[1]+relative_position[1],docked_robot_body->GetPos()[2]+relative_position[2]));
    
    ChVector3d coupler_position = docked_robot_body->GetPos() + rotated_position;
    std::cout << "Body Position" << docked_robot_body->GetPos()[0]  << ", " << docked_robot_body->GetPos()[1] << ", "<< docked_robot_body->GetPos()[2] << std::endl;
    std::cout << "Coupler Position" <<coupler_position[0]  << ", " << coupler_position[1] << ", "<< coupler_position[2] << std::endl;

    m_trusses[truss_num].ConstructTruss(coupler_position);

    // Get the link and initialize it correctly
    auto link = std::make_shared<ChLinkLockSpherical>();
    link->Initialize(docked_robot_body, m_trusses[truss_num].coupler, ChFrame<>(coupler_position, QUNIT));

    // Add the link to the system
    m_sys->AddLink(link);
}


void Spirit::HandleROS(){
    
    rclcpp::spin_some(node);

    std::vector<ChVector3d> toe_positions;
    toe_positions.push_back(m_leg_bodies[0][3]->GetPos());
    toe_positions.push_back(m_leg_bodies[1][3]->GetPos());
    toe_positions.push_back(m_leg_bodies[2][3]->GetPos());
    toe_positions.push_back(m_leg_bodies[3][3]->GetPos());

    std::vector<double> joint_angles;
    for(size_t i = 0; i < 4; ++i){
        joint_angles.push_back(m_leg_motors[i][2]->GetMotorAngle());
        joint_angles.push_back(m_leg_motors[i][1]->GetMotorAngle());
        joint_angles.push_back(m_leg_motors[i][0]->GetMotorAngle());
    }

    node->publish_pose(m_body->GetPos(), m_body->GetRot());
    node->publish_toe_pos(toe_positions);
    node->publish_joint_angles(joint_angles);

    std::vector<double> truss_lengths;
    for(size_t i = 0; i < m_trusses.size(); ++i){
        truss_lengths.push_back(m_trusses[i].GetLength());
    }
    node->publish_truss_lengths(truss_lengths);

    auto joint_pos = node->get_set_joint_pos();
    if (!joint_pos.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<4; i++){
            //SetAbsDesiredPosition(i, joint_pos.data[(3*i)]);
            //SetShoulderDesiredPosition(i, joint_pos.data[(3*i)+1]);
            //SetKneeDesiredPosition(i, joint_pos.data[(3*i)+2]);
        }
    } else {
        //std::cout << "No joint position data received yet." << std::endl;
    }

    auto joint_PID = node->get_set_joint_PID();
    if (!joint_PID.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<4; i++){
            SetAbGains(i, joint_PID.data[(3*i)], 0, 0);
            SetShoulderGains(i, joint_PID.data[(3*i)+1], 0, 0);
            SetKneeGains(i, joint_PID.data[(3*i)+2], 0, 0);
        }
    } else {
        //std::cout << "No joint PID data received yet." << std::endl;
    }

    auto truss_lengths_msg = node->get_set_truss_lengths();
    if (!truss_lengths_msg.data.empty()) {  // Check if the data array is not empty
        //Handle the message
        for (size_t i=0; i<truss_lengths_msg.data.size(); i++){
            m_trusses[i].SetArmDesiredLength(truss_lengths_msg.data[i]);
        }
    } else {
        //std::cout << "No truss length data received yet." << std::endl;
    }
    //node->publish_probe_forces(m_probe_forces);

};
/*
Spirit::Spirit(ChSystem* sys, std::string package_filepath, int robot_num, int truss_num, double dt){
    m_package_filepath = package_filepath;
    m_suffix = "_spirit"+std::to_string(robot_num);
    m_robot_num = robot_num;
    m_sys = sys;
    m_truss_num = truss_num;
    m_dt = dt;
};

void Spirit::SpawnRobot(ChVector3d position=ChVector3d(0,0,0),ChQuaternion<> orientation=ChQuaternion<>(.707107,.707107,0,0)){

    //Logic to determine which URDF to load
    if(m_truss_num == 0){
        m_robot= new ChParserURDF(m_package_filepath+"URDF_files/spirit_"+std::to_string(m_robot_num)+".urdf");
    }else if(m_truss_num == 1){
        m_robot= new ChParserURDF(m_package_filepath+"URDF_files/spirit_1_truss_"+std::to_string(m_robot_num)+".urdf");
    }else if(m_truss_num == 2){
        m_robot= new ChParserURDF(m_package_filepath+"URDF_files/spirit_2_truss_"+std::to_string(m_robot_num)+".urdf");
    }else if(m_truss_num == 3){
        m_robot= new ChParserURDF(m_package_filepath+"URDF_files/spirit_3_truss_"+std::to_string(m_robot_num)+".urdf");
    }else{
        throw std::runtime_error("URDF for that truss number not created yet");
    }
    m_robot->SetRootInitPose(ChFrame(position,orientation)); //Spawn in the robot with the given position and orientation
    m_robot->SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION); //Set the joints to be static at 0 to begin with

    //Instantiate PID controllers for Spirit legs and Truss prismatic joints
    for(int i = 0; i<(12+m_truss_num); i++){
        pidControllers.emplace_back(10,0,2);
    }

    //Set the collision types for the bodies we want to contact with the ground. NODE_CLOUD reduces bouncing on the legs
    m_robot->SetBodyMeshCollisionType("body"+m_suffix, ChParserURDF::MeshCollisionType::CONVEX_HULL);
    m_robot->SetBodyMeshCollisionType("toe0"+m_suffix, ChParserURDF::MeshCollisionType::NODE_CLOUD);
    m_robot->SetBodyMeshCollisionType("toe2"+m_suffix, ChParserURDF::MeshCollisionType::NODE_CLOUD);
    m_robot->SetBodyMeshCollisionType("toe3"+m_suffix, ChParserURDF::MeshCollisionType::NODE_CLOUD);

};

//TODO: These joints are minimally tested...
void Spirit::EnableAbs(double kp, double ki, double kd){
    m_robot->SetJointActuationType("8"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[0].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("9"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[3].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("10"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[6].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("11"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[9].setGains(kp, ki, kd);
};

void Spirit::Populate(){
    m_robot->PopulateSystem(*m_sys);
};

void Spirit::EnableShoulders(double kp, double ki, double kd){
    m_robot->SetJointActuationType("0"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[1].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("2"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[4].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("4"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[7].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("6"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[10].setGains(kp, ki, kd);
};

void Spirit::EnableKnees(double kp, double ki, double kd){
    m_robot->SetJointActuationType("1"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[2].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("3"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[5].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("5"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[8].setGains(kp, ki, kd);

    m_robot->SetJointActuationType("7"+m_suffix,ChParserURDF::ActuationType::FORCE);
    pidControllers[11].setGains(kp, ki, kd);

};

void Spirit::EnableTrusses(double kp, double ki, double kd){
    for(int i = 0; i < m_truss_num; i++){

        m_robot->SetJointActuationType("body_to_arm"+std::to_string(i)+"_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(i)+"_base_to_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(i)+"_base_to_twist"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(i)+"_twist_to_extension"+m_suffix,ChParserURDF::ActuationType::FORCE);
        pidControllers[12+i].setGains(kp,ki,kd);

        m_robot->SetJointActuationType("arm"+std::to_string(i)+"_extension_to_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(i)+"_extension_turret_to_block"+m_suffix,ChParserURDF::ActuationType::FORCE);

    }
};

void Spirit::EnableTruss(int truss_to_enable, double kp, double ki, double kd){
        m_robot->SetJointActuationType("body_to_arm"+std::to_string(truss_to_enable)+"_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(truss_to_enable)+"_base_to_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(truss_to_enable)+"_base_to_twist"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(truss_to_enable)+"_twist_to_extension"+m_suffix,ChParserURDF::ActuationType::FORCE);
        pidControllers[12+truss_to_enable].setGains(kp,ki,kd);

        m_robot->SetJointActuationType("arm"+std::to_string(truss_to_enable)+"_extension_to_turret"+m_suffix,ChParserURDF::ActuationType::FORCE);

        m_robot->SetJointActuationType("arm"+std::to_string(truss_to_enable)+"_extension_turret_to_block"+m_suffix,ChParserURDF::ActuationType::FORCE);
};

void Spirit::SetTrussEndPosition(int truss_to_set, ChVector3d position){
    auto arm_block = m_robot->GetChBody("arm"+std::to_string(truss_to_set)+"_block"+m_suffix);
    arm_block->SetPos(position);
    arm_block->SetFixed(true);
};

void Spirit::DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChSystemSMC& sys){
    // Get the arm block body as a shared pointer
    auto arm_block = m_robot->GetChBody("arm" + std::to_string(truss_num) + "_block" + m_suffix);

    // Set the position of the arm block
    arm_block->SetPos(ChVector3d(docked_robot_body->GetPos()[0],docked_robot_body->GetPos()[1]+.5,docked_robot_body->GetPos()[2]));

    // Get the link and initialize it correctly
    auto link = std::make_shared<ChLinkLockLock>();
    link->Initialize(docked_robot_body, arm_block, ChFrame<>(ChVector3d(docked_robot_body->GetPos()[0],docked_robot_body->GetPos()[1]+.5,docked_robot_body->GetPos()[2]), QuatFromAngleAxis(CH_PI, VECT_X)));

    // Add the link to the system
    sys.AddLink(link);
}

void Spirit::AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain){

    body = m_robot->GetChBody("body"+m_suffix);
    body->EnableCollision(true);
    mterrain.AddMovingPatch(body, ChVector3d(0, 0, 0), ChVector3d(0.1, .1, .1));

    toe0 = m_robot->GetChBody("toe0"+m_suffix);
    toe0->EnableCollision(true);
    mterrain.AddMovingPatch(toe0, ChVector3d(0, 0, 0), ChVector3d(0.05, .05, .05));

    toe1 = m_robot->GetChBody("toe1"+m_suffix);
    toe1->EnableCollision(true);
    mterrain.AddMovingPatch(toe1, ChVector3d(0, 0, 0), ChVector3d(0.05, .05, .05));

    toe2 = m_robot->GetChBody("toe2"+m_suffix);
    toe2->EnableCollision(true);
    mterrain.AddMovingPatch(toe2, ChVector3d(0, 0, 0), ChVector3d(0.05, .05, .05));

    toe3 = m_robot->GetChBody("toe3"+m_suffix);
    toe3->EnableCollision(true);
    mterrain.AddMovingPatch(toe3, ChVector3d(0, 0, 0), ChVector3d(0.05, .05, .05));
};

void Spirit::SetAbsPositionPID(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)].calculate(setpoint, getMotorRotationAngle(m_robot->GetChMotor(std::to_string(8+leg)+m_suffix)),m_dt);
    m_robot->SetMotorFunction(std::to_string((8+leg))+m_suffix,chrono_types::make_shared<ChFunctionConst>(-controlSignal));
}

void Spirit::SetShoulderPositionPID(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)+1].calculate(setpoint, getMotorRotationAngle(m_robot->GetChMotor(std::to_string(leg*2)+m_suffix)),m_dt);
    m_robot->SetMotorFunction(std::to_string(leg*2)+m_suffix,chrono_types::make_shared<ChFunctionConst>(-controlSignal));
}

void Spirit::SetKneePositionPID(int leg, double setpoint){
    double controlSignal = pidControllers[(leg*3)+2].calculate(setpoint, getMotorRotationAngle(m_robot->GetChMotor(std::to_string((leg*2)+1)+m_suffix)),m_dt);
    m_robot->SetMotorFunction(std::to_string((leg*2)+1)+m_suffix,chrono_types::make_shared<ChFunctionConst>(-controlSignal));
}

double Spirit::getMotorRotationAngle(const std::shared_ptr<chrono::ChLinkMotor> motor){
    chrono::ChQuaternion<double> quat1ch = motor->GetFrame1Abs().GetRot();
    chrono::ChQuaternion<double> quat2ch = motor->GetFrame2Abs().GetRot();
            
    Eigen::Quaterniond q1 = Eigen::Quaterniond(quat1ch.e0(),quat1ch.e1(),quat1ch.e2(),quat1ch.e3());
    Eigen::Quaterniond q2 = Eigen::Quaterniond(quat2ch.e0(),quat2ch.e1(),quat2ch.e2(),quat2ch.e3());
    Eigen::Quaterniond delta_q = q1.conjugate() * q2;
    Eigen::AngleAxisd aa(delta_q);
    return aa.angle() * aa.axis().dot(Eigen::Vector3d::UnitZ());
}

double Spirit::getTrussLength(int trussNum){
    chrono::ChVector3d pos1 = m_robot->GetChBody("arm"+std::to_string(trussNum)+"_base_turret"+m_suffix)->GetPos();
    chrono::ChVector3d pos2 = m_robot->GetChBody("arm"+std::to_string(trussNum)+"_extension_turret"+m_suffix)->GetPos();

    Eigen::Vector3d vector1(pos1.x(),pos1.y(),pos1.z());
    Eigen::Vector3d vector2(pos2.x(),pos2.y(),pos2.z());

    Eigen::Vector3d difference = vector2- vector1;

    double length = difference.norm();

    return length;
}

*/