#include "sim_tools/Rover.h"

Rover::Rover(ChSystem* sys, std::shared_ptr<chrono::ChBody> body, std::vector<std::shared_ptr<chrono::ChBody>> wheels, std::vector<std::shared_ptr<chrono::ChLinkMotorRotationTorque>> wheel_motors, std::vector<Truss> trusses, int roverNum){
    m_sys = sys;
    m_body = body;
    m_wheels = wheels;
    m_trusses = trusses;
    m_wheel_motors = wheel_motors;
    node = std::make_shared<ROSRover>("rover"+std::to_string(roverNum));

    //Instantiate PID controllers for Rover wheels
    for(int i = 0; i<4; i++){
        pidControllers.emplace_back(100,0,0.001);
    }
};

void Rover::AddMovingPatchesAndCollisions(chrono::vehicle::SCMTerrain mterrain){

    m_body->EnableCollision(true);
    mterrain.AddMovingPatch(m_body, ChVector3d(0, 0, 0), ChVector3d(3, 3, 3));
    
    for(auto wheel : m_wheels){
        wheel->EnableCollision(true);
    }
};

void Rover::DockTruss(int truss_num, std::shared_ptr<ChBody> docked_robot_body, ChVector3d relative_position){
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

    std::cout << "Built truss" << std::endl;

    // Get the link and initialize it correctly
    auto link = std::make_shared<ChLinkLockSpherical>();
    link->Initialize(docked_robot_body, m_trusses[truss_num].coupler, ChFrame<>(coupler_position, QUNIT));

    // Add the link to the system
    m_sys->AddLink(link);
}

void Rover::LockTrussEndPosition(int truss_to_lock, ChVector3d position){
    m_trusses[truss_to_lock].coupler->SetPos(position);

    auto world = chrono_types::make_shared<ChBody>();
    m_sys->AddBody(world);

    world->SetPos(position);
    world->SetFixed(true);

    auto link = std::make_shared<ChLinkLockSpherical>();
    link->Initialize(world, m_trusses[truss_to_lock].coupler, ChFrame<>(position, QUNIT));
    m_sys->AddLink(link);
};

void Rover::SetWheelGains(double kp, double ki, double kd){
    for(int i = 0; i<4; i++){
        pidControllers[i].setGains(kp, ki, kd);
    }
};

void Rover::SetWheelDesiredPosition(int wheel, double setpoint){
    double controlSignal = pidControllers[wheel].calculate(setpoint, m_wheel_motors[wheel]->GetMotorAngle(),m_sys->GetStep());
    m_wheel_motors[wheel]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};

void Rover::SetWheelDesiredVelocity(int wheel, double setpoint){
    double controlSignal = pidControllers[wheel].calculate(setpoint, m_wheel_motors[wheel]->GetMotorAngleDt(),m_sys->GetStep());
    m_wheel_motors[wheel]->SetMotorFunction(chrono_types::make_shared<ChFunctionConst>(controlSignal));
};


void Rover::HandleROS(){
    node->publish_pose(m_body->GetPos(),m_body->GetRot());

}