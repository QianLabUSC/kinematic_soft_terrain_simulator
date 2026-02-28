#include "sim_tools/ROSSpirit.h"

  ROSSpirit::ROSSpirit(const std::string &node_name) : Node(node_name){ 
    pose_pub = this->create_publisher<geometry_msgs::msg::Pose>(node_name+"/pose", 10);
    joint_angles_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(node_name+"/joint_angles", 10);
    toe_pos_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(node_name+"/toe_pos", 10);
    truss_lengths_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(node_name+"/truss_lengths", 10);
  
    set_joint_pos_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      node_name+"/set_joint_pos", 10, 
      std::bind(&ROSSpirit::set_joint_pos_callback, this, std::placeholders::_1));

    set_joint_PID_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      node_name+"/set_joint_PID", 10, 
      std::bind(&ROSSpirit::set_joint_PID_callback, this, std::placeholders::_1));

    set_truss_lengths_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      node_name+"/set_truss_lengths", 10, 
      std::bind(&ROSSpirit::set_truss_lengths_callback, this, std::placeholders::_1));
  
  }

  // Method to publish pose value
  void ROSSpirit::publish_pose(ChVector3d position, ChQuaterniond quat)
  {
    auto message = geometry_msgs::msg::Pose();
    message.position.x = position[0];
    message.position.y = position[1];
    message.position.z = position[2];

    message.orientation.w = quat[0];
    message.orientation.x = quat[1];
    message.orientation.y = quat[2];
    message.orientation.z = quat[3];
    pose_pub->publish(message);
  }

  void ROSSpirit::publish_joint_angles(std::vector<double>& angles){
    auto message = std_msgs::msg::Float32MultiArray();
    for(size_t i = 0; i < angles.size(); ++i){
        message.data.push_back(angles[i]);
    }
    joint_angles_pub->publish(message);
  }

 
void ROSSpirit::publish_toe_pos(std::vector<ChVector3d>& positions){
    auto message = std_msgs::msg::Float32MultiArray();
    for(size_t i = 0; i < positions.size(); ++i){
        message.data.push_back(positions[i][0]);
        message.data.push_back(positions[i][1]);
        message.data.push_back(positions[i][2]);
    }
    toe_pos_pub->publish(message);
}

void ROSSpirit::publish_truss_lengths(std::vector<double>& lengths){
    auto message = std_msgs::msg::Float32MultiArray();

    //Do I need this to be a for loop or can I just assign it? Idk. Probably need a float32 array.
    for(size_t i = 0; i < lengths.size(); ++i){
        message.data.push_back(lengths[i]);
    }
    truss_lengths_pub->publish(message);
}

void ROSSpirit::set_joint_pos_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg){set_joint_pos_msg = *msg; }
void ROSSpirit::set_joint_PID_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg){set_joint_PID_msg = *msg; }
void ROSSpirit::set_truss_lengths_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg){set_truss_lengths_msg = *msg; }