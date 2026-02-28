#include <mutex>
#include <string>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int64.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace chrono;

#ifndef ROSRHEX_H
#define ROSRHEX_H

class ROSRHex : public rclcpp::Node {
public:
    explicit ROSRHex(const std::string &node_name);

    void publish_pose(ChVector3d position, ChQuaterniond quat);
    void publish_joint_angles(std::vector<double>& angles);
    void publish_toe_pos(std::vector<ChVector3d>& positions);
    void publish_truss_lengths(std::vector<double>& lengths);

    std_msgs::msg::Float32MultiArray get_set_joint_pos() const { return set_joint_pos_msg; }
    std_msgs::msg::Float32MultiArray get_set_joint_PID() const { return set_joint_PID_msg; }
    std_msgs::msg::Float32MultiArray get_set_truss_lengths() const { return set_truss_lengths_msg; }

private:

  void set_joint_pos_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg);
  void set_joint_PID_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg);
  void set_truss_lengths_callback(const std::shared_ptr<const std_msgs::msg::Float32MultiArray> msg);

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr toe_pos_pub;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr truss_lengths_pub;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr set_joint_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr set_joint_PID_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr set_truss_lengths_sub_;

  std_msgs::msg::Float32MultiArray set_joint_pos_msg;
  std_msgs::msg::Float32MultiArray set_joint_PID_msg;
  std_msgs::msg::Float32MultiArray set_truss_lengths_msg;
};


#endif